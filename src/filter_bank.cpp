#include "filter_bank.h"

#include <algorithm>

namespace curves {

void FilterBank::prepare(int num_bands, double sample_rate, int num_channels) {
    num_bands_ = num_bands;
    fs_ = sample_rate;
    fc_.assign(num_bands, 1000.0);
    a1_.assign(num_bands, 0.0);
    a2_.assign(num_bands, 0.0);
    a3_.assign(num_bands, 0.0);
    g_.assign(num_bands, 1.0);
    target_.assign(num_bands, 1.0);
    s1_.assign(num_channels, std::vector<double>(num_bands, 0.0));
    s2_.assign(num_channels, std::vector<double>(num_bands, 0.0));
    steady_env_.assign(num_channels, std::vector<double>(num_bands, 0.0));
    recompute_coefficients();
    set_smoothing(30.0);
    set_steady(0.0, 0.25);
}

void FilterBank::set_band_frequency(int band, double fc) {
    fc_[band] = fc;
    recompute_coefficients();
}

void FilterBank::set_smoothing(double tau_ms) {
    // One-pole coefficient: alpha = 1 - exp(-1 / (tau * fs)). This is the
    // entire smoothing system — fixed math, no state to corrupt.
    const double tau = tau_ms * 1e-3;
    alpha_ = 1.0 - std::exp(-1.0 / (tau * fs_));
}

void FilterBank::set_steady(double amount, double threshold) {
    steady_amount_ = std::clamp(amount, 0.0, 1.0);
    steady_threshold_ = std::max(threshold, 1e-4);
    // Follower time constants: fast attack (5 ms), slow release (150 ms).
    atk_alpha_ = 1.0 - std::exp(-1.0 / (0.005 * fs_));
    rel_alpha_ = 1.0 - std::exp(-1.0 / (0.150 * fs_));
}

void FilterBank::set_targets(const double* linear_gains) {
    for (int b = 0; b < num_bands_; ++b)
        target_[b] = linear_gains[b];
}

void FilterBank::recompute_coefficients() {
    // TPT SVF with fixed g (cutoff) and k = 1/Q. Q = 1.0 for M1 (broad,
    // EQ-like); per-node Q arrives with the M2+ parameter surface.
    constexpr double k = 1.0;
    for (int b = 0; b < num_bands_; ++b) {
        const double g = std::tan(fc_[b] * 3.14159265358979323846 / fs_);
        const double a1 = 1.0 / (1.0 + g * (g + k));
        a1_[b] = a1;
        a2_[b] = g * a1;
        a3_[b] = g * a2_[b];
    }
}

}  // namespace curves
