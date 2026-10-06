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
    ramp_.assign(num_bands, 1.0);
    g_eff_.assign(num_bands, 1.0);
    gs_.assign(num_bands, 1.0);
    gs_eff_.assign(num_bands, 1.0);
    ramp_s_.assign(num_bands, 1.0);
    pan_.assign(num_bands, 0.0);
    pan_step_.assign(num_bands, 0.0);
    spatial_ = false;
    steady_env_.assign(num_bands, 0.0);
    s1_.assign(std::max(num_channels, 2), std::vector<double>(num_bands, 0.0));
    s2_.assign(std::max(num_channels, 2), std::vector<double>(num_bands, 0.0));
    recompute_coefficients();
    set_steady(0.0, 0.25);
}

void FilterBank::set_band_frequency(int band, double fc) {
    fc_[band] = fc;
    recompute_coefficients();
}

void FilterBank::set_steady(double amount, double threshold) {
    steady_amount_ = std::clamp(amount, 0.0, 1.0);
    steady_threshold_ = std::max(threshold, 1e-4);
    // Follower time constants: fast attack (5 ms), slow release (150 ms).
    atk_alpha_ = 1.0 - std::exp(-1.0 / (0.005 * fs_));
    rel_alpha_ = 1.0 - std::exp(-1.0 / (0.150 * fs_));
}

void FilterBank::set_block_targets(const double* linear_gains, int num_frames) {
    // Geometric step: after num_frames applications of g *= ramp, g lands on
    // the target exactly, and every intermediate value is linear in dB. That
    // is what keeps Range dB out of the gain's transient shape — Range is
    // applied once, upstream, by a memoryless map.
    const double inv_n = 1.0 / static_cast<double>(std::max(1, num_frames));
    for (int b = 0; b < num_bands_; ++b) {
        const double g0 = std::max(g_[b], 1e-9);
        const double g1 = std::max(linear_gains[b], 1e-9);
        ramp_[b] = std::pow(g1 / g0, inv_n);
    }
}

void FilterBank::set_spatial_targets(const double* side_gains,
                                     const double* pans, int num_frames) {
    const double inv_n = 1.0 / static_cast<double>(std::max(1, num_frames));
    for (int b = 0; b < num_bands_; ++b) {
        const double g0 = std::max(gs_[b], 1e-9);
        const double g1 = std::max(side_gains[b], 1e-9);
        ramp_s_[b] = std::pow(g1 / g0, inv_n);
        pan_step_[b] = (pans[b] - pan_[b]) * inv_n;
    }
    spatial_ = true;
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
