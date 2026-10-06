#include "filter_bank.h"

#include <algorithm>

namespace curves {

void FilterBank::prepare(int num_bands, double sample_rate, int num_channels) {
    num_bands_ = num_bands;
    fs_ = sample_rate;
    fc_.assign(num_bands, 1000.0);
    gt_.assign(num_bands, 0.0);
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
    refresh_overlap();
    set_steady(0.0, 0.25);
}

void FilterBank::set_band_frequency(int band, double fc) {
    fc_[band] = fc;
    gt_[band] = std::tan(fc * 3.14159265358979323846 / fs_);
    recompute_coefficients();
    refresh_overlap();
}

void FilterBank::set_q(double q) {
    q_ = std::max(q, 0.05);
    k_ = 1.0 / q_;
    recompute_coefficients();
    refresh_overlap();
}

void FilterBank::set_overlap_correction(bool on) {
    overlap_on_ = on;
    refresh_overlap();
}

// Sum of the Q-NORMALISED bandpass responses at f. The TPT SVF realises the
// analog prototype exactly in the warped variable u = tan(pi*f/fs)/g_b, so
// this is the true digital response, not an analog approximation:
//
//     Hn(ju) = j*k*u / (1 - u^2 + j*k*u)      (peak 1 at u = 1, any Q)
//
// out = x + sum_b Hn_b*(g_b - 1), so for a flat g this sum IS the factor the
// per-band depth gets multiplied by.
double FilterBank::overlap_factor(double f) const {
    const double t = std::tan(std::min(f, 0.49 * fs_) * 3.14159265358979323846 / fs_);
    double re = 0.0;
    for (int b = 0; b < num_bands_; ++b) {
        if (gt_[b] <= 0.0) continue;
        const double u = t / gt_[b];
        const double ku = k_ * u;
        // j*ku / (d + j*ku) with d = 1 - u^2
        const double d = 1.0 - u * u;
        const double den = d * d + ku * ku;
        if (den <= 0.0) continue;
        // Hn = j*ku/(d + j*ku), d = 1 - u^2. Re is always >= 0 and the
        // ON-CENTRE band contributes exactly 1, so a sum below 1 means the
        // probe missed a centre. The imaginary parts of the neighbours
        // carry OPPOSITE signs above and below f (d flips with u) and
        // cancel on a symmetric log map, which is why a real scalar works.
        re += ku * ku / den;
    }
    return re;
}

// K is read AT A BAND CENTRE -- specifically the middle band. Probing the
// geometric centre of the RANGE instead put the probe between two bands,
// which at Q = 1 is invisible but at Q = 159 read the gap (0.04) rather
// than the peak (1.01). The sum is flat across the interior (measured
// 1.68 / 1.69 / 1.66 over two decades at 128 bands), so one centre is
// enough; the outermost bands roll off because they have no neighbours
// beyond the map, which is correct and must not be corrected away.
void FilterBank::refresh_overlap() {
    overlap_k_ = 1.0;
    if (overlap_on_ && num_bands_ > 1) {
        const double k = overlap_factor(fc_[num_bands_ / 2]);
        if (k > 1e-6) overlap_k_ = k;
    }
    ok_ = k_ / overlap_k_;
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
    // TPT SVF, k = 1/Q. tan() is cached in gt_ because it depends only on
    // fc -- so changing Q is 1 divide + 3 multiplies per band, which is what
    // makes string-DRIVEN Q affordable later (O4).
    const double k = k_;
    for (int b = 0; b < num_bands_; ++b) {
        const double g = gt_[b];
        const double a1 = 1.0 / (1.0 + g * (g + k));
        a1_[b] = a1;
        a2_[b] = g * a1;
        a3_[b] = g * a2_[b];
    }
}

}  // namespace curves
