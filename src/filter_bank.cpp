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

void FilterBank::set_overlap_correction(double amount) {
    overlap_amt_ = std::clamp(amount, 0.0, 1.0);
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
    if (overlap_amt_ > 0.0 && num_bands_ > 1) {
        const double k = overlap_factor(fc_[num_bands_ / 2]);
        if (k > 1e-6)
            overlap_k_ = std::pow(k, overlap_amt_);
    }
    ok_ = k_ / overlap_k_;
    rebuild_response_kernel();
    build_inverse_kernel();
}

// The kernel for response_curve(). Hn depends only on the lag because the
// bands are geometrically spaced: u = f_i/f_b = ratio^(i-b).
//
// The window is the FULL band count, and that is not laziness -- a narrower
// one was tried and measured wrong. Sizing it by bandwidth-in-bands (7x the
// -3 dB width, 12 bands at matched Q) left 3.95 dB of error at 128 bands and
// 6.46 dB at 512, barely better than the nominal curve it replaces.
//
// The reason: the REAL part of Hn falls off as (BW/lag)^2 and truncates
// happily, but the IMAGINARY part falls off as k/u -- geometric in the
// frequency RATIO, not in bandwidths. At matched Q on 512 bands, u only
// reaches 1.16 twelve bands out, so im is still ~4% there, and it is
// precisely the quadrature term that dominates a cut (see section I). A
// window wide enough for it works out at several hundred bands, which is
// most of the map anyway.
//
// So: exact, and pay for it. Measured 193 us per call at 512 bands, 12 us
// at 128 -- 1.16% of a core at the 60 Hz publish rate, worst case.
void FilterBank::rebuild_response_kernel() {
    if (num_bands_ < 2) {
        resp_w_ = 0;
        kern_re_.assign(1, 1.0 / overlap_k_);
        kern_im_.assign(1, 0.0);
        return;
    }
    const double ratio = std::pow(fc_[num_bands_ - 1] / fc_[0],
                                  1.0 / double(num_bands_ - 1));
    resp_w_ = num_bands_ - 1;
    kern_re_.assign(2 * resp_w_ + 1, 0.0);
    kern_im_.assign(2 * resp_w_ + 1, 0.0);
    const double inv_k = 1.0 / overlap_k_;
    for (int lag = -resp_w_; lag <= resp_w_; ++lag) {
        const double u = std::pow(ratio, double(lag));
        const double ku = k_ * u;
        const double d = 1.0 - u * u;
        const double den = d * d + ku * ku;
        if (den <= 0.0) continue;
        kern_re_[lag + resp_w_] = (ku * ku / den) * inv_k;
        kern_im_[lag + resp_w_] = (ku * d / den) * inv_k;
    }
}

// Build the inverse of the REAL part of the response kernel.
//
// Matching the real part rather than the magnitude is deliberate: the
// response is 1 + Re + jIm, the imaginary parts of the neighbours cancel on
// a symmetric log map, and what survives only matters where 1 + Re is near
// zero -- which the gain floor already keeps us away from. Matching |1+S|
// exactly would make this a nonlinear solve for no measurable gain.
void FilterBank::build_inverse_kernel() {
    inv_w_ = std::min(num_bands_ - 1, 64);
    if (inv_w_ < 1) {
        inv_re_.assign(1, 1.0);
        inv_residual_ = 0.0;
        return;
    }
    const int w = inv_w_, m = 2 * w + 1;
    // Truncated forward kernel. Its lags sum to 1 by construction (the 1/K
    // in rebuild_response_kernel is exactly that normalisation), so the
    // convolution operator's largest eigenvalue is 1 at DC and Landweber
    // is stable for a step below 2.
    std::vector<double> kf(m, 0.0);
    for (int lag = -w; lag <= w; ++lag) {
        const int src = lag + resp_w_;
        if (src >= 0 && src < (int)kern_re_.size())
            kf[lag + w] = kern_re_[src];
    }
    std::vector<double> inv(m, 0.0), conv(m, 0.0);
    inv[w] = 1.0;                       // start from an impulse
    constexpr int kIters = 40;
    constexpr double kStep = 1.0;
    for (int it = 0; it < kIters; ++it) {
        for (int i = 0; i < m; ++i) {
            double acc = 0.0;
            for (int j = 0; j < m; ++j) {
                const int k = i - j + w;
                if (k >= 0 && k < m)
                    acc += inv[j] * kf[k];
            }
            conv[i] = acc;
        }
        for (int i = 0; i < m; ++i) {
            const double want = (i == w) ? 1.0 : 0.0;
            inv[i] += kStep * (want - conv[i]);
        }
    }
    double res = 0.0;
    for (int i = 0; i < m; ++i) {
        const double want = (i == w) ? 1.0 : 0.0;
        res = std::max(res, std::abs(conv[i] - want));
    }
    inv_residual_ = res;
    inv_re_.swap(inv);
}

void FilterBank::predistort(const double* big_in, double* big_out) const {
    const int w = inv_w_;
    if (w < 1 || (int)inv_re_.size() != 2 * w + 1) {
        for (int i = 0; i < num_bands_; ++i) big_out[i] = big_in[i];
        return;
    }
    for (int i = 0; i < num_bands_; ++i) {
        double acc = 0.0;
        const int lo = std::max(-w, i - (num_bands_ - 1));
        const int hi = std::min(w, i);
        for (int lag = lo; lag <= hi; ++lag)
            acc += inv_re_[lag + w] * big_in[i - lag];
        big_out[i] = acc;
    }
}

void FilterBank::response_curve(const double* big, double scale,
                                double* out_re, double* out_im) const {
    for (int i = 0; i < num_bands_; ++i) {
        double re = 0.0, im = 0.0;
        const int lo = std::max(-resp_w_, i - (num_bands_ - 1));
        const int hi = std::min(resp_w_, i);
        for (int lag = lo; lag <= hi; ++lag) {
            const double c = big[i - lag];
            re += kern_re_[lag + resp_w_] * c;
            im += kern_im_[lag + resp_w_] * c;
        }
        out_re[i] = re * scale;
        out_im[i] = im * scale;
    }
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
