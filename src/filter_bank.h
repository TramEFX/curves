// Curves — string-driven filter bank.
//
// The subscriber side of the architecture: N TPT/SVF peaking filters, one
// per string node, whose gains are pushed BY THE STRING.
//
// The bank is deliberately DUMB about time. It holds no smoothing filter:
// the string's control signal is smoothed upstream, in the log (dB) domain
// where the string lives, and the bank is handed a per-block target plus a
// geometric ramp to walk there. See PluginProcessor for why that matters —
// briefly, a one-pole on the LINEAR gain is not scale-invariant, so Range dB
// leaked into the transient shape of the EQ and made boosts and cuts
// asymmetric (measured 2.21x at Range 18).
//
// ONE INSTANCE IS ONE STRING. Everything here is channel-coherent: both
// channels get an identical gain and an identical Steady duck, derived from
// a mono detector. Stereo interest is produced by running separate
// INSTANCES on L and R, not by letting one instance drift apart internally
// — a process that ducks L and R differently is a dynamic image-mover.
// (Same reasoning the OutputLeveler already uses for its control signal.)
//
// Zavalishin TPT SVF, peaking configuration: out = x + bandpass * (G - 1).
// JUCE-free (portable, testable); the plugin adapter lives in src/plugin.

#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace curves {

class FilterBank {
public:
    void prepare(int num_bands, double sample_rate, int num_channels);

    // Fixed band center frequencies; caller owns the frequency map.
    void set_band_frequency(int band, double fc);

    // Uniform Q for every band. Q = 1.0 is the historical fixed value and
    // is BIT-IDENTICAL to the pre-Q bank (the normalisation below multiplies
    // by exactly 1.0). Matched-to-spacing Q is ~0.156*N -- see q_test.
    void set_q(double q);
    double q() const { return q_; }

    // Band OVERLAP correction (B1). The bands add, so a nominal g arrives as
    // 1 + K*(g-1). With Q = 1, K ~= 0.152*N -- it SCALES WITH NODE COUNT,
    // which is what made every per-band depth nominal rather than actual.
    // At Q matched to the band spacing K collapses to a CONSTANT ~1.70, and
    // one scalar removes it. K is computed analytically from the band map,
    // not fitted. Enabling this changes no display code: the correction is
    // folded into the audio-path scalar, so gains() keeps reporting the
    // nominal value, which is now also the actual response. WYSIWYG holds
    // structurally rather than by remembering to mirror it.
    // amount 0 = no correction (the historical bank), 1 = fully corrected.
    // Blended as K^amount so BOTH endpoints are exact and the middle is a
    // smooth morph: snapping the correction on at the first touch of the
    // knob would jump the level by up to 96x.
    void set_overlap_correction(double amount);
    double overlap_k() const { return overlap_k_; }
    double overlap_corrected() const { return overlap_amt_; }
    // How much of the overlap is still UNCORRECTED, as a factor. 1 = none
    // left. The canvas widens its dB axis by this, so a partly corrected
    // bank gets a partly widened axis.
    double overlap_k_residual() const {
        if (num_bands_ < 2) return 1.0;
        const double k = overlap_factor(fc_[num_bands_ / 2]);
        return (k > 1e-6) ? k / overlap_k_ : 1.0;
    }
    // Sum of the Q-normalised bandpasses at f -- the overlap factor there.
    double overlap_factor(double f) const;

    // THE TRUE RESPONSE of the summed bank at every band centre, given a
    // per-band excess (g_b - 1). This is what the audio actually does:
    //
    //     out/in at f_i  =  1 + sum_b Hn_b(f_i) * big_b / K
    //
    // The drawn curve used big_i alone, which is the NOMINAL per-band gain
    // and never was the response -- measured 31 dB out at 128 bands and
    // 43 dB at 512 (q_test section I). Drawing this instead makes WYSIWYG
    // true by construction at any Q, rather than only approaching truth as
    // Q rises.
    //
    // Cheap because the bands are GEOMETRICALLY spaced and Q is uniform, so
    // Hn_b(f_i) depends only on the lag i-b: the whole curve is one
    // convolution with a kernel that changes only when Q or the band map
    // does. No transcendentals per point, and `scale` folds in Mix.
    //
    // out_re/out_im must hold num_bands each. The caller adds the 1 (or
    // not: placement is a cross-feed, not a gain, so it has no unity term).
    void response_curve(const double* big, double scale,
                        double* out_re, double* out_im) const;
    int response_window() const { return resp_w_; }

    // Per-band dynamic EQ ("Steady"): bands whose bandpass energy exceeds
    // the threshold get their own contribution squeezed. amount 0 = off.
    // Threshold is linear amplitude on the bandpass signal.
    void set_steady(double amount, double threshold);

    // Spatial (stereo) subscriber: per-band side gain and mid->side pan for
    // the coming chunk. Call set_spatial_off() to skip the stage entirely.
    void set_spatial_targets(const double* side_gains, const double* pans,
                             int num_frames);
    void set_spatial_off() { spatial_ = false; }

    // Target linear gains for the coming block, and how many frames to take
    // getting there. The walk is GEOMETRIC (g *= r), which is exactly linear
    // in dB for one multiply per band per frame. A linear ramp in gain was
    // measured at up to 24 dB of mid-block error at Smoothing 2 ms.
    void set_block_targets(const double* linear_gains, int num_frames);

    // Truth for the display: effective per-band linear gain, INCLUDING the
    // Steady duck. This is what the EQ is applying right now.
    const std::vector<double>& gains() const { return g_eff_; }
    // The SIDE equivalent. With the EQ gated off the mid gain is flat and
    // carries no shape, but the spatial subscriber is still driving this --
    // so this is what the curve must be drawn from in that case.
    const std::vector<double>& side_gains() const { return gs_eff_; }
    // And placement. With the EQ off AND width at zero, this is the only
    // carrier left holding the shape -- the mid and side gains are both
    // flat. Nothing read it, so the curve vanished a second time.
    const std::vector<double>& pans() const { return pan_; }
    bool spatial_active() const { return spatial_; }

    // One sample FRAME. The gain advances exactly once here, not once per
    // channel — the previous per-channel entry point double-stepped the
    // smoother, so stereo ran at half the set smoothing time and a mono
    // instance did not match a stereo one.
    // One sample FRAME, in MID/SIDE.
    //
    // Running the bank on M/S rather than L/R is MATHEMATICALLY IDENTICAL
    // while mid and side carry the same gain (verified: max difference
    // 3.3e-16 against the L/R form), and it makes stereo almost free -- the
    // SVF count is unchanged and the M/S conversion is 4 ops per FRAME
    // rather than per band. Three per-band coefficients fall out:
    //
    //   gm   mid gain      -- the EQ
    //   gs   side gain     -- width is gs/gm, i.e. their difference in dB
    //   pan  mid->side     -- placement, S += pan*v1m
    //
    // All three come from the control surface and are drawable, which is
    // what the image-provenance invariant requires.
    inline void process_frame(float& l, float& r, bool stereo) {
        const bool steady_on = steady_amount_ > 0.0;
        const double xl = l, xr = r;
        const double xm = 0.5 * (xl + xr);
        const double xs = stereo ? 0.5 * (xl - xr) : 0.0;
        double ym = xm, ys = xs;
        for (int b = 0; b < num_bands_; ++b) {
            const double gm = g_[b] * ramp_[b];
            g_[b] = gm;
            double bigm = gm - 1.0;

            // --- mid band ---
            double s1 = s1_[0][b], s2 = s2_[0][b];
            double v3 = xm - s2;
            const double v1m = a1_[b] * s1 + a2_[b] * v3;
            double v2 = s2 + a2_[b] * s1 + a3_[b] * v3;
            s1_[0][b] = 2.0 * v1m - s1;
            s2_[0][b] = 2.0 * v2 - s2;

            // --- side band ---
            double v1s = 0.0;
            if (stereo) {
                s1 = s1_[1][b];
                s2 = s2_[1][b];
                v3 = xs - s2;
                v1s = a1_[b] * s1 + a2_[b] * v3;
                v2 = s2 + a2_[b] * s1 + a3_[b] * v3;
                s1_[1][b] = 2.0 * v1s - s1;
                s2_[1][b] = 2.0 * v2 - s2;
            }

            double bigs = bigm;
            double pan = 0.0;
            if (spatial_) {
                const double gs = gs_[b] * ramp_s_[b];
                gs_[b] = gs;
                bigs = gs - 1.0;
                pan_[b] += pan_step_[b];
                pan = pan_[b];
            }

            if (steady_on) {
                // Detector is the MID bandpass -- already mono by
                // construction now. The duck multiplies mid and side by the
                // SAME factor, so it cannot move the image: a content-driven
                // width change is exactly what image provenance forbids.
                //
                // NORMALISED by k, like every other use of v1. v1 peaks at
                // Q, so against a fixed threshold the raw value would make
                // the detector Q-SENSITIVE: at matched Q on 512 nodes it
                // sees ~80x more signal and Steady would slam to full duck
                // on almost anything. k*v1 has unity passband gain at any
                // Q. Exact at Q = 1, where k is 1.
                const double mag = std::abs(v1m) * k_;
                double& env = steady_env_[b];
                env += (mag > env ? atk_alpha_ : rel_alpha_) * (mag - env);
                if (env > steady_threshold_) {
                    const double over = env / steady_threshold_ - 1.0;
                    const double duck = 1.0 / (1.0 + steady_amount_ * over);
                    bigm *= duck;
                    bigs *= duck;
                }
            }
            g_eff_[b] = 1.0 + bigm;
            gs_eff_[b] = 1.0 + bigs;

            // v1 is the Q-SCALED bandpass: |v1| peaks at Q, not at 1. At
            // Q = 1 that is invisible, which is why the bank was correct by
            // accident. Normalising by k = 1/Q makes the peak gain exactly
            // G at ANY Q -- without it Q = 80 would be 80x too deep.
            // ok_ = k_ / K folds BOTH scalars into the one multiply that
            // was already here: k = 1/Q normalises the SVF's Q-scaled
            // bandpass, 1/K removes the band overlap. At Q = 1 with the
            // correction off, ok_ is exactly 1.0 and this is bit-identical
            // to the pre-Q bank.
            ym += v1m * (bigm * ok_);
            if (stereo)
                ys += v1s * (bigs * ok_) + v1m * (pan * ok_);
        }
        if (stereo) {
            l = static_cast<float>(ym + ys);
            r = static_cast<float>(ym - ys);
        } else {
            l = static_cast<float>(ym);
            r = l;
        }
    }

private:
    void recompute_coefficients();
    void refresh_overlap();
    void rebuild_response_kernel();

    int num_bands_ = 0;
    double fs_ = 48000.0;
    double q_ = 1.0, k_ = 1.0;          // k = 1/Q, the SVF damping
    double overlap_k_ = 1.0;            // K: how much the bands add up
    double ok_ = 1.0;                   // k_ / overlap_k_, the audio scalar
    double overlap_amt_ = 0.0;
    double steady_amount_ = 0.0, steady_threshold_ = 0.25;
    double atk_alpha_ = 0.0, rel_alpha_ = 0.0;
    std::vector<double> fc_;            // band center frequencies
    std::vector<double> gt_;            // tan(pi*fc/fs), independent of Q
    // Response kernel, indexed by lag + resp_w_. 1/K is folded in.
    int resp_w_ = 0;
    std::vector<double> kern_re_, kern_im_;
    std::vector<double> a1_, a2_, a3_;  // SVF coefficients (g, k fixed per band)
    std::vector<double> g_;             // current linear gain per band
    std::vector<double> ramp_;          // per-frame geometric step toward target
    std::vector<double> g_eff_;         // gain after Steady — the display truth
    std::vector<double> steady_env_;    // ONE follower per band (mid detector)
    // Spatial subscriber state. gs_ ramps geometrically like g_; pan_ ramps
    // LINEARLY because it is signed and passes through zero.
    bool spatial_ = false;
    std::vector<double> gs_, ramp_s_, pan_, pan_step_, gs_eff_;
    std::vector<std::vector<double>> s1_, s2_;  // integrator state [channel][band]
};

}  // namespace curves
