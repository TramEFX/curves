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
                const double mag = std::abs(v1m);
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

            ym += v1m * bigm;
            if (stereo)
                ys += v1s * bigs + v1m * pan;
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

    int num_bands_ = 0;
    double fs_ = 48000.0;
    double steady_amount_ = 0.0, steady_threshold_ = 0.25;
    double atk_alpha_ = 0.0, rel_alpha_ = 0.0;
    std::vector<double> fc_;            // band center frequencies
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
