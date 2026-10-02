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

    // Target linear gains for the coming block, and how many frames to take
    // getting there. The walk is GEOMETRIC (g *= r), which is exactly linear
    // in dB for one multiply per band per frame. A linear ramp in gain was
    // measured at up to 24 dB of mid-block error at Smoothing 2 ms.
    void set_block_targets(const double* linear_gains, int num_frames);

    // Truth for the display: effective per-band linear gain, INCLUDING the
    // Steady duck. This is what the EQ is applying right now.
    const std::vector<double>& gains() const { return g_eff_; }

    // One sample FRAME. The gain advances exactly once here, not once per
    // channel — the previous per-channel entry point double-stepped the
    // smoother, so stereo ran at half the set smoothing time and a mono
    // instance did not match a stereo one.
    inline void process_frame(float& l, float& r, bool stereo) {
        const bool steady_on = steady_amount_ > 0.0;
        const double xl = l, xr = r;
        double yl = xl, yr = xr;
        for (int b = 0; b < num_bands_; ++b) {
            const double g = g_[b] * ramp_[b];
            g_[b] = g;
            double big = g - 1.0;  // peaking boost/cut amount

            double s1 = s1_[0][b], s2 = s2_[0][b];
            double v3 = xl - s2;
            const double v1l = a1_[b] * s1 + a2_[b] * v3;
            double v2 = s2 + a2_[b] * s1 + a3_[b] * v3;
            s1_[0][b] = 2.0 * v1l - s1;
            s2_[0][b] = 2.0 * v2 - s2;

            double v1r = 0.0;
            if (stereo) {
                s1 = s1_[1][b];
                s2 = s2_[1][b];
                v3 = xr - s2;
                v1r = a1_[b] * s1 + a2_[b] * v3;
                v2 = s2 + a2_[b] * s1 + a3_[b] * v3;
                s1_[1][b] = 2.0 * v1r - s1;
                s2_[1][b] = 2.0 * v2 - s2;
            }

            if (steady_on) {
                // MONO detector: max of the two bandpasses, one follower,
                // one duck applied identically to both channels.
                const double mag = stereo ? std::max(std::abs(v1l), std::abs(v1r))
                                          : std::abs(v1l);
                double& env = steady_env_[b];
                env += (mag > env ? atk_alpha_ : rel_alpha_) * (mag - env);
                if (env > steady_threshold_) {
                    const double over = env / steady_threshold_ - 1.0;
                    big *= 1.0 / (1.0 + steady_amount_ * over);
                }
            }
            g_eff_[b] = 1.0 + big;

            yl += v1l * big;
            if (stereo)
                yr += v1r * big;
        }
        l = static_cast<float>(yl);
        r = static_cast<float>(stereo ? yr : yl);
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
    std::vector<double> steady_env_;    // ONE follower per band (mono detector)
    std::vector<std::vector<double>> s1_, s2_;  // integrator state [channel][band]
};

}  // namespace curves
