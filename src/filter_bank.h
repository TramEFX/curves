// Curves — string-driven filter bank.
//
// The subscriber side of the architecture: N TPT/SVF peaking filters, one
// per string node, whose gains are pushed BY THE STRING. Band center
// frequencies are fixed (log-spaced); the string modulates gain per sample
// through one one-pole smoother per band — the entire smoothing system.
//
// Cutoff modulation (the "chorus without detune" motion) is deliberately
// deferred to M2; gain-only modulation keeps M1 airtight.
//
// Zavalishin TPT SVF, peaking configuration: out = x + bandpass * (G - 1).
// JUCE-free (portable, testable); the plugin adapter lives in src/plugin.

#pragma once

#include <cmath>
#include <vector>

namespace curves {

class FilterBank {
public:
    void prepare(int num_bands, double sample_rate, int num_channels);

    // Fixed band center frequencies; caller owns the frequency map.
    void set_band_frequency(int band, double fc);

    // Smoothing time constant of the one-pole per band, milliseconds.
    void set_smoothing(double tau_ms);

    // Per-band dynamic EQ ("Steady"): bands whose bandpass energy exceeds
    // the threshold get their own contribution squeezed. amount 0 = off.
    // Threshold is linear amplitude on the bandpass signal.
    void set_steady(double amount, double threshold);

    // Per-block target gains, linear. Length must equal num_bands.
    // The per-sample one-pole pulls the smoothed gain toward these.
    void set_targets(const double* linear_gains);

    // Truth for the display: the currently smoothed per-band linear gains
    // (what the EQ is actually applying right now, including Steady).
    const std::vector<double>& gains() const { return g_; }

    inline float process_sample(int channel, float x) {
        const double a = alpha_;
        float y = x;
        const bool steady_on = steady_amount_ > 0.0;
        const double atk = atk_alpha_, rel = rel_alpha_;
        for (int b = 0; b < num_bands_; ++b) {
            // one-pole toward target
            g_[b] += a * (target_[b] - g_[b]);
            double big = g_[b] - 1.0;  // peaking boost/cut amount
            const double s1 = s1_[channel][b], s2 = s2_[channel][b];
            const double v3 = x - s2;
            const double v1 = a1_[b] * s1 + a2_[b] * v3;
            const double v2 = s2 + a2_[b] * s1 + a3_[b] * v3;
            s1_[channel][b] = 2.0 * v1 - s1;
            s2_[channel][b] = 2.0 * v2 - s2;
            if (steady_on) {
                // per-band follower on the bandpass magnitude; squeeze only
                // this band's contribution when it runs over threshold
                const double mag = std::abs(v1);
                double& env = steady_env_[channel][b];
                env += (mag > env ? atk : rel) * (mag - env);
                if (env > steady_threshold_) {
                    const double over = env / steady_threshold_ - 1.0;
                    big *= 1.0 / (1.0 + steady_amount_ * over);
                }
            }
            y += static_cast<float>(v1 * big);
        }
        return y;
    }

private:
    void recompute_coefficients();

    int num_bands_ = 0;
    double fs_ = 48000.0;
    double alpha_ = 0.05;
    double steady_amount_ = 0.0, steady_threshold_ = 0.25;
    double atk_alpha_ = 0.0, rel_alpha_ = 0.0;
    std::vector<double> fc_;        // band center frequencies
    std::vector<double> a1_, a2_, a3_;  // SVF coefficients (g, k fixed per band)
    std::vector<double> g_;         // smoothed linear gain per band
    std::vector<double> target_;    // target linear gain per band
    std::vector<std::vector<double>> s1_, s2_;   // integrator state [channel][band]
    std::vector<std::vector<double>> steady_env_;  // follower state [channel][band]
};

}  // namespace curves
