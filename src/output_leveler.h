// Curves — output leveler.
//
// Two layers, both one-pole math, no exception paths:
//   1. Slow RMS auto-trim: pulls output loudness toward a target with a
//      ~1 s time constant, clamped to ±6 dB of correction. amount 0 = off.
//   2. Fast safety limiter: catches peaks above -1 dBFS with 1 ms attack.
//
// JUCE-free; the plugin adapter lives in src/plugin.

#pragma once

#include <algorithm>
#include <cmath>

namespace curves {

class OutputLeveler {
public:
    void prepare(double sample_rate) {
        fs_ = sample_rate;
        // RMS follower ~0.5 s, trim smoothing ~1 s, limiter attack ~1 ms.
        rms_alpha_ = 1.0 - std::exp(-1.0 / (0.5 * fs_));
        trim_alpha_ = 1.0 - std::exp(-1.0 / (1.0 * fs_));
        lim_atk_ = 1.0 - std::exp(-1.0 / (0.001 * fs_));
        lim_rel_ = 1.0 - std::exp(-1.0 / (0.100 * fs_));
    }

    void set_amount(double amount) { amount_ = std::clamp(amount, 0.0, 1.0); }

    // In-place stereo/mono process: control signal = max(|L|, |R|) so both
    // channels receive the identical trim/limiter gain (no image shifts).
    // For mono, pass the same variable twice.
    inline void process(float& l, float& r) {
        const double sq = std::max(static_cast<double>(l) * l,
                                   static_cast<double>(r) * r);
        rms_ += rms_alpha_ * (sq - rms_);

        // slow trim toward target RMS (-18 dBFS mean square), ±6 dB clamp
        double trim = 1.0;
        if (amount_ > 0.0 && rms_ > 1e-9) {
            const double target = 0.0625;  // (-18 dBFS)^2
            double g = std::sqrt(target / rms_);
            g = std::clamp(g, 0.5, 2.0);
            trim *= std::pow(g, amount_);
        }
        trim_ += trim_alpha_ * (trim - trim_);

        // fast peak limiter just under full scale
        const double mag = std::max(std::abs(static_cast<double>(l)),
                                    std::abs(static_cast<double>(r))) * trim_;
        lim_ += (mag > lim_ ? lim_atk_ : lim_rel_) * (mag - lim_);
        double lim = 1.0;
        if (lim_ > 0.891)  // -1 dBFS
            lim = 0.891 / lim_;

        l = static_cast<float>(l * trim_ * lim);
        r = static_cast<float>(r * trim_ * lim);
    }

private:
    double fs_ = 48000.0;
    double amount_ = 0.0;
    double rms_ = 0.0, trim_ = 1.0, lim_ = 0.0;
    double rms_alpha_ = 0.0, trim_alpha_ = 0.0, lim_atk_ = 0.0, lim_rel_ = 0.0;
};

}  // namespace curves
