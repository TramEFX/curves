// Curves — output leveler.
//
// Two layers, both one-pole math, no exception paths:
//   1. Slow RMS auto-trim: pulls output loudness toward -18 dBFS RMS with a
//      ~1 s time constant, clamped to ±12 dB of correction. amount 0 = off.
//   2. Fast safety limiter: catches peaks above -1 dBFS with 1 ms attack.
//      ALWAYS ACTIVE, independent of `amount` — it is a speaker guard, not
//      part of the levelling effect. amount = 0 therefore means "no RMS
//      trim", NOT "bypass": the signal path is still peak-limited if it
//      runs hot. (Earlier notes claiming amount 0 gives a strictly
//      string-only path were wrong on this.)
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

    // `amount` is a DEPTH control: the correction is g^amount, so 0 = none
    // and 1 = full. g tracks the live RMS, so the exponentiation cannot be
    // hoisted out of the sample loop; it is done with exp/log instead of
    // pow (see fast_pow), and short-circuits entirely at amount = 1.
    void set_amount(double amount) { amount_ = std::clamp(amount, 0.0, 1.0); }

    // In-place stereo/mono process: control signal = max(|L|, |R|) so both
    // channels receive the identical trim/limiter gain (no image shifts).
    // For mono, pass the same variable twice.
    inline void process(float& l, float& r) {
        const double sq = std::max(static_cast<double>(l) * l,
                                   static_cast<double>(r) * r);
        rms_ += rms_alpha_ * (sq - rms_);

        // Slow trim toward the target RMS, ±12 dB of correction.
        //
        // The target is the MEAN SQUARE of the goal level, so it must be
        // 10^(dBFS/10): -18 dBFS -> 0.0158489. The old constant was 0.0625,
        // which is (0.25)^2 = -12 dBFS squared, so the stage levelled 6 dB
        // hotter than the comment and the design notes both claimed — fed
        // exactly -18 dBFS it applied +5.9 dB instead of nothing.
        //
        // Range widened from ±6 dB (g in [0.5, 2]) to ±12 dB (g in
        // [0.251, 3.98]): with Range at ±18 dB the EQ alone can swing the
        // output further than 6 dB, so the old clamp pinned and the depth
        // knob had little left to scale.
        double trim = 1.0;
        if (amount_ > 0.0 && rms_ > 1e-9) {
            const double target = 0.0158489;  // -18 dBFS, as mean square
            double g = std::sqrt(target / rms_);
            g = std::clamp(g, 0.2511886, 3.9810717);  // -12 dB .. +12 dB
            trim *= fast_pow(g, amount_);
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
    // g^amount without a per-sample std::pow: exp(amount * log(g)). Both
    // calls are cheaper than pow and this runs once per sample per stage.
    static inline double fast_pow(double g, double amount) {
        return (amount >= 1.0) ? g : std::exp(amount * std::log(g));
    }

    double fs_ = 48000.0;
    double amount_ = 0.0;
    double rms_ = 0.0, trim_ = 1.0, lim_ = 0.0;
    double rms_alpha_ = 0.0, trim_alpha_ = 0.0, lim_atk_ = 0.0, lim_rel_ = 0.0;
};

}  // namespace curves
