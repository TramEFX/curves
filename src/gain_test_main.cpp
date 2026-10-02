// Verifies the gain path against the two properties it was rebuilt for:
//   1. the DRAWN CURVE is independent of Range dB, moving or still;
//   2. boosts and cuts are symmetric in dB.
// Exercises the shipped FilterBank, not a replica of it.

#include "filter_bank.h"

#include <cmath>
#include <cstdio>
#include <vector>

using curves::FilterBank;

namespace {
constexpr double kFs = 48000.0;
constexpr int kBands = 64;
constexpr int kBlock = 128;

struct Run {
    std::vector<double> shown;  // normalized curve the display would draw
    double peak = 0.0, rms = 0.0;
};

// Mirrors PluginProcessor: smooth disp at the tick rate, map once with Range,
// hand the bank a block target, read the curve back the way publish_display
// does (bank gains -> dB -> / Range).
Run run(double range_db, double smooth_ms, double hz, double amp,
        double dur_s, int divisor = 2) {
    FilterBank bank;
    bank.prepare(kBands, kFs, 2);
    for (int b = 0; b < kBands; ++b)
        bank.set_band_frequency(b, 30.0 * std::pow(600.0, double(b) / (kBands - 1)));
    bank.set_steady(0.0, 0.25);

    const double tick_rate = kFs / divisor;
    const double alpha = 1.0 - std::exp(-1.0 / ((smooth_ms * 1e-3) * tick_rate));
    std::vector<double> sdisp(kBands, 0.0), targets(kBands, 1.0);

    Run out;
    const int blocks = int(dur_s * kFs / kBlock);
    int counter = 0;
    double t = 0.0;
    for (int blk = 0; blk < blocks; ++blk) {
        for (int b = 0; b < kBands; ++b) {
            const double db = std::fmin(std::fmax(sdisp[b], -1.0), 1.0) * range_db;
            targets[b] = std::exp2(db * (1.0 / 6.020599913279624));
        }
        bank.set_block_targets(targets.data(), kBlock);
        for (int s = 0; s < kBlock; ++s) {
            if (++counter >= divisor) {
                counter = 0;
                const double disp = amp * std::sin(2.0 * 3.14159265358979323846 * hz * t);
                for (int b = 0; b < kBands; ++b)
                    sdisp[b] += alpha * (disp - sdisp[b]);
                t += divisor / kFs;
            }
            float l = 0.1f, r = 0.1f;
            bank.process_frame(l, r, true);
        }
        // what publish_display() computes
        const double g = bank.gains()[0];
        const double shown = 20.0 * std::log10(std::fmax(g, 1e-6)) / range_db;
        out.shown.push_back(shown);
    }
    for (double v : out.shown) {
        out.peak = std::fmax(out.peak, std::fabs(v));
        out.rms += v * v;
    }
    out.rms = std::sqrt(out.rms / out.shown.size());
    return out;
}
}  // namespace

int main() {
    const double ranges[] = {3.0, 6.0, 12.0, 18.0, 24.0, 48.0};

    std::printf("1. DRAWN CURVE vs Range dB  (string wobbling, Smoothing 30 ms)\n");
    std::printf("   deviation is measured against Range 18, in units of full scale\n\n");
    for (double hz : {0.5, 3.0, 8.0}) {
        Run base = run(18.0, 30.0, hz, 0.8, 3.0);
        std::printf("   %.1f Hz:  %8s %10s %10s %14s\n", hz,
                    "Range", "peak", "rms", "max deviation");
        double worst = 0.0;
        for (double r : ranges) {
            Run x = run(r, 30.0, hz, 0.8, 3.0);
            double dev = 0.0;
            for (size_t i = 0; i < x.shown.size(); ++i)
                dev = std::fmax(dev, std::fabs(x.shown[i] - base.shown[i]));
            worst = std::fmax(worst, dev);
            std::printf("            %8.0f %10.4f %10.4f %14.2e\n",
                        r, x.peak, x.rms, dev);
        }
        std::printf("            worst deviation across a 16x Range change: %.2e\n\n",
                    worst);
    }

    std::printf("2. BOOST / CUT SYMMETRY  (step to +-Range, time to 90%% in dB)\n\n");
    std::printf("   %8s %12s %12s %12s\n", "Range", "boost", "cut", "asymmetry");
    for (double r : ranges) {
        double tms[2];
        for (int k = 0; k < 2; ++k) {
            const double sign = k == 0 ? 1.0 : -1.0;
            FilterBank bank;
            bank.prepare(kBands, kFs, 2);
            for (int b = 0; b < kBands; ++b)
                bank.set_band_frequency(b, 1000.0);
            bank.set_steady(0.0, 0.25);
            const double alpha = 1.0 - std::exp(-1.0 / (0.030 * (kFs / 2)));
            std::vector<double> sd(kBands, 0.0), tg(kBands, 1.0);
            int counter = 0, frames = 0;
            tms[k] = -1.0;
            for (int blk = 0; blk < 400; ++blk) {
                for (int b = 0; b < kBands; ++b)
                    tg[b] = std::exp2(sd[b] * r * (1.0 / 6.020599913279624));
                bank.set_block_targets(tg.data(), kBlock);
                for (int s = 0; s < kBlock; ++s) {
                    if (++counter >= 2) {
                        counter = 0;
                        for (int b = 0; b < kBands; ++b)
                            sd[b] += alpha * (sign - sd[b]);
                    }
                    float l = 0.1f, rr = 0.1f;
                    bank.process_frame(l, rr, true);
                    ++frames;
                }
                const double db = 20.0 * std::log10(std::fmax(bank.gains()[0], 1e-9));
                if (tms[k] < 0.0 && (sign > 0 ? db >= 0.9 * r : db <= -0.9 * r))
                    tms[k] = frames / kFs * 1000.0;
            }
        }
        std::printf("   %8.0f %10.2fms %10.2fms %11.2fx\n", r, tms[0], tms[1],
                    std::fmax(tms[0], tms[1]) / std::fmin(tms[0], tms[1]));
    }
    std::printf("\n3. STEADY ON THE CURVE  (one band on-centre, Range 18 -> +18 dB)\n");
    std::printf("   the curve must now SHOW the duck, not just apply it\n\n");
    std::printf("   %8s %14s %18s %10s\n", "Steady", "curve shows",
                "actually applied", "hidden");
    for (double amt : {0.0, 0.2, 0.35, 0.6, 1.0}) {
        FilterBank bank;
        bank.prepare(1, kFs, 2);
        bank.set_band_frequency(0, 1000.0);
        bank.set_steady(amt, 0.25);
        std::vector<double> tg(1, std::exp2(18.0 * (1.0 / 6.020599913279624)));
        const double w = 2.0 * 3.14159265358979323846 * 1000.0 / kFs;
        double in_pk = 0.0, out_pk = 0.0;
        const int total = int(2.0 * kFs);
        for (int blk = 0; blk * kBlock < total; ++blk) {
            bank.set_block_targets(tg.data(), kBlock);
            for (int s = 0; s < kBlock; ++s) {
                const int i = blk * kBlock + s;
                float l = float(0.5 * std::sin(w * i));
                float r = l;
                const double xin = l;
                bank.process_frame(l, r, true);
                if (i > total * 3 / 4) {
                    in_pk = std::fmax(in_pk, std::fabs(xin));
                    out_pk = std::fmax(out_pk, std::fabs(double(l)));
                }
            }
        }
        const double shown = 20.0 * std::log10(bank.gains()[0]);
        const double applied = 20.0 * std::log10(out_pk / in_pk);
        std::printf("   %8.2f %12.2fdB %16.2fdB %8.2fdB\n",
                    amt, shown, applied, shown - applied);
    }
    return 0;
}
