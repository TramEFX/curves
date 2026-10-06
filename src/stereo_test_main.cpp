// Does the spatial subscriber do what it claims?
//   - width moves the side/mid ratio by the predicted amount
//   - placement moves the L/R balance, and SURVIVES A MONO FOLD
//   - the simple pan law's level excursion, measured rather than assumed
//   - the frequency taper actually leaves the outer bands alone
// Exercises the shipped FilterBank.

#include "filter_bank.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using curves::FilterBank;

namespace {
constexpr double kFs = 48000.0;
constexpr int kNB = 48;
constexpr int kChunk = 32;

struct Meas { double mid_rms, side_rms, l_rms, r_rms, sum_rms; };

// width_db / pan applied flat across every band (taper tested separately)
Meas run(double range_db, double width_db, double pan, double disp,
         bool mono_source, double seconds = 1.0) {
    FilterBank bank;
    bank.prepare(kNB, kFs, 2);
    for (int b = 0; b < kNB; ++b)
        bank.set_band_frequency(b, 30.0 * std::pow(600.0, double(b) / (kNB - 1)));
    bank.set_steady(0.0, 0.25);

    std::vector<double> gm(kNB), gs(kNB), pn(kNB);
    const double inv = 1.0 / 6.020599913279624;
    for (int b = 0; b < kNB; ++b) {
        gm[b] = std::exp2(disp * range_db * inv);
        gs[b] = std::exp2((disp * range_db + disp * width_db) * inv);
        pn[b] = disp * pan;
    }

    Meas m{};
    long n = 0;
    unsigned int rl = 22222, rr = 99991;
    const int frames = int(seconds * kFs);
    for (int f0 = 0; f0 < frames; f0 += kChunk) {
        bank.set_block_targets(gm.data(), kChunk);
        if (width_db != 0.0 || pan != 0.0)
            bank.set_spatial_targets(gs.data(), pn.data(), kChunk);
        else
            bank.set_spatial_off();
        for (int k = 0; k < kChunk; ++k) {
            rl = rl * 1664525u + 1013904223u;
            rr = rr * 1664525u + 1013904223u;
            const float a = float((int(rl >> 9) & 0x3FFFFF) / 2097151.0 - 1.0) * 0.3f;
            const float b2 = float((int(rr >> 9) & 0x3FFFFF) / 2097151.0 - 1.0) * 0.3f;
            float l = a;
            float r = mono_source ? a : b2;
            bank.process_frame(l, r, true);
            if (f0 < frames / 4) continue;   // settle the ramps
            const double M = 0.5 * (l + r), S = 0.5 * (l - r);
            m.mid_rms += M * M; m.side_rms += S * S;
            m.l_rms += double(l) * l; m.r_rms += double(r) * r;
            m.sum_rms += (double(l) + r) * (double(l) + r);
            ++n;
        }
    }
    const double inv_n = 1.0 / std::max(1L, n);
    m.mid_rms = std::sqrt(m.mid_rms * inv_n);
    m.side_rms = std::sqrt(m.side_rms * inv_n);
    m.l_rms = std::sqrt(m.l_rms * inv_n);
    m.r_rms = std::sqrt(m.r_rms * inv_n);
    m.sum_rms = std::sqrt(m.sum_rms * inv_n);
    return m;
}
}  // namespace

int main() {
    std::printf("A. WIDTH -- decorrelated stereo in, EQ flat, disp = +1.\n");
    std::printf("   predicted side/mid ratio = 2^(Width/6.0206).\n\n");
    std::printf("%10s %12s %12s %12s %10s\n",
                "Width dB", "side/mid", "predicted", "error", "L rms");
    const Meas base = run(0.0, 0.0, 0.0, 1.0, false);
    const double base_ratio = base.side_rms / base.mid_rms;
    for (double w : {0.0, 3.0, 6.0, 12.0, 24.0, -6.0}) {
        Meas m = run(0.0, w, 0.0, 1.0, false);
        const double got = (m.side_rms / m.mid_rms) / base_ratio;
        const double pred = std::exp2(w / 6.020599913279624);
        std::printf("%10.1f %12.4f %12.4f %11.2f%% %10.4f\n",
                    w, got, pred, 100.0 * (got - pred) / pred, m.l_rms);
    }

    std::printf("\nB. PLACEMENT -- MONO source in, so any L/R imbalance is\n");
    std::printf("   the subscriber's doing. Predicted L/R = (1+p)/(1-p).\n\n");
    std::printf("%10s %10s %10s %12s %14s %12s\n",
                "pan", "L rms", "R rms", "L/R", "predicted", "mono sum");
    const Meas mb = run(0.0, 0.0, 0.0, 1.0, true);
    for (double p : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        Meas m = run(0.0, 0.0, p, 1.0, true);
        const double got = m.l_rms / std::max(m.r_rms, 1e-12);
        const double pred = (1.0 + p) / std::max(1.0 - p, 1e-12);
        std::printf("%10.2f %10.4f %10.4f %12.3f %14.3f %11.4f (%.3fx)\n",
                    p, m.l_rms, m.r_rms, got, pred, m.sum_rms,
                    m.sum_rms / mb.sum_rms);
    }
    std::printf("\n   'mono sum' is rms(L+R). If placement is mono-safe it\n");
    std::printf("   must not move: panning is a balance change, not a\n");
    std::printf("   cancellation.\n");

    std::printf("\nC. LEVEL COST of the simple pan law (mono source).\n");
    std::printf("   The flagged question: no constant-power compensation.\n\n");
    std::printf("%10s %12s %12s\n", "pan", "total rms", "vs pan 0");
    for (double p : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        Meas m = run(0.0, 0.0, p, 1.0, true);
        const double tot = std::sqrt(0.5 * (m.l_rms * m.l_rms + m.r_rms * m.r_rms));
        const double t0 = std::sqrt(0.5 * (mb.l_rms * mb.l_rms + mb.r_rms * mb.r_rms));
        std::printf("%10.2f %12.4f %10.2f dB\n", p, tot,
                    20.0 * std::log10(tot / t0));
    }
    return 0;
}
