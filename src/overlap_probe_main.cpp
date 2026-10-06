// Band overlap in the parallel peaking bank.
//
// out = x + sum_b bp_b * (g_b - 1). With Q = 1 the bands are ~1.4 octaves
// wide while the spacing is 9.23/N octaves, so roughly 0.15*N of them
// overlap at any frequency and their contributions ADD:
//
//     actual = 1 + K*(g - 1),   K ~= 0.15 * N
//
// Consequences: the depth of every per-band subscriber is NOMINAL rather
// than actual, it scales with node count, and cuts can drive the effective
// gain NEGATIVE (polarity inversion) rather than attenuating.

#include "filter_bank.h"
#include <cmath>
#include <cstdio>
#include <vector>
using curves::FilterBank;

double flat_gain_db(int nb, double nominal_db) {
    FilterBank bank;
    bank.prepare(nb, 48000.0, 2);
    for (int b = 0; b < nb; ++b)
        bank.set_band_frequency(b, 30.0 * std::pow(600.0, double(b) / (nb - 1)));
    bank.set_steady(0.0, 0.25);
    std::vector<double> g(nb, std::exp2(nominal_db / 6.020599913279624));
    double in = 0.0, out = 0.0;
    unsigned int rs = 12345;
    for (int c = 0; c < 1500; ++c) {
        bank.set_block_targets(g.data(), 32);
        bank.set_spatial_off();
        for (int k = 0; k < 32; ++k) {
            rs = rs * 1664525u + 1013904223u;
            const float x = float((int(rs >> 9) & 0x3FFFFF) / 2097151.0 - 1.0) * 0.2f;
            float l = x, r = x;
            bank.process_frame(l, r, true);
            if (c < 400) continue;
            in += double(x) * x;
            out += double(l) * l;
        }
    }
    return 20.0 * std::log10(std::sqrt(out / in));
}

int main() {
    std::printf("BOOSTS: a nominal +6 dB on every band.\n\n");
    std::printf("%8s %11s %11s %10s %10s\n",
                "bands", "actual dB", "actual x", "fitted K", "0.15*N");
    const double G = std::exp2(6.0 / 6.020599913279624);
    for (int nb : {8, 48, 128, 256, 512}) {
        const double a = flat_gain_db(nb, 6.0);
        const double act = std::pow(10.0, a / 20.0);
        std::printf("%8d %10.2fdB %10.3fx %10.2f %10.1f\n",
                    nb, a, act, (act - 1.0) / (G - 1.0), 0.152 * nb);
    }
    std::printf("\nCUTS: 1 + K*(g-1) goes NEGATIVE once K*(1-g) > 1, so the\n");
    std::printf("band inverts polarity and a cut comes back as a BOOST.\n\n");
    std::printf("%8s %10s %11s\n", "bands", "nominal", "actual dB");
    for (int nb : {48, 512})
        for (double d : {-3.0, -6.0, -12.0})
            std::printf("%8d %9.1fdB %10.2fdB\n", nb, d, flat_gain_db(nb, d));
    return 0;
}
