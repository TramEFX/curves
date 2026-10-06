// With the EQ gated off, is the curve still there to draw?
// The mid gain goes flat; the shape lives in the side gain. publish_display
// must follow it, or the canvas flatlines while the image is still moving.
#include "filter_bank.h"
#include <cmath>
#include <cstdio>
#include <vector>
using curves::FilterBank;

int main() {
    const int NB = 32;
    FilterBank bank;
    bank.prepare(NB, 48000.0, 2);
    for (int b = 0; b < NB; ++b)
        bank.set_band_frequency(b, 30.0 * std::pow(600.0, double(b) / (NB - 1)));
    bank.set_steady(0.0, 0.25);

    // EQ OFF -> range_db = 0, so every mid target is exactly 1.0.
    // Width 12 dB, with a shape across the bands.
    const double inv = 1.0 / 6.020599913279624;
    std::vector<double> gm(NB), gs(NB), pn(NB, 0.0), shape(NB);
    for (int b = 0; b < NB; ++b) {
        shape[b] = std::sin(2.0 * 3.14159265 * b / NB);
        gm[b] = 1.0;                                   // EQ off
        gs[b] = std::exp2(shape[b] * 12.0 * inv);      // width carries it
    }
    for (int c = 0; c < 400; ++c) {
        bank.set_block_targets(gm.data(), 32);
        bank.set_spatial_targets(gs.data(), pn.data(), 32);
        for (int k = 0; k < 32; ++k) { float l = 0.2f, r = -0.1f; bank.process_frame(l, r, true); }
    }
    std::printf("EQ gated off, Width 12 dB, a sine shape across %d bands.\n\n", NB);
    std::printf("%6s %10s %14s %14s %12s\n",
                "band", "shape", "mid (drawn?)", "side (drawn?)", "side norm");
    double mid_span = 0.0, side_err = 0.0;
    for (int b = 0; b < NB; b += 4) {
        const double m = bank.gains()[b], sg = bank.side_gains()[b];
        const double norm = 20.0 * std::log10(sg) / 12.0;
        std::printf("%6d %10.3f %14.5f %14.5f %12.4f\n", b, shape[b], m, sg, norm);
    }
    for (int b = 0; b < NB; ++b) {
        mid_span = std::max(mid_span, std::fabs(bank.gains()[b] - 1.0));
        side_err = std::max(side_err,
            std::fabs(20.0 * std::log10(bank.side_gains()[b]) / 12.0 - shape[b]));
    }
    std::printf("\nmid gain deviation from flat: %.2e  (nothing to draw)\n", mid_span);
    std::printf("side gain, normalised by Width dB, vs the shape: max err %.2e\n", side_err);
    std::printf("-> the curve IS recoverable, from the side gain.\n");
    return 0;
}
