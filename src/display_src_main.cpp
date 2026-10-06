// Is the curve recoverable in EVERY subscriber configuration?
// The curve went missing twice because the display enumerated sources and
// missed one. This checks all four, exhaustively.
#include "filter_bank.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
using curves::FilterBank;

int main() {
    const int NB = 32;
    const double inv = 1.0 / 6.020599913279624;
    std::printf("%-34s %12s %12s %12s %10s\n",
                "configuration", "mid carries", "side carries", "pan carries", "drawable");

    struct Cfg { const char* name; double range, width, pan; };
    const Cfg cfgs[] = {
        {"EQ 18 dB, width 0, place 0",      18.0, 0.0,  0.0},
        {"EQ 18 dB, width 12, place 0.5",   18.0, 12.0, 0.5},
        {"EQ OFF,   width 12, place 0",      0.0, 12.0, 0.0},
        {"EQ OFF,   width 0,  place 0.5",    0.0, 0.0,  0.5},
        {"EQ OFF,   width 0,  place 0",      0.0, 0.0,  0.0},
    };
    for (const auto& c : cfgs) {
        FilterBank bank;
        bank.prepare(NB, 48000.0, 2);
        for (int b = 0; b < NB; ++b)
            bank.set_band_frequency(b, 30.0 * std::pow(600.0, double(b) / (NB - 1)));
        bank.set_steady(0.0, 0.25);
        std::vector<double> gm(NB), gs(NB), pn(NB), shape(NB);
        for (int b = 0; b < NB; ++b) {
            shape[b] = std::sin(2.0 * 3.14159265 * b / NB);
            gm[b] = std::exp2(shape[b] * c.range * inv);
            gs[b] = std::exp2(shape[b] * (c.range + c.width) * inv);
            pn[b] = shape[b] * c.pan;
        }
        const bool spatial = (c.width > 1e-6 || c.pan > 1e-6);
        for (int k2 = 0; k2 < 400; ++k2) {
            bank.set_block_targets(gm.data(), 32);
            if (spatial) bank.set_spatial_targets(gs.data(), pn.data(), 32);
            else bank.set_spatial_off();
            for (int k = 0; k < 32; ++k) {
                float l = 0.2f, r = -0.1f;
                bank.process_frame(l, r, true);
            }
        }
        double sm = 0, ss = 0, sp = 0;
        for (int b = 0; b < NB; ++b) {
            sm = std::max(sm, std::fabs(bank.gains()[b] - 1.0));
            ss = std::max(ss, std::fabs(bank.side_gains()[b] - 1.0));
            sp = std::max(sp, std::fabs(bank.pans()[b]));
        }
        const bool drawable = (sm > 1e-9 || ss > 1e-9 || sp > 1e-9);
        std::printf("%-34s %12.4f %12.4f %12.4f %10s\n", c.name, sm, ss, sp,
                    drawable ? "yes" : "FLAT");
    }
    std::printf("\nA FLAT row is only correct when nothing is applied --\n");
    std::printf("the last one. Any other FLAT row is the bug recurring.\n");
    return 0;
}
