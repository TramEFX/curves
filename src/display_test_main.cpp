// Does the drawn EQ curve track the string, and if not, why?
//
// Reported: with Smoothing at minimum the faint (sim) line still visibly
// disagrees with the bright (EQ) line on slow large humps. Runs the real
// string through the real gain path and reports where the two part company.

#include "filter_bank.h"
#include "string_sim.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace curves;

int main() {
    const int n = 512, div = 2;
    const double fs = 48000.0, tick = fs / div, range_db = 18.0;
    int block = 128;

    std::printf("Faint line = sim u. Bright line = what publish_display() draws.\n");
    std::printf("With smoothing at minimum they should coincide.\n\n");
    std::printf("%8s %10s %12s %12s\n", "block", "Smoothing", "max|sim-eq|", "rms diff");

    for (int blk : {32, 64, 128, 256, 512}) {
      block = blk;
      for (double sm : {2.0, 15.0}) {
        for (double steady : {0.0}) {
            StringParams p;
            p.tick_rate = tick; p.wave_speed = 300.0;
            p.damping = 1.2; p.stiffness = 2.0;
            StringSim sim(n, p);
            sim.anchor_high = true;
            sim.force_model = StringSim::kClothAero;
            sim.set_cloth_aero(192.0 * n / 128.0, double(n), 1.5);
            sim.aero_rho_cd = 0.6;
            sim.set_density_taper(2.0);
            sim.set_cloth_length(1.0);
            sim.set_walls(0.0, 1.0);
            sim.wind_speed = 25.0 * 0.596 * std::pow(128.0 / n, 0.25) * 0.913;
            sim.wrinkle_speed = sim.wind_speed;
            sim.wave_density = 0.5 + 4.5 * 0.4;
            sim.wind_phase_speed = 0.6 * 300.0;
            sim.mid_waves = 11.0; sim.mid_ripple = 1.19;
            sim.mid_lock_d = 0.65; sim.mid_env = 1.4;
            sim.mid_travel = 0.829; sim.mid_flap = 0.45;

            FilterBank bank;
            bank.prepare(n, fs, 2);
            for (int b = 0; b < n; ++b)
                bank.set_band_frequency(b, 30.0 * std::pow(600.0, double(b) / (n - 1)));
            bank.set_steady(steady, 0.25);

            const double alpha = 1.0 - std::exp(-1.0 / ((sm * 1e-3) * tick));
            std::vector<double> sd(n, 0.0), tg(n, 1.0);
            int counter = 0;
            double maxd = 0.0, sumsq = 0.0;
            long cnt = 0;
            unsigned int rng = 12345;
            const int blocks = int(10.0 * fs / block);
            constexpr int kGainChunk = 32;   // matches PluginProcessor
            for (int bi = 0; bi < blocks; ++bi) {
              for (int base = 0; base < block; base += kGainChunk) {
                const int len = std::min(kGainChunk, block - base);
                for (int i = 0; i < n; ++i) {
                    const double db = std::clamp(sd[i], -1.0, 1.0) * range_db;
                    tg[i] = std::exp2(db * (1.0 / 6.020599913279624));
                }
                bank.set_block_targets(tg.data(), len);
                for (int s2 = 0; s2 < len; ++s2) {
                    if (++counter >= div) {
                        counter = 0;
                        sim.tick();
                        const auto& u = sim.positions();
                        for (int i = 0; i < n; ++i) sd[i] += alpha * (u[i] - sd[i]);
                    }
                    rng = rng * 1664525u + 1013904223u;
                    float l = float((int(rng >> 9) & 0x3FFFFF) / 2097151.0 - 1.0) * 0.3f;
                    float r = l;
                    bank.process_frame(l, r, true);
                }
              }
                if (bi < blocks / 4) continue;
                const auto& u = sim.positions();
                const auto& g = bank.gains();
                for (int i = 0; i < n; ++i) {
                    const double faint = std::clamp(u[i], -1.0, 1.0);
                    const double bright = std::clamp(
                        20.0 * std::log10(std::max(g[i], 1e-6)) / range_db, -1.0, 1.0);
                    const double d = std::fabs(faint - bright);
                    maxd = std::max(maxd, d);
                    sumsq += d * d; ++cnt;
                }
            }
            std::printf("%8d %8.0fms %12.4f %12.4f%s", blk, sm, maxd,
                        std::sqrt(sumsq / cnt), "\n");
        }
      }
    }

    return 0;
}
