// Bright line vs faint line: what exactly separates them?
//
// faint = sim_->positions()           -- raw string displacement u
// bright = clamp(20log10(gains)/Range) -- what the audio actually goes through
//
// Four things can make them differ. This isolates each one.

#include "filter_bank.h"
#include "string_sim.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace curves;

namespace {
constexpr int kN = 512, kDiv = 2, kChunk = 32, kBlock = 128;
constexpr double kFs = 48000.0, kTick = kFs / kDiv;

struct Res { double maxd, rms, faint_pk, bright_pk; };

Res run(double smooth_ms, double velocity, double steady, double range_db,
        double drive_level, double seconds = 10.0) {
    StringParams p;
    p.tick_rate = kTick; p.wave_speed = 300.0;
    p.damping = 1.2; p.stiffness = 2.0;
    StringSim sim(kN, p);
    sim.anchor_high = true;
    sim.force_model = StringSim::kClothAero;
    sim.set_cloth_aero(192.0 * kN / 128.0, double(kN), 1.5);
    sim.aero_rho_cd = 0.6;
    sim.set_density_taper(2.0);
    sim.set_cloth_length(1.0);
    sim.set_walls(0.0, 1.0);
    sim.wind_speed = 25.0 * 0.596 * std::pow(128.0 / kN, 0.25) * 0.913;
    sim.wrinkle_speed = sim.wind_speed;
    sim.wave_density = 0.5 + 4.5 * 0.4;
    sim.wind_phase_speed = 0.6 * 300.0;
    sim.mid_waves = 11.0; sim.mid_ripple = 1.19;
    sim.mid_lock_d = 0.65; sim.mid_env = 1.4;
    sim.mid_travel = 0.829; sim.mid_flap = 0.45;

    FilterBank bank;
    bank.prepare(kN, kFs, 2);
    for (int b = 0; b < kN; ++b)
        bank.set_band_frequency(b, 30.0 * std::pow(600.0, double(b) / (kN - 1)));
    bank.set_steady(steady, 0.25);

    const double alpha = 1.0 - std::exp(-1.0 / ((smooth_ms * 1e-3) * kTick));
    const double vs = 30.0 / 300.0;
    std::vector<double> sd(kN, 0.0), tg(kN, 1.0);
    int counter = 0;
    Res r{};
    long cnt = 0; double sumsq = 0.0;
    unsigned int rng = 12345;
    const int blocks = int(seconds * kFs / kBlock);
    for (int bi = 0; bi < blocks; ++bi) {
        for (int base = 0; base < kBlock; base += kChunk) {
            for (int i = 0; i < kN; ++i) {
                const double db = std::clamp(sd[i], -1.0, 1.0) * range_db;
                tg[i] = std::exp2(db * (1.0 / 6.020599913279624));
            }
            bank.set_block_targets(tg.data(), kChunk);
            for (int s = 0; s < kChunk; ++s) {
                if (++counter >= kDiv) {
                    counter = 0;
                    sim.tick();
                    const auto& u = sim.positions();
                    const auto& v = sim.velocities();
                    for (int i = 0; i < kN; ++i) {
                        const double disp = (1.0 - velocity) * u[i]
                                            + velocity * (vs * v[i]);
                        sd[i] += alpha * (disp - sd[i]);
                    }
                }
                rng = rng * 1664525u + 1013904223u;
                float l = float((int(rng >> 9) & 0x3FFFFF) / 2097151.0 - 1.0)
                          * float(drive_level);
                float rr = l;
                bank.process_frame(l, rr, true);
            }
        }
        if (bi < blocks / 4) continue;
        const auto& u = sim.positions();
        const auto& g = bank.gains();
        for (int i = 0; i < kN; ++i) {
            const double faint = u[i];
            const double bright = std::clamp(
                20.0 * std::log10(std::max(g[i], 1e-6)) / range_db, -1.0, 1.0);
            const double d = std::fabs(std::clamp(faint, -1.0, 1.0) - bright);
            r.maxd = std::max(r.maxd, d);
            sumsq += d * d; ++cnt;
            r.faint_pk = std::max(r.faint_pk, std::fabs(faint));
            r.bright_pk = std::max(r.bright_pk, std::fabs(bright));
        }
    }
    r.rms = std::sqrt(sumsq / cnt);
    return r;
}
}  // namespace

int main() {
    std::printf("What separates the faint line (raw string u) from the\n");
    std::printf("bright line (the gains the audio goes through)?\n");
    std::printf("512 nodes, shipped Flagpole defaults, 10 s.\n\n");
    std::printf("%-44s %10s %10s\n", "configuration", "max diff", "rms diff");

    struct Case { const char* name; double sm, vel, st, range, lvl; };
    const Case cases[] = {
        {"baseline: Smooth 2ms, Vel 0, Steady 0",        2.0,  0.0,  0.00, 18.0, 0.30},
        {"  + Smoothing 15 ms (the default)",           15.0,  0.0,  0.00, 18.0, 0.30},
        {"  + Smoothing 100 ms (max)",                 100.0,  0.0,  0.00, 18.0, 0.30},
        {"  + Steady 0.35 (default), quiet input",       2.0,  0.0,  0.35, 18.0, 0.10},
        {"  + Steady 0.35 (default), loud input",        2.0,  0.0,  0.35, 18.0, 0.90},
        {"  + Velocity 0.5",                             2.0,  0.5,  0.00, 18.0, 0.30},
        {"  + Velocity 1.0",                             2.0,  1.0,  0.00, 18.0, 0.30},
        {"EVERYTHING at defaults (15ms, Steady .35)",   15.0,  0.0,  0.35, 18.0, 0.50},
    };
    for (const auto& c : cases) {
        Res r = run(c.sm, c.vel, c.st, c.range, c.lvl);
        std::printf("%-44s %10.4f %10.4f\n", c.name, r.maxd, r.rms);
    }
    return 0;
}
