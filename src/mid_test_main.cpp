// Two questions about the mid band:
//   1. How far toward the TIP does its detail actually reach? (reported:
//      "the mid-level waves dampen a bit too much toward the end".)
//   2. How wide is the resonance? i.e. can the ripples be made to travel at
//      a speed other than the string's own without dying? (reported: "wind
//      should make those waves propagate faster down the string".)
// Curvature per fifth of the cloth is the detail proxy, as in the wrinkle
// and mid scenarios.

#include "string_sim.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace curves;

namespace {
constexpr int kN = 512;

struct R { double fifth[5]; double peak; };

R run(double env, double lock_d, double travel, double mid_amp,
      double wind_knob, double seconds = 14.0, double flap = 0.0,
      double wrinkle = 0.0) {
    const double tick = 24000.0;
    StringParams p;
    p.tick_rate = tick; p.wave_speed = 300.0;
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
    sim.wind_phase_speed = 0.12 * std::pow(25.0, wind_knob) * 300.0;
    sim.mid_waves = 6.0 * std::pow(20.0 / 6.0, wind_knob);
    sim.wrinkle = wrinkle;
    sim.mid_ripple = mid_amp;
    sim.mid_env = env;
    sim.mid_lock_d = lock_d;
    sim.mid_travel = travel;
    sim.mid_flap = flap;

    R r{};
    const long long ticks = (long long)(seconds * tick);
    const long long warm = ticks / 4;
    for (long long t = 0; t < ticks; ++t) {
        sim.tick();
        if (t < warm) continue;
        const auto& u = sim.positions();
        for (int i = 0; i < kN; ++i) r.peak = std::max(r.peak, std::fabs(u[i]));
        // node 0 is the TIP (free whip end); node N-1 is the pole
        for (int i = 1; i + 1 < kN; ++i) {
            const double curv = std::fabs(u[i - 1] - 2.0 * u[i] + u[i + 1]);
            const int f = std::min(4, (kN - 1 - i) * 5 / kN);  // 0 = tip side
            r.fifth[f] += curv;
        }
    }
    return r;
}
}  // namespace

double g_lo = 1.0, g_hi = 1.0;

int main() {
    std::printf("512 nodes. Curvature per fifth of the cloth,\n");
    std::printf("fifth 0 = TIP (free whip end) ... fifth 4 = POLE (anchored).\n");

    std::printf("\n1. ENVELOPE / LOCK POSITION -- where does the detail land?\n");
    std::printf("%6s %7s %9s %9s %9s %9s %9s %9s\n",
                "env", "lock_d", "tip", "f1", "f2", "f3", "pole", "peak");
    R off = run(2.0, 0.5, 1.0, 0.0, 0.5);
    std::printf("%6s %7s %9.0f %9.0f %9.0f %9.0f %9.0f %9.4f   <- mid OFF\n",
                "-", "-", off.fifth[0], off.fifth[1], off.fifth[2],
                off.fifth[3], off.fifth[4], off.peak);
    const double envs[] = {2.0, 1.4, 1.0, 0.7};
    const double locks[] = {0.5, 0.65, 0.8};
    for (double e : envs)
        for (double l : locks) {
            R r = run(e, l, 1.0, 1.5, 0.5);
            std::printf("%6.1f %7.2f %9.0f %9.0f %9.0f %9.0f %9.0f %9.4f\n",
                        e, l, r.fifth[0], r.fifth[1], r.fifth[2],
                        r.fifth[3], r.fifth[4], r.peak);
        }

    std::printf("\n2. RESONANCE WIDTH at the chosen shape (env 1.4, lock 0.65).\n");
    std::printf("   response normalized to travel = 1.0 -> the comp table.\n");
    std::printf("%8s %9s %9s %9s %9s %9s %9s %9s\n",
                "travel", "pole", "f1", "f2", "f3", "tip", "total", "R");
    {
        double tot_off = 0.0;
        for (int f = 0; f < 5; ++f) tot_off += off.fifth[f];
        R base = run(1.4, 0.65, 1.0, 1.5, 0.5);
        double tot_base = 0.0;
        for (int f = 0; f < 5; ++f) tot_base += base.fifth[f];
        for (double tv : {0.35, 0.5, 0.65, 0.8, 1.0, 1.2, 1.45, 1.8, 2.4}) {
            R r = run(1.4, 0.65, tv, 1.5, 0.5);
            double tot = 0.0;
            for (int f = 0; f < 5; ++f) tot += r.fifth[f];
            std::printf("%8.2f %9.0f %9.0f %9.0f %9.0f %9.0f %8.2fx %9.3f\n",
                        tv, r.fifth[0], r.fifth[1], r.fifth[2], r.fifth[3],
                        r.fifth[4], tot / tot_off, tot / tot_base);
        }
    }

    std::printf("\n3. BUDGET of the chosen shape vs amount.\n");
    std::printf("%8s %9s %9s %9s\n", "amount", "tip", "total", "peak");
    {
        for (double m : {0.0, 0.8, 1.0, 1.2, 1.5}) {
            R r = run(1.4, 0.65, 1.0, m, 0.5);
            double tot = 0.0;
            for (int f = 0; f < 5; ++f) tot += r.fifth[f];
            std::printf("%8.2f %9.0f %9.0f %8.4f (%.2fx)\n",
                        m, r.fifth[4], tot, r.peak, r.peak / off.peak);
        }
    }

    for (int cfg = 0; cfg < 3; ++cfg) {
    g_lo = (cfg == 0) ? 1.0 : (cfg == 1 ? 0.8 : 0.7);
    g_hi = (cfg == 0) ? 1.0 : (cfg == 1 ? 1.25 : 1.6);
    std::printf("\n4. SHIPPED mapping, comp clamp [%.2f, %.2f]\n", g_lo, g_hi);
    std::printf("   travel and size both follow Wind; response compensated.\n");
    std::printf("%6s %8s %8s %8s %9s %9s %9s %9s\n",
                "Wind", "travel", "waves", "amount", "f2", "f3", "tip", "peak");
    {
        const double lo = 0.55, hi = 1.25;
        const double tv[] = {0.35,0.50,0.65,0.80,1.00,1.20,1.45,1.80,2.40};
        const double tr[] = {0.286,0.346,0.476,0.794,1.000,0.593,0.286,0.166,0.114};
        for (double w : {0.0, 0.25, 0.5, 0.75, 1.0}) {
            const double t = lo * std::pow(hi / lo, w);
            double rr = tr[8];
            for (int i = 0; i < 8; ++i)
                if (t <= tv[i + 1]) {
                    const double f = std::log(t / tv[i]) / std::log(tv[i + 1] / tv[i]);
                    rr = std::exp(std::log(tr[i]) + f * std::log(tr[i + 1] / tr[i]));
                    break;
                }
            const double comp = std::min(std::max(0.82 / rr, g_lo), g_hi);
            R r = run(1.4, 0.65, t, 1.2 * comp, w);
            std::printf("%6.2f %8.3f %8.1f %8.3f %9.0f %9.0f %9.0f %7.4f (%.2fx)\n",
                        w, t, 6.0 * std::pow(20.0 / 6.0, w), 1.2 * comp,
                        r.fifth[2], r.fifth[3], r.fifth[4], r.peak,
                        r.peak / off.peak);
        }
    }
    }

    std::printf("\n5. FLAP -- how hard the mid band drives the FREE END.\n");
    std::printf("   shipped shape, Wind 0.5. f4 is the tip fifth.\n");
    std::printf("%7s %9s %9s %9s %9s %9s %9s\n",
                "flap", "f1", "f2", "f3", "tip", "tip/f3", "peak");
    for (double fl : {0.0, 0.25, 0.45, 0.7, 1.0}) {
        R r = run(1.4, 0.65, 0.829, 1.194, 0.5, 14.0, fl);
        std::printf("%7.2f %9.0f %9.0f %9.0f %9.0f %9.2f %7.4f (%.2fx)\n",
                    fl, r.fifth[1], r.fifth[2], r.fifth[3], r.fifth[4],
                    r.fifth[4] / std::max(1.0, r.fifth[3]), r.peak,
                    r.peak / off.peak);
    }

    // O1: Flap and Wrinkle sit OUTSIDE the Motion excitation budget, so
    // they ADD amplitude instead of redistributing it. The sim is mapped to
    // +-1 for the EQ, and past that the control signal clamps -- which reads
    // as the string vibrating hard without travelling, because the peaks are
    // being flat-topped. This measures the combination rather than
    // multiplying the two separately-measured factors and hoping.
    std::printf("\n6. O1 -- FLAP + WRINKLE vs the +-1 clamp (Wind 0.5).\n");
    std::printf("   peak is the sim displacement; >1.00 means the control\n");
    std::printf("   signal is being clipped before it reaches the bank.\n\n");
    std::printf("%7s %9s %9s %9s %9s\n", "flap", "wrinkle", "tip", "peak", "clipped");
    for (double fl : {0.0, 0.5, 1.0})
        for (double wr : {0.0, 1.95, 3.9}) {
            R r = run(1.4, 0.65, 0.829, 1.194, 0.5, 14.0, fl, wr);
            std::printf("%7.2f %9.2f %9.0f %9.4f %9s\n", fl, wr,
                        r.fifth[4], r.peak, r.peak > 1.0 ? "YES" : "-");
        }

    return 0;
}
