// Does the randomiser actually randomise, and what does it cost?
//
//  - spread 0 must reproduce the fixed constant sets exactly;
//  - different seeds must give genuinely different behaviour;
//  - amplitude must not run away;
//  - and for FLUTTER specifically: the outward travel direction is a
//    project invariant, so check how far spread erodes it. Frequency
//    spread necessarily blurs the phase ramp that sets direction.

#include "string_sim.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace curves;

namespace {
constexpr int kN = 256;
constexpr double kTick = 24000.0;

StringSim* make() {
    StringParams p;
    p.tick_rate = kTick; p.wave_speed = 300.0;
    p.damping = 1.2; p.stiffness = 2.0;
    auto* s = new StringSim(kN, p);
    s->anchor_high = true;
    s->force_model = StringSim::kClothAero;
    s->set_cloth_aero(192.0 * kN / 128.0, double(kN), 1.5);
    s->aero_rho_cd = 0.6;
    s->set_density_taper(2.0);
    s->set_cloth_length(1.0);
    s->wave_density = 0.5 + 7.5 * 0.24;
    return s;
}

// energy-flux direction: positive = travelling AWAY from the pole
double travel_dir(StringSim& sim, double seconds) {
    // p is distance from the POLE, so dp/di flips with the wind origin.
    // Hardcoding +1 here silently inverted every reading -- the same class
    // of error that once produced an exactly backwards verdict on this very
    // question. Matches scenarios_main.cpp.
    const double dpdi = (sim.wind_origin == 1) ? 1.0 : -1.0;
    double fnum = 0.0, fden_v = 0.0, fden_p = 0.0;
    const long long ticks = (long long)(seconds * kTick);
    const long long warm = ticks / 3;
    for (long long t = 0; t < ticks; ++t) {
        sim.tick();
        if (t < warm) continue;
        const auto& u = sim.positions();
        const auto& v = sim.velocities();
        for (int i = 1; i + 1 < kN; ++i) {
            const double u_p = 0.5 * (u[i + 1] - u[i - 1]) * dpdi;
            fnum -= v[i] * u_p;
            fden_v += v[i] * v[i];
            fden_p += u_p * u_p;
        }
    }
    return fnum / std::sqrt(std::max(fden_v * fden_p, 1e-30));
}

double peak_of(StringSim& sim, double seconds) {
    const long long ticks = (long long)(seconds * kTick);
    const long long warm = ticks / 3;
    double pk = 0.0;
    for (long long t = 0; t < ticks; ++t) {
        sim.tick();
        if (t < warm) continue;
        const auto& u = sim.positions();
        for (int i = 0; i < kN; ++i) pk = std::max(pk, std::fabs(u[i]));
    }
    return pk;
}
}  // namespace

int main() {
    // METRIC VALIDATION FIRST: reproduce the shed scenario's configuration
    // and check this code returns its recorded +0.372 before believing any
    // other number it prints.
    {
        StringParams p;
        p.tick_rate = 1000.0; p.wave_speed = 300.0;
        p.damping = 1.2; p.stiffness = 2.0;
        StringSim v(128, p);
        // EXACTLY scenario_shed: no anchors, shed_gain 60, default tilt
        // handling. Earlier this block added anchor_high and used gain 30,
        // and the check correctly flagged the result as untrustworthy.
        v.set_cloth_aero(192.0, 128.0, 1.5);
        v.wind_speed = 0.0;
        v.aero_rho_cd = 0.6;
        v.set_density_taper(2.0);
        v.set_cloth_length(1.0);
        v.shed_gain = 60.0;
        double fnum = 0.0, fv = 0.0, fp = 0.0;
        const double dpdi = -1.0;
        for (int s = 0; s < 6000; ++s) {
            v.tick();
            if (s / 1000.0 < 2.0) continue;
            const auto& u = v.positions();
            const auto& vv = v.velocities();
            for (int i = 1; i + 1 < 128; ++i) {
                const double u_p = 0.5 * (u[i + 1] - u[i - 1]) * dpdi;
                fnum -= vv[i] * u_p; fv += vv[i] * vv[i]; fp += u_p * u_p;
            }
        }
        const double d = fnum / std::sqrt(std::max(fv * fp, 1e-30));
        std::printf("METRIC CHECK: shed scenario direction = %+.4f"
                    "  (recorded +0.372)  %s\n\n", d,
                    d > 0.2 ? "OK" : "SUSPECT -- do not trust what follows");
    }

    std::printf("A. WIND randomiser: does the seed change the weather?\n");
    std::printf("   gust_spread 0 must reproduce the constants exactly.\n\n");
    std::printf("%10s %8s %12s\n", "spread", "seed", "peak |u|");
    for (double sp : {0.0, 0.5, 1.0}) {
        for (int sd : {1, 7, 20260103}) {
            StringSim* s = make();
            s->wind_speed = 25.0 * std::pow(128.0 / kN, 0.25);
            s->gust_spread = sp; s->gust_seed = sd;
            std::printf("%10.2f %8d %12.4f\n", sp, sd, peak_of(*s, 30.0));
            delete s;
            if (sp == 0.0) break;   // seed is irrelevant at spread 0
        }
    }

    std::printf("\nB. FLUTTER randomiser, with NO gust wind so the wake bank\n");
    std::printf("   is the only thing moving. Direction is the invariant:\n");
    std::printf("   positive = travelling away from the pole.\n\n");
    std::printf("%10s %8s %12s %12s\n", "spread", "seed", "direction", "peak |u|");
    for (double sp : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        StringSim* s = make();
        s->wind_speed = 0.0;
        s->shed_gain = 30.0;
        s->shed_freq_hz = 3.0;
        s->shed_spread = sp;
        s->shed_seed = 7;
        const double d = travel_dir(*s, 24.0);
        delete s;
        StringSim* s2 = make();
        s2->wind_speed = 0.0;
        s2->shed_gain = 30.0;
        s2->shed_freq_hz = 3.0;
        s2->shed_spread = sp;
        s2->shed_seed = 7;
        std::printf("%10.2f %8d %12.4f %12.4f\n", sp, 7, d, peak_of(*s2, 24.0));
        delete s2;
    }
    // Does the flutter SEED change anything? Section B only swept spread
    // at one seed, which cannot answer that. Owner reports the waves look
    // "extremely even at any setting", so the question is whether reseeding
    // moves the result and by how much.
    std::printf("\nC. FLUTTER seeds at fixed spread -- do they differ?\n");
    std::printf("   \"unevenness\" = sd across nodes of the wake amplitude\n");
    std::printf("   field, which is what a seed is supposed to reshape.\n\n");
    std::printf("%10s %12s %12s %14s\n",
                "spread", "seed", "peak |u|", "amp field sd");
    for (double sp : {0.5, 1.0}) {
        double lo = 1e9, hi = -1e9;
        for (int sd : {1, 7, 99, 20260103}) {
            StringSim* s = make();
            s->wind_speed = 0.0;
            s->shed_gain = 30.0;
            s->shed_freq_hz = 3.0;
            s->shed_spread = sp;
            s->shed_seed = sd;
            const double pk = peak_of(*s, 24.0);
            // the per-node amplitude field the seed builds
            const auto& amp = s->shed_amp_field();
            double m = 0.0;
            for (double a2 : amp) m += a2;
            m /= std::max<std::size_t>(1, amp.size());
            double v = 0.0;
            for (double a2 : amp) v += (a2 - m) * (a2 - m);
            v = std::sqrt(v / std::max<std::size_t>(1, amp.size()));
            lo = std::min(lo, pk);
            hi = std::max(hi, pk);
            std::printf("%10.2f %12d %12.4f %14.4f\n", sp, sd, pk, v);
            delete s;
        }
        std::printf("%10.2f %12s %12.4f  <- range across seeds\n\n",
                    sp, "RANGE", hi - lo);
    }

    return 0;
}
