// How much does Tension squash the string?
//
// Replicates the plugin's Flagpole mapping exactly (PluginProcessor.cpp) and
// sweeps Tension with every other control held at its default, measuring the
// amplitude that survives. tension_norm = (c/300)^0.6 is supposed to cancel
// this; the question is how much residual is left.

#include "string_sim.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace curves;

namespace {
// --- constants lifted from PluginProcessor.cpp ---------------------------
constexpr double kRateRatio[] = {0.15, 0.30, 0.50, 0.75, 1.00,
                                 1.50, 2.50, 4.00, 7.00, 12.0, 25.0};
constexpr double kRateResp[] = {0.23, 0.43, 0.75, 0.93, 1.00,
                                0.93, 0.63, 0.34, 0.117, 0.043, 0.010};
constexpr int kRateN = 11;
constexpr double kRateRef = 0.213;
constexpr double kRateCompMin = 0.25, kRateCompMax = 8.0;
constexpr double kWindA = 0.8720, kWindP = 1.679;
constexpr double kShedA = 1.2756, kShedP = 0.707;
constexpr double kMotionPeak = 0.85;
constexpr double kNodeWindGain[3] = {1.590, 1.031, 0.913};
constexpr double kNodeShedGain[3] = {2.796, 0.978, 1.099};
constexpr double kMixC[] = {0.0, 0.33, 0.66, 1.0};
constexpr double kMixBoost[3][4] = {{0.969, 1.462, 1.106, 0.847},
                                    {1.007, 1.340, 1.017, 0.999},
                                    {0.968, 0.978, 0.805, 1.012}};
constexpr int kNodes[3] = {128, 256, 512};
constexpr int kDiv[3] = {4, 2, 2};

double mix_boost(double c, int prec) {
    c = std::clamp(c, 0.0, 1.0);
    const double* row = kMixBoost[std::clamp(prec, 0, 2)];
    for (int i = 0; i + 1 < 4; ++i) {
        if (c > kMixC[i + 1]) continue;
        const double t = (c - kMixC[i]) / (kMixC[i + 1] - kMixC[i]);
        return row[i] + t * (row[i + 1] - row[i]);
    }
    return row[3];
}
double dial_for_peak(double target, double A, double P) {
    if (target <= 1e-9) return 0.0;
    return std::clamp(std::pow(target / A, 1.0 / P), 0.0, 2.0);
}
double flutter_rate_compensation(double ratio) {
    const double r = std::clamp(ratio, kRateRatio[0], kRateRatio[kRateN - 1]);
    double resp = kRateResp[kRateN - 1];
    for (int i = 0; i + 1 < kRateN; ++i) {
        if (r > kRateRatio[i + 1]) continue;
        const double t = std::log(r / kRateRatio[i]) /
                         std::log(kRateRatio[i + 1] / kRateRatio[i]);
        resp = std::exp(std::log(kRateResp[i]) +
                        t * std::log(kRateResp[i + 1] / kRateResp[i]));
        break;
    }
    return std::clamp(kRateRef / std::max(resp, 1e-9), kRateCompMin, kRateCompMax);
}

struct Result { double peak, rms; };

// `tnorm_p` is the exponent in tension_norm = (c/300)^p — the knob under test.
Result measure(double tension, double motion, double character, int prec,
               double flutter_hz, double seconds, double tnorm_p, double shed_q) {
    const int n = kNodes[prec];
    const double fs = 48000.0, tick_rate = fs / kDiv[prec];

    StringParams p;
    p.tick_rate = tick_rate;
    p.wave_speed = tension;
    p.damping = 1.2;    // Flagpole preset
    p.stiffness = 2.0;
    StringSim sim(n, p);
    sim.anchor_high = true;
    sim.force_model = StringSim::kClothAero;
    sim.set_cloth_aero(192.0 * n / 128.0, double(n), 1.5);
    sim.aero_rho_cd = 0.6;
    sim.set_density_taper(2.0);
    sim.set_cloth_length(1.0);
    sim.set_walls(0.0, 1.0);

    const double tension_norm = std::pow(std::max(1.0, tension) / 300.0, tnorm_p);
    const double node_norm = std::pow(128.0 / n, 0.25);
    const double target_peak = kMotionPeak * motion * mix_boost(character, prec);
    const double wind_dial = dial_for_peak((1.0 - character) * target_peak, kWindA, kWindP);
    const double shed_dial = dial_for_peak(character * target_peak, kShedA, kShedP);

    sim.wind_speed = 25.0 * wind_dial * tension_norm * node_norm * kNodeWindGain[prec];
    sim.wrinkle_speed = sim.wind_speed;
    const double span = n - 1.0;
    const double f0 = std::max(1e-6, tension / span);
    const double comp = flutter_rate_compensation(flutter_hz / f0);
    // shed_q: the impedance term the rate compensation is missing. R(f/f0)
    // captures the resonance SHAPE but is peak-NORMALIZED, so it carries no
    // information about the string getting stiffer as c rises.
    sim.shed_gain = 90.0 * shed_dial * comp * kNodeShedGain[prec]
                    * std::pow(std::max(1.0, tension) / 300.0, shed_q);
    sim.shed_freq_hz = flutter_hz;
    sim.wind_phase_speed = (10.0 + 490.0 * 0.37) * (double(n) / 128.0);  // Wind default
    sim.wave_density = 0.5 + 4.5 * 0.4;                                  // Waves default
    sim.wrinkle = 0.0;

    const long long ticks = (long long)(seconds * tick_rate);
    const long long warm = ticks / 4;
    double peak = 0.0, acc = 0.0;
    long long cnt = 0;
    for (long long t = 0; t < ticks; ++t) {
        sim.tick();
        if (t < warm) continue;
        const auto& u = sim.positions();
        double fpk = 0.0, fsq = 0.0;
        for (int i = 0; i < n; ++i) {
            fpk = std::max(fpk, std::fabs(u[i]));
            fsq += u[i] * u[i];
        }
        peak = std::max(peak, fpk);
        acc += fsq / n;
        ++cnt;
    }
    return {peak, std::sqrt(acc / std::max(1LL, cnt))};
}

void sweep(const char* title, double motion, double character, int prec,
           double tnorm_p, double shed_q) {
    std::printf("\n%s   (Motion %.2f, Character %.2f, %d nodes, exponent %.2f)\n",
                title, motion, character, kNodes[prec], tnorm_p);
    std::printf("  %8s %10s %10s %12s\n", "Tension", "peak", "rms", "rel. peak");
    const double tens[] = {50, 100, 200, 300, 450, 600};
    double base = 0.0, lo = 1e9, hi = 0.0;
    for (double c : tens) {
        Result r = measure(c, motion, character, prec, 3.0, 20.0, tnorm_p, shed_q);
        if (c == 300.0) base = r.peak;
        lo = std::min(lo, r.peak);
        hi = std::max(hi, r.peak);
        std::printf("  %8.0f %10.4f %10.4f", c, r.peak, r.rms);
        if (base > 0.0 && c >= 300.0)
            std::printf(" %11.2fx", r.peak / base);
        std::printf("\n");
    }
    std::printf("  spread across the dial: %.2fx\n", hi / std::max(lo, 1e-9));
}
}  // namespace

// Harness validation: reproduce scenario_aero exactly and check against its
// recorded figure. If this matches, the sim is being driven correctly and any
// disagreement with the Motion budget is in the budget, not here.
void selftest() {
    StringParams p;
    p.tick_rate = 1000.0;   // scenarios run at 1 kHz, NOT the plugin rate
    p.wave_speed = 300.0;
    p.damping = 1.2;
    p.stiffness = 2.0;
    StringSim sim(128, p);
    sim.remove_tilt = false;         // the frozen baselines opt out
    sim.anchor_high = true;
    sim.set_cloth_aero(192.0, 128.0, 1.5);
    sim.wind_speed = 37.0;
    sim.aero_rho_cd = 0.6;
    sim.set_density_taper(2.0);
    sim.set_cloth_length(1.0);
    double peak = 0.0;
    for (int s = 0; s < 4000; ++s) {
        sim.tick();
        // the scenario samples every 100 ticks and scores t >= 1.5 s
        if (s % 100 == 0 && s / 1000.0 >= 1.5) {
            const auto& u = sim.positions();
            for (int i = 0; i < 128; ++i) peak = std::max(peak, std::fabs(u[i]));
        }
    }
    // How much does the TICK RATE alone change the answer? The plugin runs
    // this same configuration at 12 kHz / 24 kHz, not 1 kHz.
    for (double tr : {1000.0, 3000.0, 6000.0, 12000.0, 24000.0}) {
        StringParams q;
        q.tick_rate = tr; q.wave_speed = 300.0; q.damping = 1.2; q.stiffness = 2.0;
        StringSim s2(128, q);
        s2.remove_tilt = false;
        s2.anchor_high = true;
        s2.set_cloth_aero(192.0, 128.0, 1.5);
        s2.wind_speed = 37.0; s2.aero_rho_cd = 0.6;
        s2.set_density_taper(2.0); s2.set_cloth_length(1.0);
        double pk = 0.0;
        const int n2 = int(4.0 * tr);
        for (int t = 0; t < n2; ++t) {
            s2.tick();
            if (t / tr >= 1.5) {
                const auto& u = s2.positions();
                for (int i = 0; i < 128; ++i) pk = std::max(pk, std::fabs(u[i]));
            }
        }
        std::printf("   tick rate %6.0f Hz -> peak %.4f%s", tr, pk, "\n");
    }
    std::printf("SELFTEST scenario_aero late peak|u| = %.6f  (recorded 0.722754)%s",
                peak, "\n");
    std::printf("         %s%s", std::fabs(peak - 0.722754) < 1e-5 ? "MATCH" : "MISMATCH",
                "\n\n");
}

int main(int argc, char** argv) {
    const double p = argc > 1 ? std::atof(argv[1]) : 0.6;
    const double q = argc > 2 ? std::atof(argv[2]) : 0.0;
    const int which = argc > 3 ? std::atoi(argv[3]) : 9;
    selftest();
    std::printf("Peak |u| vs Tension, everything else at its default.\n");
    std::printf("A flat column means Tension changes CHARACTER, not LOUDNESS.\n");
    if (which == 0 || which == 9) sweep("Character 0 (all gust wind)", 0.45, 0.0, 0, p, q);
    if (which == 1 || which == 9) sweep("Character 1 (all shedding)", 0.45, 1.0, 0, p, q);
    if (which == 2 || which == 9) sweep("Character 0.5 (mixed)", 0.45, 0.5, 0, p, q);
    if (which == 3 || which == 9) sweep("Character 0, 512 nodes", 0.45, 0.0, 2, p, q);
    if (which == 4 || which == 9) sweep("Character 1, 512 nodes", 0.45, 1.0, 2, p, q);
    if (which == 5 || which == 9) sweep("Character 0, Motion 0.8", 0.80, 0.0, 0, p, q);
    if (which == 6 || which == 9) sweep("Character 0, 256 nodes", 0.45, 0.0, 1, p, q);
    return 0;
}
