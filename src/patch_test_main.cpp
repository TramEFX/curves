// "What causes these big slow moving curves at these settings?"
// Reconstructs the screenshot patch -- low Tension, low Waves, Flutter
// powered OFF, Motion/Wind on High magnitude -- and isolates which control
// owns the SIZE of the big arc and which owns its RATE.

#include "string_sim.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace curves;

namespace {
constexpr int kN = 512, kDiv = 2;
constexpr double kFs = 48000.0, kTick = kFs / kDiv;
constexpr double kPi = 3.14159265358979323846;

// dominant spatial wavenumber (waves across the string), energy-weighted
void spatial(const std::vector<double>& u, int n, double& centroid, double& k1_amp) {
    double mean = 0.0;
    for (int i = 0; i < n; ++i) mean += u[i];
    mean /= n;
    double num = 0.0, den = 0.0;
    k1_amp = 0.0;
    for (int k = 1; k <= 40; ++k) {
        double re = 0.0, im = 0.0;
        for (int i = 0; i < n; ++i) {
            const double a = -2.0 * kPi * k * i / n;
            const double v = u[i] - mean;
            re += v * std::cos(a); im += v * std::sin(a);
        }
        const double mag = std::sqrt(re*re + im*im) / n;
        num += mag * k; den += mag;
        if (k <= 3) k1_amp = std::max(k1_amp, mag);
    }
    centroid = den > 1e-12 ? num / den : 0.0;
}

struct Out { double centroid, rate_s, swing; };

Out run(double tension, double waves_knob, bool flutter_on, double motion,
        double wind_knob, double seconds = 90.0) {
    StringParams p;
    p.tick_rate = kTick; p.wave_speed = tension;
    p.damping = 1.2; p.stiffness = 2.0;
    StringSim sim(kN, p);
    sim.anchor_high = true;
    sim.force_model = StringSim::kClothAero;
    sim.set_cloth_aero(192.0 * kN / 128.0, double(kN), 1.5);
    sim.aero_rho_cd = 0.6;
    sim.set_density_taper(2.0);
    sim.set_cloth_length(1.0);
    sim.set_walls(0.0, 1.0);

    const double density = 0.5 + 7.5 * waves_knob;
    const double node_norm = std::pow(128.0 / kN, 0.25);
    const double tnorm = std::pow(std::max(1.0, tension) / 300.0, 0.60);
    const double target = 0.85 * motion * 0.968;
    const double dial = std::clamp(std::pow(target / 0.8720, 1.0 / 1.679), 0.0, 2.0);
    sim.wave_density = density;
    sim.wind_speed = 25.0 * dial * tnorm * node_norm * 0.913;
    sim.wrinkle_speed = sim.wind_speed;
    sim.shed_gain = flutter_on ? 90.0 * 0.5 : 0.0;
    sim.shed_freq_hz = 3.0;
    const double ratio = 0.12 * std::pow(25.0, wind_knob);
    sim.wind_phase_speed = ratio * tension;
    sim.mid_waves = 6.0 * std::pow(20.0 / 6.0, wind_knob);
    sim.mid_ripple = 1.2; sim.mid_lock_d = 0.65; sim.mid_env = 1.4;
    sim.mid_travel = 0.55 * std::pow(1.25 / 0.55, wind_knob);
    sim.mid_flap = 0.9;

    const long long ticks = (long long)(seconds * kTick);
    const long long warm = ticks / 5;
    const long long every = (long long)(kTick / 10);   // 10 Hz trace
    std::vector<double> low;
    double cacc = 0.0; int cn = 0;
    for (long long t = 0; t < ticks; ++t) {
        sim.tick();
        if (t < warm || (t - warm) % every) continue;
        const auto& u = sim.positions();
        double c, a;
        spatial(u, kN, c, a);
        cacc += c; ++cn;
        // signed projection on mode 2 = the big arc
        double mean = 0.0;
        for (int i = 0; i < kN; ++i) mean += u[i];
        mean /= kN;
        double re = 0.0;
        for (int i = 0; i < kN; ++i)
            re += (u[i] - mean) * std::cos(2.0 * kPi * 2.0 * i / kN);
        low.push_back(2.0 * re / kN);
    }
    Out o;
    o.centroid = cacc / std::max(1, cn);
    int flips = 0;
    double mn = 1e9, mx = -1e9;
    for (size_t i = 0; i < low.size(); ++i) {
        if (i && (low[i] > 0) != (low[i-1] > 0)) ++flips;
        mn = std::min(mn, low[i]); mx = std::max(mx, low[i]);
    }
    const double dur = low.size() / 10.0;
    o.rate_s = flips ? 2.0 * dur / flips : dur;
    o.swing = mx - mn;
    return o;
}
}  // namespace

int main() {
    std::printf("Big-arc behaviour. 'centroid' = dominant spatial wavenumber\n");
    std::printf("(waves across the string -- smaller = bigger arc).\n");
    std::printf("'period' = mean full cycle of the lowest surviving mode.\n\n");

    std::printf("%-46s %10s %10s %9s\n", "patch", "centroid", "period s", "swing");
    struct C { const char* n; double ten, wav; bool fl; double mo, wi; };
    const C cases[] = {
        {"SCREENSHOT-LIKE: Ten 110, Waves .10, Flut OFF", 110, 0.10, false, 0.90, 0.80},
        {"  Tension 110 -> 300 (default-ish)",            300, 0.10, false, 0.90, 0.80},
        {"  Tension 110 -> 600",                          600, 0.10, false, 0.90, 0.80},
        {"  Waves .10 -> .40",                            110, 0.40, false, 0.90, 0.80},
        {"  Waves .10 -> 1.0",                            110, 1.00, false, 0.90, 0.80},
        {"  Flutter ON",                                  110, 0.10, true,  0.90, 0.80},
        {"  Motion .90 -> .35",                           110, 0.10, false, 0.35, 0.80},
        {"  Wind .80 -> .30",                             110, 0.10, false, 0.90, 0.30},
    };
    for (const auto& c : cases) {
        Out o = run(c.ten, c.wav, c.fl, c.mo, c.wi);
        std::printf("%-46s %10.2f %10.2f %9.3f\n", c.n, o.centroid, o.rate_s, o.swing);
    }
    std::printf("\nstring fundamental f0 = c/span:\n");
    for (double t : {110.0, 300.0, 600.0})
        std::printf("   Tension %3.0f -> f0 %.3f Hz  (mode-2 period %.2f s)\n",
                    t, t / 511.0, 511.0 / (2.0 * t));
    return 0;
}
