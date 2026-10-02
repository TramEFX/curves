// What does the Wind knob actually do, and does the control smoother eat it?
//
// Measures, at each Wind setting:
//   - the SIM's own motion (peak, and the dominant temporal frequency of a
//     probe node), i.e. the physics before anything downstream touches it;
//   - the same quantities AFTER the control smoother at several Smoothing
//     settings, which is what the EQ and the drawn curve actually see.
// If column A moves and column B does not, the smoother is the problem.

#include "string_sim.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace curves;

namespace {
constexpr double kPi = 3.14159265358979323846;

struct Out {
    double sim_peak, sim_hz, sm_peak, sm_hz, travel_nodes_s;
};

// Dominant frequency by centroid of the magnitude spectrum, DC excluded.
double dominant_hz(const std::vector<double>& x, double rate) {
    const int n = int(x.size());
    double mean = 0.0;
    for (double v : x) mean += v;
    mean /= n;
    double num = 0.0, den = 0.0;
    const int bins = std::min(n / 2, 400);
    for (int k = 1; k < bins; ++k) {
        double re = 0.0, im = 0.0;
        for (int i = 0; i < n; ++i) {
            const double a = -2.0 * kPi * k * i / n;
            const double v = x[i] - mean;
            re += v * std::cos(a);
            im += v * std::sin(a);
        }
        const double mag = std::sqrt(re * re + im * im);
        const double f = k * rate / n;
        num += mag * f;
        den += mag;
    }
    return den > 1e-12 ? num / den : 0.0;
}

// ratio_mode: wind_phase_speed = wind * c  (i.e. `wind` IS f_gust/f0)
Out measure(double wind, double smoothing_ms, int prec, double motion,
            double seconds, bool ratio_mode = false, double damping = 1.2) {
    const int nodes[3] = {128, 256, 512};
    const int divs[3] = {4, 2, 2};
    const int n = nodes[prec];
    const double fs = 48000.0, tick = fs / divs[prec];

    StringParams p;
    p.tick_rate = tick;
    p.wave_speed = 300.0;
    p.damping = damping;
    p.stiffness = 2.0;
    StringSim sim(n, p);
    sim.anchor_high = true;
    sim.force_model = StringSim::kClothAero;
    sim.set_cloth_aero(192.0 * n / 128.0, double(n), 1.5);
    sim.aero_rho_cd = 0.6;
    sim.set_density_taper(2.0);
    sim.set_cloth_length(1.0);
    sim.set_walls(0.0, 1.0);

    // Character 0 -> all gust wind. Motion mapping, Tension 300 so
    // tension_norm == 1 whatever the exponent.
    const double node_norm = std::pow(128.0 / n, 0.25);
    const double gain[3] = {1.590, 1.031, 0.913};
    const double boost[3] = {0.969, 1.007, 0.968};
    const double target = 0.85 * motion * boost[prec];
    const double dial = std::clamp(std::pow(target / 0.8720, 1.0 / 1.679), 0.0, 2.0);
    sim.wind_speed = 25.0 * dial * node_norm * gain[prec];
    sim.wrinkle_speed = sim.wind_speed;
    sim.shed_gain = 0.0;
    sim.wave_density = 0.5 + 4.5 * 0.4;
    sim.wrinkle = 0.0;
    // ratio_mode false = OLD mapping; true = SHIPPED mapping (0.12*25^Wind)*c
    sim.wind_phase_speed = ratio_mode
                               ? 0.12 * std::pow(25.0, wind) * 300.0
                               : (10.0 + 490.0 * wind) * (double(n) / 128.0);

    const double alpha = 1.0 - std::exp(-1.0 / ((smoothing_ms * 1e-3) * tick));
    std::vector<double> sd(n, 0.0);

    const long long ticks = (long long)(seconds * tick);
    const long long warm = ticks / 4;
    // probe two nodes a known distance apart to time the travel
    const int pa = n / 4, pb = n / 2;
    std::vector<double> traceA, traceB, traceS;
    double sim_peak = 0.0, sm_peak = 0.0;
    const int decim = std::max(1, int(tick / 200.0));  // ~200 Hz trace
    for (long long t = 0; t < ticks; ++t) {
        sim.tick();
        const auto& u = sim.positions();
        for (int i = 0; i < n; ++i)
            sd[i] += alpha * (u[i] - sd[i]);
        if (t < warm) continue;
        for (int i = 0; i < n; ++i) {
            sim_peak = std::max(sim_peak, std::fabs(u[i]));
            sm_peak = std::max(sm_peak, std::fabs(sd[i]));
        }
        if ((t - warm) % decim == 0) {
            traceA.push_back(u[pa]);
            traceB.push_back(u[pb]);
            traceS.push_back(sd[pa]);
        }
    }
    const double trace_rate = tick / decim;
    Out o;
    o.sim_peak = sim_peak;
    o.sm_peak = sm_peak;
    o.sim_hz = dominant_hz(traceA, trace_rate);
    o.sm_hz = dominant_hz(traceS, trace_rate);
    // travel speed: lag of best cross-correlation between the two probes
    double best = -1e30; int best_lag = 0;
    const int maxlag = int(trace_rate * 2.0);
    for (int lag = -maxlag; lag <= maxlag; ++lag) {
        double acc = 0.0; int cnt = 0;
        for (size_t i = 0; i < traceA.size(); ++i) {
            const long j = long(i) + lag;
            if (j < 0 || j >= (long)traceB.size()) continue;
            acc += traceA[i] * traceB[j];
            ++cnt;
        }
        if (cnt > 10 && acc / cnt > best) { best = acc / cnt; best_lag = lag; }
    }
    const double dt_lag = best_lag / trace_rate;
    o.travel_nodes_s = std::fabs(dt_lag) > 1e-6 ? (pb - pa) / dt_lag : 0.0;
    return o;
}
}  // namespace

int main(int argc, char** argv) {
    const double motion = argc > 1 ? std::atof(argv[1]) : 0.45;
    const int nodes[3] = {128, 256, 512};
    std::printf("Wind knob: OLD mapping vs SHIPPED mapping.%s", "\n");
    std::printf("Wanted: motion frequency rising monotonically across the dial,%s", "\n");
    std::printf("at every precision, with amplitude not collapsing.%s", "\n\n");
    for (int prec : {0, 2}) {
        for (int mode = 0; mode < 2; ++mode) {
            std::printf("--- %d nodes, %s ---%s", nodes[prec],
                        mode ? "SHIPPED" : "old", "\n");
            std::printf("%8s %12s %10s %10s%s", "Wind", "phase_sp",
                        "SIM peak", "SIM Hz", "\n");
            for (double w : {0.0, 0.25, 0.5, 0.75, 1.0}) {
                Out a = measure(w, 2.0, prec, motion, 20.0, mode == 1);
                const double v = mode ? 0.12 * std::pow(25.0, w) * 300.0
                                      : (10.0 + 490.0 * w) * (nodes[prec] / 128.0);
                std::printf("%8.2f %12.1f %10.4f %10.3f%s", w, v,
                            a.sim_peak, a.sim_hz, "\n");
            }
            std::printf("%s", "\n");
        }
    }
    return 0;
}
