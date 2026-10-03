// Curves — C++ scenario runner: mirrors prototype/run_scenarios.py and
// writes CSVs in the same format (cpp_*.csv) so the C++ port can be
// validated numerically against the Python spec.

#include "string_sim.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace fs = std::filesystem;
using curves::StringParams;
using curves::StringSim;

namespace {

constexpr int kNodes = 128;
constexpr double kTickRate = 1000.0;

void save_frames(const fs::path& dir, const std::string& name,
                 const std::vector<std::pair<double, std::vector<double>>>& frames) {
    std::ofstream fh(dir / (name + ".csv"));
    fh << "scenario,tick,time_s,node,value\n";
    for (const auto& [time, u] : frames) {
        const int tick = static_cast<int>(std::lround(time * kTickRate));
        for (int i = 0; i < static_cast<int>(u.size()); ++i) {
            char tb[32], vb[64];
            std::snprintf(tb, sizeof(tb), "%.4f", time);
            std::snprintf(vb, sizeof(vb), "%.6f", u[i]);
            fh << name << ',' << tick << ',' << tb << ',' << i << ',' << vb << '\n';
        }
    }
    std::cout << "  -> " << (dir / (name + ".csv")).string() << " (" << frames.size()
              << " frames)\n";
}

double max_abs(const std::vector<double>& u) {
    double m = 0.0;
    for (double v : u) m = std::max(m, std::abs(v));
    return m;
}

void scenario_pluck(const fs::path& out) {
    std::cout << "scenario: pluck (draw, release, ring, settle)\n";
    StringParams p;
    p.tick_rate = kTickRate;
    p.wave_speed = 300.0;
    p.damping = 1.5;
    p.stiffness = 4.0;
    StringSim sim(kNodes, p);
    // Pre-tilt-blocker baseline: these scenarios are the frozen
    // regression tripwire, so they opt out of the mode-1 removal.
    sim.remove_tilt = false;
    sim.anchor_low = true;
    sim.anchor_high = true;
    sim.pluck(64, 1.0, 5.0);
    const double e_start = sim.energy();
    auto frames = sim.run(2.0, 0.05);
    save_frames(out, "cpp_pluck", frames);
    std::cout << "  energy start " << e_start << "  end " << sim.energy()
              << "  (should decay toward 0)\n";
}

void scenario_flagpole(const fs::path& out) {
    std::cout << "scenario: flagpole (anchor high end, wind blows leftward, then stops)\n";
    StringParams p;
    p.tick_rate = kTickRate;
    p.wave_speed = 300.0;
    p.damping = 1.2;
    p.stiffness = 2.0;
    StringSim sim(kNodes, p);
    // Pre-tilt-blocker baseline: these scenarios are the frozen
    // regression tripwire, so they opt out of the mode-1 removal.
    sim.remove_tilt = false;
    sim.anchor_high = true;
    // Zero-mean constraint (DC blocker) centers the sag; amplitude raised
    // to compensate.
    std::vector<double> wind(kNodes);
    for (int i = 0; i < kNodes; ++i)
        wind[i] = -12.0 * (1.0 - static_cast<double>(i) / (kNodes - 1));
    sim.set_force_field(wind);
    auto frames = sim.run(1.5, 0.1);
    const double sag = max_abs(frames.back().second);
    sim.clear_forces();
    auto frames_b = sim.run(1.0, 0.1);
    frames.insert(frames.end(), frames_b.begin(), frames_b.end());
    save_frames(out, "cpp_flagpole", frames);
    std::cout << "  sag under wind: max|u| = " << sag << "\n";
    std::cout << "  after release:  max|u| = " << max_abs(frames.back().second)
              << " (should sag back)\n";
}

void scenario_travel(const fs::path& out) {
    std::cout << "scenario: travel (localized bump -> genuine wave propagation)\n";
    StringParams p;
    p.tick_rate = kTickRate;
    p.wave_speed = 300.0;
    p.damping = 0.8;
    p.stiffness = 0.0;
    StringSim sim(kNodes, p);
    // Pre-tilt-blocker baseline: these scenarios are the frozen
    // regression tripwire, so they opt out of the mode-1 removal.
    sim.remove_tilt = false;
    sim.anchor_low = true;
    sim.anchor_high = true;
    sim.pluck(24, 1.0, 3.0);
    auto frames = sim.run(0.8, 0.02);
    save_frames(out, "cpp_travel", frames);
    std::cout << "  wavefront crossing time ~ " << kNodes / 300.0 << "s per traverse\n";
}

void scenario_recover(const fs::path& out) {
    std::cout << "scenario: recover (loose+undamped string, then tension back up)\n";
    StringParams p;
    p.tick_rate = kTickRate;
    p.wave_speed = 40.0;
    p.damping = 0.05;
    p.stiffness = 4.0;
    StringSim sim(kNodes, p);
    // Pre-tilt-blocker baseline: these scenarios are the frozen
    // regression tripwire, so they opt out of the mode-1 removal.
    sim.remove_tilt = false;
    sim.anchor_low = true;
    sim.anchor_high = true;
    sim.pluck(64, 1.0, 8.0);
    auto frames = sim.run(1.0, 0.25);
    sim.set_wave_speed(300.0);
    auto frames_b = sim.run(2.0, 0.25);
    frames.insert(frames.end(), frames_b.begin(), frames_b.end());
    save_frames(out, "cpp_recover", frames);
    double dc = 0.0, peak = 0.0;
    for (const auto& [t, u] : frames) {
        if (t < 1.25) continue;
        double mean = 0.0;
        for (double v : u) mean += v;
        mean /= u.size();
        dc = std::max(dc, std::abs(mean));
        peak = std::max(peak, max_abs(u));
    }
    std::cout << "  late |mean(u)| max: " << dc
              << "  (DC blocker keeps this ~0 — no stuck breathing)\n";
    std::cout << "  late max|u|:        " << peak << "  (energy still travels)\n";
}

void scenario_cloth(const fs::path& out, bool reversed) {
    std::cout << "scenario: cloth" << (reversed ? " reversed (low->high, length 0.8)" : "")
              << " (pole anchored, free whip end)\n";
    StringParams p;
    p.tick_rate = kTickRate;
    p.wave_speed = 300.0;
    p.damping = 1.2;
    p.stiffness = 2.0;
    StringSim sim(kNodes, p);
    // Pre-tilt-blocker baseline: these scenarios are the frozen
    // regression tripwire, so they opt out of the mode-1 removal.
    sim.remove_tilt = false;
    sim.anchor_high = true;  // the pole; the left end is FREE (whips)
    sim.set_flagpole_wind(192.0, 128.0, 1.5);
    sim.wind_strength = 400.0;
    sim.set_density_taper(2.0);
    sim.set_cloth_length(reversed ? 0.8 : 1.0);
    sim.flap_limit = 0.6;
    sim.set_wind_origin(reversed ? 1 : 0);
    std::vector<std::pair<double, std::vector<double>>> uframes;
    std::vector<std::vector<double>> vframes;
    for (int s = 0; s < 4000; ++s) {
        sim.tick();
        if (s % 100 == 0) {
            uframes.emplace_back(s / kTickRate, sim.positions());
            vframes.push_back(sim.velocities());
        }
    }
    save_frames(out, reversed ? "cpp_cloth_rev" : "cpp_cloth", uframes);
    double pole_u = 0, tip_u = 0, pole_v = 0, tip_v = 0, tail_u = 0;
    for (const auto& [t, u] : uframes) {
        if (t < 1.5) continue;
        for (int i = 0; i < kNodes; ++i) {
            if (i < kNodes / 8) tip_u = std::max(tip_u, std::abs(u[i]));
            if (i >= 7 * kNodes / 8) pole_u = std::max(pole_u, std::abs(u[i]));
        }
    }
    for (std::size_t fi = 15; fi < vframes.size(); ++fi) {
        for (int i = 0; i < kNodes; ++i) {
            if (i < kNodes / 8) tip_v = std::max(tip_v, std::abs(vframes[fi][i]));
            if (i >= 7 * kNodes / 8) pole_v = std::max(pole_v, std::abs(vframes[fi][i]));
        }
    }
    std::cout << "  pole_u " << pole_u << "  tip_u " << tip_u
              << "  (u-ratio " << tip_u / std::max(pole_u, 1e-6) << "x)\n";
    std::cout << "  pole_v " << pole_v << "  tip_v " << tip_v
              << "  (v-ratio " << tip_v / std::max(pole_v, 1e-6) << "x — the whip)\n";
    if (reversed) {
        const int n_end = static_cast<int>(std::lround((kNodes - 1) * 0.8));
        double whip_u = 0.0;
        for (const auto& [t, u] : uframes) {
            if (t < 1.5) continue;
            for (int i = 0; i < n_end; ++i)
                tail_u = std::max(tail_u, std::abs(u[i]));
            for (int i = n_end; i < kNodes; ++i)
                whip_u = std::max(whip_u, std::abs(u[i]));
        }
        std::cout << "  tail_u " << tail_u << " (damped)   whip_u " << whip_u
                  << " (at the highs)\n";
    }
}

// Cloth AERO: the wind is a velocity field and the force comes from
// RELATIVE velocity, so the string can fight the wind. Checks the whip is
// intact, amplitude is bounded without a flap limiter, and nothing blows up.
void scenario_aero(const fs::path& out) {
    std::cout << "scenario: aero (relative-velocity drag, no flap limiter)\n";
    StringParams p;
    p.tick_rate = kTickRate;
    p.wave_speed = 300.0;
    p.damping = 1.2;
    p.stiffness = 2.0;
    StringSim sim(kNodes, p);
    // Pre-tilt-blocker baseline: these scenarios are the frozen
    // regression tripwire, so they opt out of the mode-1 removal.
    sim.remove_tilt = false;
    sim.anchor_high = true;  // the pole; the left end is FREE (whips)
    sim.set_cloth_aero(192.0, 128.0, 1.5);
    // A deliberately hot wind that exercises the drag model; this is a
    // frozen baseline value, NOT the shipping dial top (the plugin maps
    // Force = 1.0 to 25 * (tension/300)^0.6 * (128/n)^0.25).
    sim.wind_speed = 37.0;
    sim.aero_rho_cd = 0.6;
    sim.set_density_taper(2.0);
    sim.set_cloth_length(1.0);
    // flap_limit deliberately left OFF: drag is the amplitude mechanism now.
    std::vector<std::pair<double, std::vector<double>>> uframes;
    std::vector<std::vector<double>> vframes;
    for (int s = 0; s < 4000; ++s) {
        sim.tick();
        if (s % 100 == 0) {
            uframes.emplace_back(s / kTickRate, sim.positions());
            vframes.push_back(sim.velocities());
        }
    }
    save_frames(out, "cpp_aero", uframes);
    double pole_u = 0, tip_u = 0, pole_v = 0, tip_v = 0, peak = 0;
    bool finite = true;
    for (const auto& [t, u] : uframes) {
        for (int i = 0; i < kNodes; ++i)
            if (!std::isfinite(u[i])) finite = false;
        if (t < 1.5) continue;
        for (int i = 0; i < kNodes; ++i) {
            peak = std::max(peak, std::abs(u[i]));
            if (i < kNodes / 8) tip_u = std::max(tip_u, std::abs(u[i]));
            if (i >= 7 * kNodes / 8) pole_u = std::max(pole_u, std::abs(u[i]));
        }
    }
    for (std::size_t fi = 15; fi < vframes.size(); ++fi)
        for (int i = 0; i < kNodes; ++i) {
            if (i < kNodes / 8) tip_v = std::max(tip_v, std::abs(vframes[fi][i]));
            if (i >= 7 * kNodes / 8) pole_v = std::max(pole_v, std::abs(vframes[fi][i]));
        }
    std::cout << "  pole_u " << pole_u << "  tip_u " << tip_u
              << "  (u-ratio " << tip_u / std::max(pole_u, 1e-6) << "x)\n";
    std::cout << "  pole_v " << pole_v << "  tip_v " << tip_v
              << "  (v-ratio " << tip_v / std::max(pole_v, 1e-6) << "x - the whip)\n";
    std::cout << "  late peak|u| " << peak
              << "  (bounded by drag, no limiter; under the +-1 EQ clamp)\n";
    std::cout << "  finite: " << (finite ? "yes" : "NO - BLEW UP") << "\n";
}

// Tilt blocker: an asymmetric deposit dumps most of its energy into mode 1
// (a linear tilt), which tension cannot restore. Without the blocker that
// mode rings as a slow global rocking that outlives every real wave. With
// it, the tilt stays at zero and only genuine waves remain.
void scenario_tilt(const fs::path& out) {
    std::cout << "scenario: tilt (mode-1 blocker; asymmetric deposit)\n";
    StringParams p;
    p.tick_rate = kTickRate;
    p.wave_speed = 300.0;
    p.damping = 0.4;   // low damping: where the rocking was worst
    p.stiffness = 4.0;
    StringSim sim(kNodes, p);   // remove_tilt defaults to true
    std::vector<double> shape(kNodes);
    for (int i = 0; i < kNodes; ++i)
        shape[i] = 0.6 * std::tanh(-2.5 + 5.0 * i / (kNodes - 1.0));
    sim.displace(shape);
    const double mid = 0.5 * (kNodes - 1.0);
    double sxx = 0.0;
    for (int i = 0; i < kNodes; ++i) sxx += (i - mid) * (i - mid);
    std::vector<std::pair<double, std::vector<double>>> frames;
    double late_tilt = 0.0, late_peak = 0.0;
    for (int s = 0; s < 8000; ++s) {
        sim.tick();
        if (s % 200 == 0) frames.emplace_back(s / kTickRate, sim.positions());
        const double t = s / kTickRate;
        if (t >= 3.0) {
            const auto& u = sim.positions();
            double sxu = 0.0, pk = 0.0;
            for (int i = 0; i < kNodes; ++i) {
                sxu += (i - mid) * u[i];
                pk = std::max(pk, std::abs(u[i]));
            }
            late_tilt = std::max(late_tilt, std::abs(sxu / sxx) * mid);
            late_peak = std::max(late_peak, pk);
        }
    }
    save_frames(out, "cpp_tilt", frames);
    std::cout << "  late tilt displacement " << late_tilt
              << "  (blocker holds it at ~0 - no slow rocking)\n";
    std::cout << "  late peak|u|           " << late_peak
              << "  (only genuine waves remain)\n";
}

// Vortex shedding: the wake oscillator is a genuine limit cycle, so the
// string must flap with the gust wind switched OFF ENTIRELY. That is the
// whole point - motion that is alive rather than driven. Also checks the
// wake decorrelates into cells instead of one global in-phase breath.
void scenario_shed(const fs::path& out) {
    std::cout << "scenario: shed (wake oscillator, ZERO gust wind)\n";
    StringParams p;
    p.tick_rate = kTickRate;
    p.wave_speed = 300.0;
    p.damping = 1.2;
    p.stiffness = 2.0;
    StringSim sim(kNodes, p);
    sim.set_cloth_aero(192.0, 128.0, 1.5);
    sim.wind_speed = 0.0;     // no gusting at all - shedding must self-sustain
    sim.aero_rho_cd = 0.6;
    sim.set_density_taper(2.0);
    sim.set_cloth_length(1.0);
    sim.shed_gain = 60.0;
    std::vector<std::pair<double, std::vector<double>>> frames;
    double peak = 0.0;
    int crossings = 0;
    double prev_probe = 0.0;
    bool finite = true;
    const int probe = kNodes / 4;
    // Wave momentum flux, accumulated online. u_t IS the velocity array, so
    // no time differencing is needed: for u = f(p - c t), u_t = -c f' and
    // u_p = f', so -sum(u_t * u_p) is positive when c > 0, i.e. when waves
    // travel away from the pole. Normalized to [-1, 1]. p is distance from
    // the pole, so dp/di flips sign with the wind origin.
    double fnum = 0.0, fden_v = 0.0, fden_p = 0.0, fden = 0.0;
    const double dpdi = (sim.wind_origin == 1) ? 1.0 : -1.0;
    double t_first = -1.0, t_last = 0.0;
    for (int s = 0; s < 6000; ++s) {
        sim.tick();
        if (s % 100 == 0) frames.emplace_back(s / kTickRate, sim.positions());
        const double t = s / kTickRate;
        const auto& u = sim.positions();
        for (int i = 0; i < kNodes; ++i)
            if (!std::isfinite(u[i])) finite = false;
        if (t >= 2.0) {
            peak = std::max(peak, std::abs(*std::max_element(
                u.begin(), u.end(),
                [](double a, double b) { return std::abs(a) < std::abs(b); })));
            const double cur = u[probe];
            if (prev_probe != 0.0 && ((cur > 0.0) != (prev_probe > 0.0))) {
                ++crossings;
                if (t_first < 0.0) t_first = t;
                t_last = t;
            }
            prev_probe = cur;
            const auto& v = sim.velocities();
            for (int i = 1; i + 1 < kNodes; ++i) {
                const double u_p = 0.5 * (u[i + 1] - u[i - 1]) * dpdi;
                fnum -= v[i] * u_p;
                fden_v += v[i] * v[i];
                fden_p += u_p * u_p;
            }
            fden = fden_v * fden_p;
        }
    }
    save_frames(out, "cpp_shed", frames);
    const double rate = (crossings > 2 && t_last > t_first)
                            ? 0.5 * (crossings - 1) / (t_last - t_first) : 0.0;
    const double flux = (fden > 0.0) ? fnum / std::sqrt(fden_v * fden_p) : 0.0;
    std::cout << "  travel direction  " << flux
              << "  (POSITIVE = waves move AWAY from the anchor)\n";
    std::cout << "  late peak|u|      " << peak
              << "  (self-sustained with NO gust wind - the alive test)\n";
    std::cout << "  probe modulation  " << rate << " Hz  (shed_freq_hz = 3.0)\n";
    std::cout << "  finite: " << (finite ? "yes" : "NO - BLEW UP") << "\n";
}

// Tip wrinkles: fine detail concentrated in the LAST THIRD. Asserts the
// detail is localized - tip curvature rises while the pole end stays put.
// Curvature (the second spatial difference) is used as the density proxy:
// it weights short wavelengths heavily, and is cheap and deterministic in
// both languages, unlike an FFT.
void scenario_wrinkle(const fs::path& out) {
    std::cout << "scenario: wrinkle (tip-resonant fine detail, last third)\n";
    StringParams p;
    p.tick_rate = kTickRate;
    p.wave_speed = 300.0;
    p.damping = 1.2;
    p.stiffness = 2.0;
    // Wrinkles need resolution: at 128 nodes their wavelengths land at
    // 10.7-4.1 nodes, at or past what the grid can carry, and the tip then
    // measures LESS detailed than the pole. This scenario therefore runs at
    // 512 nodes - the regime the feature is for. Usage note on record:
    // Wrinkle is a high-precision control.
    const int kWNodes = 512;
    p.tick_rate = kTickRate;
    StringSim sim(kWNodes, p);
    sim.set_cloth_aero(192.0 * kWNodes / 128.0, (double)kWNodes, 1.5);
    sim.wind_speed = 25.0 * std::pow(128.0 / kWNodes, 0.25);
    sim.aero_rho_cd = 0.6;
    sim.set_density_taper(2.0);
    sim.set_cloth_length(1.0);
    sim.wrinkle = 2.0;
    std::vector<std::pair<double, std::vector<double>>> frames;
    double tip_c = 0.0, tip_a = 0.0, pole_c = 0.0, pole_a = 0.0, peak = 0.0;
    bool finite = true;
    for (int s = 0; s < 6000; ++s) {
        sim.tick();
        if (s % 100 == 0) frames.emplace_back(s / kTickRate, sim.positions());
        if (s / kTickRate < 2.0) continue;
        const auto& u = sim.positions();
        for (int i = 0; i < kWNodes; ++i) {
            if (!std::isfinite(u[i])) finite = false;
            // peak over ALL nodes: the free whip end is usually the largest,
            // and excluding it made this disagree with the Python spec's
            // report even though the sims were bit-identical.
            peak = std::max(peak, std::abs(u[i]));
        }
        for (int i = 1; i + 1 < kWNodes; ++i) {
            const double curv = std::abs(u[i - 1] - 2.0 * u[i] + u[i + 1]);
            if (i < kWNodes / 3) { tip_c += curv; tip_a += std::abs(u[i]); }
            else if (i >= 2 * kWNodes / 3) { pole_c += curv; pole_a += std::abs(u[i]); }
        }
    }
    save_frames(out, "cpp_wrinkle", frames);
    const double tipd = tip_c / std::max(tip_a, 1e-12);
    const double poled = pole_c / std::max(pole_a, 1e-12);
    std::cout << "  tip   wrinkle density " << tipd << "\n";
    std::cout << "  pole  wrinkle density " << poled
              << "   (ratio " << tipd / std::max(poled, 1e-12)
              << "x - detail is localized at the END)\n";
    std::cout << "  late peak|u| " << peak << "   finite: "
              << (finite ? "yes" : "NO - BLEW UP") << "\n";
}

}  // namespace

void scenario_mid(const fs::path& out) {
    std::cout << "scenario: mid (mid-band ripples, resonance-locked mid-cloth)" << "\n";
    StringParams p;
    p.tick_rate = kTickRate;
    p.wave_speed = 300.0;
    p.damping = 1.2;
    p.stiffness = 2.0;
    const int kMNodes = 512;
    StringSim sim(kMNodes, p);
    sim.set_cloth_aero(192.0 * kMNodes / 128.0, (double)kMNodes, 1.5);
    sim.wind_speed = 25.0 * std::pow(128.0 / kMNodes, 0.25);
    sim.aero_rho_cd = 0.6;
    sim.set_density_taper(2.0);
    sim.set_cloth_length(1.0);
    sim.mid_ripple = 1.5;
    sim.mid_waves = 10.0;
    // the shipped shape, so this scenario covers what the plugin uses
    sim.mid_lock_d = 0.65;
    sim.mid_env = 1.4;
    sim.mid_travel = 0.83;
    sim.mid_flap = 0.45;
    std::vector<std::pair<double, std::vector<double>>> frames;
    double mid_c = 0.0, end_c = 0.0, peak = 0.0;
    bool finite = true;
    for (int s = 0; s < 6000; ++s) {
        sim.tick();
        if (s % 100 == 0) frames.emplace_back(s / kTickRate, sim.positions());
        if (s / kTickRate < 2.0) continue;
        const auto& u = sim.positions();
        for (int i = 0; i < kMNodes; ++i) {
            if (!std::isfinite(u[i])) finite = false;
            peak = std::max(peak, std::abs(u[i]));
        }
        // curvature in the middle third vs the two outer thirds: the bump
        // envelope should put the extra detail in the middle
        for (int i = 1; i + 1 < kMNodes; ++i) {
            const double curv = std::abs(u[i - 1] - 2.0 * u[i] + u[i + 1]);
            if (i >= kMNodes / 3 && i < 2 * kMNodes / 3) mid_c += curv;
            else end_c += curv;
        }
    }
    save_frames(out, "cpp_mid", frames);
    std::cout << "  mid-third curvature " << mid_c
              << "  outer " << end_c
              << "  (ratio " << mid_c / std::max(end_c, 1e-9) << "x)" << "\n";
    std::cout << "  late peak|u| " << peak
              << "   finite: " << (finite ? "yes" : "NO - BLEW UP") << "\n";
}

void scenario_random(const fs::path& out) {
    std::cout << "scenario: random (seeded spread on both gust and wake)" << "\n";
    StringParams p;
    p.tick_rate = kTickRate;
    p.wave_speed = 300.0;
    p.damping = 1.2;
    p.stiffness = 2.0;
    const int kRN = 256;
    StringSim sim(kRN, p);
    sim.anchor_high = true;
    sim.force_model = StringSim::kClothAero;
    sim.set_cloth_aero(192.0 * kRN / 128.0, (double)kRN, 1.5);
    sim.wind_speed = 25.0 * std::pow(128.0 / kRN, 0.25);
    sim.aero_rho_cd = 0.6;
    sim.set_density_taper(2.0);
    sim.set_cloth_length(1.0);
    sim.shed_gain = 30.0;
    sim.shed_freq_hz = 3.0;
    // the point of the scenario: both spreads non-zero, so the perturbed
    // constant sets and the per-node wake frequencies are exercised in BOTH
    // engines. At spread 0 this path is identical to the old one and the
    // frozen baselines prove nothing about it.
    sim.gust_spread = 0.7;
    sim.gust_seed = 20260103;
    sim.shed_spread = 0.6;
    sim.shed_seed = 7;
    std::vector<std::pair<double, std::vector<double>>> frames;
    double peak = 0.0;
    bool finite = true;
    for (int s = 0; s < 6000; ++s) {
        sim.tick();
        if (s % 100 == 0) frames.emplace_back(s / kTickRate, sim.positions());
        if (s / kTickRate < 2.0) continue;
        const auto& u = sim.positions();
        for (int i = 0; i < kRN; ++i) {
            if (!std::isfinite(u[i])) finite = false;
            peak = std::max(peak, std::abs(u[i]));
        }
    }
    save_frames(out, "cpp_random", frames);
    std::cout << "  late peak|u| " << peak
              << "   finite: " << (finite ? "yes" : "NO - BLEW UP") << "\n";
}

int main(int argc, char** argv) {
    fs::path out = (argc > 1) ? fs::path(argv[1]) : fs::path("..") / "prototype";
    fs::create_directories(out);
    std::cout << std::fixed;
    scenario_pluck(out);
    scenario_flagpole(out);
    scenario_travel(out);
    scenario_recover(out);
    scenario_cloth(out, false);
    scenario_cloth(out, true);
    scenario_aero(out);
    scenario_tilt(out);
    scenario_shed(out);
    scenario_wrinkle(out);
    scenario_mid(out);
    scenario_random(out);
    std::cout << "done.\n";
    return 0;
}
