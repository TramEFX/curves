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

}  // namespace

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
    std::cout << "done.\n";
    return 0;
}
