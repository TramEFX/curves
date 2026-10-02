// Curves - audio-thread cost benchmark.
//
// The glitching report that motivated this: "lags, skips and artifacts in
// high precision with the wind running", at ~6.5% total CPU on a 16-thread
// machine - which is ~1.04 of ONE core, i.e. the audio thread saturated.
// This measures each stage's cost as a FRACTION OF REAL TIME so the
// offender is a number, not a guess.

#include "filter_bank.h"
#include "string_sim.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <functional>
#include <vector>

using namespace curves;
using clk = std::chrono::steady_clock;

static double seconds(const std::function<void()>& fn, int reps) {
    const auto t0 = clk::now();
    for (int i = 0; i < reps; ++i) fn();
    return std::chrono::duration<double>(clk::now() - t0).count();
}

int main(int argc, char** argv) {
    // Sample rate matters a lot: the sim ticks at fs/divisor and the bank
    // runs per sample, so BOTH scale linearly with fs. Pass it to check
    // headroom at 96 k before trusting a 48 k measurement.
    const double fs = (argc > 1) ? std::atof(argv[1]) : 48000.0;
    std::printf("sample rate: %.0f Hz\n", fs);
    std::printf("stage                              ns/call   calls/s     %% of one core\n");
    std::printf("-------------------------------------------------------------------------\n");

    for (int n : {128, 256, 512, 1024}) {
        const int divisor = (n == 128) ? 4 : 2;
        const double tick_rate = fs / divisor;

        // --- sim tick, wind OFF (static field) ---
        {
            StringParams p; p.tick_rate = tick_rate; p.wave_speed = 300.0;
            StringSim sim(n, p);
            const int reps = 20000;
            const double t = seconds([&] { sim.tick(); }, reps);
            const double ns = t / reps * 1e9;
            std::printf("sim tick  n=%-4d wind OFF          %8.0f  %8.0f   %8.2f\n",
                        n, ns, tick_rate, ns * 1e-9 * tick_rate * 100.0);
        }
        // --- sim tick, wind ON (cloth aero + gust field) ---
        {
            StringParams p; p.tick_rate = tick_rate; p.wave_speed = 300.0;
            StringSim sim(n, p);
            sim.set_cloth_aero(192.0 * n / 128.0, (double)n, 1.5);
            sim.wind_speed = 25.0; sim.aero_rho_cd = 0.6;
            sim.set_density_taper(2.0); sim.set_cloth_length(1.0);
            const int reps = 20000;
            const double t = seconds([&] { sim.tick(); }, reps);
            const double ns = t / reps * 1e9;
            std::printf("sim tick  n=%-4d wind ON           %8.0f  %8.0f   %8.2f\n",
                        n, ns, tick_rate, ns * 1e-9 * tick_rate * 100.0);
        }
        // --- sim tick, wind ON + flutter ---
        {
            StringParams p; p.tick_rate = tick_rate; p.wave_speed = 300.0;
            StringSim sim(n, p);
            sim.set_cloth_aero(192.0 * n / 128.0, (double)n, 1.5);
            sim.wind_speed = 25.0; sim.aero_rho_cd = 0.6; sim.shed_gain = 30.0;
            sim.set_density_taper(2.0); sim.set_cloth_length(1.0);
            const int reps = 20000;
            const double t = seconds([&] { sim.tick(); }, reps);
            const double ns = t / reps * 1e9;
            std::printf("sim tick  n=%-4d wind + flutter    %8.0f  %8.0f   %8.2f\n",
                        n, ns, tick_rate, ns * 1e-9 * tick_rate * 100.0);
        }
        // --- filter bank, per sample, 2 channels ---
        {
            FilterBank bank;
            bank.prepare(n, fs, 2);
            for (int b = 0; b < n; ++b)
                bank.set_band_frequency(b, 30.0 * std::pow(600.0, (double)b / (n - 1)));
            bank.set_smoothing(30.0);
            std::vector<double> g(n, 1.0);
            bank.set_targets(g.data());
            const int reps = 20000;
            float x = 0.1f;
            const double t = seconds([&] {
                x = bank.process_sample(0, x) * 0.5f + 0.05f;
                x = bank.process_sample(1, x) * 0.5f + 0.05f;
            }, reps);
            const double ns = t / reps * 1e9;
            std::printf("bank      n=%-4d 2ch/sample        %8.0f  %8.0f   %8.2f\n",
                        n, ns, fs, ns * 1e-9 * fs * 100.0);
            std::printf("\n");
        }
    }
    return 0;
}
