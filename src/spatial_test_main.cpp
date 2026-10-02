// Where does the string's SHAPE live, in spatial wavenumber?
//
// Reported: "there's big slow waves, and there's very small ripples when I
// turn wind up really high ... we need some mid-sized ripples and waves."
// Bins |FFT across nodes| of the string shape by waves-across-the-string, so
// a missing middle shows up as a hole between the gust band and wrinkle.

#include "string_sim.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace curves;

namespace {
constexpr double kPi = 3.14159265358979323846;
// bands in "waves across the active span"
struct Band { const char* name; double lo, hi; };
const Band kBands[] = {
    {"  1-2 ", 0.75, 2.5},   {"  2-6 ", 2.5, 6.0},
    {" 6-12 ", 6.0, 12.0},   {"12-25 ", 12.0, 25.0},
    {"25-64 ", 25.0, 64.0},
};
constexpr int kNB = 5;

void spectrum(const std::vector<double>& u, int n, double* out) {
    for (int b = 0; b < kNB; ++b) out[b] = 0.0;
    double mean = 0.0;
    for (int i = 0; i < n; ++i) mean += u[i];
    mean /= n;
    const int kmax = std::min(n / 2 - 1, 70);
    for (int k = 1; k <= kmax; ++k) {
        double re = 0.0, im = 0.0;
        for (int i = 0; i < n; ++i) {
            const double a = -2.0 * kPi * k * i / n;
            const double v = u[i] - mean;
            re += v * std::cos(a);
            im += v * std::sin(a);
        }
        const double mag = std::sqrt(re * re + im * im) / n;
        for (int b = 0; b < kNB; ++b)
            if (k >= kBands[b].lo && k < kBands[b].hi) out[b] += mag * mag;
    }
    for (int b = 0; b < kNB; ++b) out[b] = std::sqrt(out[b]);
}

void run(double wind, double waves, double wrinkle_amt, int prec,
         double motion, double* acc, double mid_amp = 0.0,
         double* peak_out = nullptr) {
    const int nodes[3] = {128, 256, 512};
    const int divs[3] = {4, 2, 2};
    const int n = nodes[prec];
    const double fs = 48000.0, tick = fs / divs[prec];
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
    const double node_norm = std::pow(128.0 / n, 0.25);
    const double gain[3] = {1.590, 1.031, 0.913};
    const double boost[3] = {0.969, 1.007, 0.968};
    const double target = 0.85 * motion * boost[prec];
    const double dial = std::clamp(std::pow(target / 0.8720, 1.0 / 1.679), 0.0, 2.0);
    sim.wind_speed = 25.0 * dial * node_norm * gain[prec];
    sim.wrinkle_speed = sim.wind_speed;
    sim.wave_density = 0.5 + 4.5 * waves;
    sim.wrinkle = 3.9 * wrinkle_amt;
    sim.wind_phase_speed = 0.12 * std::pow(25.0, wind) * 300.0;
    sim.mid_waves = 6.0 * std::pow(20.0 / 6.0, wind);
    sim.mid_ripple = mid_amp;

    const long long ticks = (long long)(16.0 * tick);
    const long long warm = ticks / 4;
    int frames = 0;
    double band[kNB];
    for (int b = 0; b < kNB; ++b) acc[b] = 0.0;
    const long long every = (long long)(tick / 8);
    double pk = 0.0;
    for (long long t = 0; t < ticks; ++t) {
        sim.tick();
        if (t >= warm) {
            const auto& uu = sim.positions();
            for (int i = 0; i < n; ++i) pk = std::max(pk, std::fabs(uu[i]));
        }
        if (t < warm || (t - warm) % every) continue;
        spectrum(sim.positions(), n, band);
        for (int b = 0; b < kNB; ++b) acc[b] += band[b];
        ++frames;
    }
    for (int b = 0; b < kNB; ++b) acc[b] /= std::max(1, frames);
    if (peak_out) *peak_out = pk;
}

void header(const char* title) {
    std::printf("\n%s\n", title);
    std::printf("%22s", "waves across string:");
    for (int b = 0; b < kNB; ++b) std::printf("%9s", kBands[b].name);
    std::printf("\n");
}
}  // namespace

constexpr double kChosen = 1.0;   // candidate mid_ripple amount

int main(int argc, char** argv) {
    const int prec = argc > 1 ? std::atoi(argv[1]) : 2;
    double a[kNB];
    std::printf("Spatial spectrum of the string shape (rms magnitude per band).\n");
    std::printf("%d nodes, Motion 0.45, Character 0.\n", argc > 1 ? (argv[1][0]=='0'?128:(argv[1][0]=='1'?256:512)) : 512);

    header("A. Wind sweep, Waves 0.4 (default), Wrinkle 0");
    for (double w : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        run(w, 0.4, 0.0, prec, 0.45, a);
        std::printf("      Wind %.2f        ", w);
        for (int b = 0; b < kNB; ++b) std::printf("%9.5f", a[b]);
        std::printf("\n");
    }

    header("B. Waves sweep, Wind 0.5 (default), Wrinkle 0");
    for (double v : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        run(0.5, v, 0.0, prec, 0.45, a);
        std::printf("     Waves %.2f (d=%.1f) ", v, 0.5 + 4.5 * v);
        for (int b = 0; b < kNB; ++b) std::printf("%9.5f", a[b]);
        std::printf("\n");
    }

    header("C. Wrinkle sweep, Wind 0.5, Waves 0.4");
    for (double r : {0.0, 0.5, 1.0}) {
        run(0.5, 0.4, r, prec, 0.45, a);
        std::printf("   Wrinkle %.2f        ", r);
        for (int b = 0; b < kNB; ++b) std::printf("%9.5f", a[b]);
        std::printf("\n");
    }

    std::printf("\nD. MID BAND amount -- spectrum AND excitation budget\n");
    std::printf("%22s", "waves across string:");
    for (int b = 0; b < kNB; ++b) std::printf("%9s", kBands[b].name);
    std::printf("%10s%10s\n", "peak|u|", "vs mid=0");
    {
        double base_pk = 0.0;
        for (double m : {0.0, 0.25, 0.5, 1.0, 1.5, 2.0}) {
            double pk = 0.0;
            run(0.5, 0.4, 0.0, prec, 0.45, a, m, &pk);
            if (m == 0.0) base_pk = pk;
            std::printf("  mid_amp %.2f         ", m);
            for (int b = 0; b < kNB; ++b) std::printf("%9.5f", a[b]);
            std::printf("%10.4f%9.2fx\n", pk, pk / base_pk);
        }
    }

    header("E. Wind sweep WITH the mid band");
    for (double w : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        double pk = 0.0;
        run(w, 0.4, 0.0, prec, 0.45, a, kChosen, &pk);
        std::printf("      Wind %.2f        ", w);
        for (int b = 0; b < kNB; ++b) std::printf("%9.5f", a[b]);
        std::printf("%10.4f\n", pk);
    }

    return 0;
}
