// Two questions about Waves.
//  A. Where do the big slow blooming curves come from, and why do they
//     "swing both directions seemingly randomly"?
//  B. What does the Waves knob actually move, and how far can its range go?

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

struct Band { const char* name; double lo, hi; };
const Band kBands[] = {{" 1-2 ",0.75,2.5},{" 2-6 ",2.5,6.0},{" 6-12",6.0,12.0},
                       {"12-25",12.0,25.0},{"25-64",25.0,64.0}};
constexpr int kNB = 5;

// signed projection of the shape onto the lowest spatial modes, so the sign
// tells us which way the big curve is leaning
double low_mode_signed(const std::vector<double>& u, int n) {
    double mean = 0.0;
    for (int i = 0; i < n; ++i) mean += u[i];
    mean /= n;
    double re = 0.0;
    for (int i = 0; i < n; ++i)
        re += (u[i] - mean) * std::cos(2.0 * kPi * 2.0 * i / n);
    return 2.0 * re / n;
}

void bands_of(const std::vector<double>& u, int n, double* out) {
    for (int b = 0; b < kNB; ++b) out[b] = 0.0;
    double mean = 0.0;
    for (int i = 0; i < n; ++i) mean += u[i];
    mean /= n;
    for (int k = 1; k <= 70; ++k) {
        double re = 0.0, im = 0.0;
        for (int i = 0; i < n; ++i) {
            const double a = -2.0 * kPi * k * i / n;
            const double v = u[i] - mean;
            re += v * std::cos(a); im += v * std::sin(a);
        }
        const double mag = std::sqrt(re*re + im*im) / n;
        for (int b = 0; b < kNB; ++b)
            if (k >= kBands[b].lo && k < kBands[b].hi) out[b] += mag*mag;
    }
    for (int b = 0; b < kNB; ++b) out[b] = std::sqrt(out[b]);
}

// mirrors wave_density_compensation() in PluginProcessor
double wave_comp(double density) {
    const double D[] = {0.5,1.0,2.3,3.5,5.0,7.0,9.5,13.0,18.0};
    const double R[] = {1.548,1.397,1.000,0.636,0.431,0.359,0.325,0.321,0.304};
    const double d = std::clamp(density, D[0], D[8]);
    double r = R[8];
    for (int i = 0; i < 8; ++i) {
        if (d > D[i+1]) continue;
        const double f = std::log(d/D[i]) / std::log(D[i+1]/D[i]);
        r = std::exp(std::log(R[i]) + f*std::log(R[i+1]/R[i]));
        break;
    }
    return std::clamp(std::pow(1.0/std::max(r,1e-9), 1.0/1.679), 0.7, 2.2);
}

StringSim* make(double density, bool compensate = false) {
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
    s->set_walls(0.0, 1.0);
    s->wind_speed = 25.0 * 0.596 * std::pow(128.0 / kN, 0.25) * 0.913;
    s->wrinkle_speed = s->wind_speed;
    s->wave_density = density;
    if (compensate) s->wind_speed *= wave_comp(density);
    s->wind_phase_speed = 0.6 * 300.0;
    s->mid_waves = 11.0; s->mid_ripple = 1.19;
    s->mid_lock_d = 0.65; s->mid_env = 1.4;
    s->mid_travel = 0.829; s->mid_flap = 0.45;
    return s;
}
}  // namespace

bool g_comp = false;

int main() {
    std::printf("A. THE BIG SLOW BLOOMS -- 150 s at the shipped defaults.\n");
    std::printf("   Signed amplitude of the lowest surviving spatial mode,\n");
    std::printf("   sampled every 2 s. Sign = which way the curve leans.\n\n");
    {
        StringSim* s = make(0.5 + 4.5 * 0.4);
        const long long ticks = (long long)(150.0 * kTick);
        const long long every = (long long)(2.0 * kTick);
        std::vector<double> trace;
        for (long long t = 0; t < ticks; ++t) {
            s->tick();
            if (t % every == 0 && t > kTick * 10)
                trace.push_back(low_mode_signed(s->positions(), kN));
        }
        for (size_t i = 0; i < trace.size(); ++i) {
            if (i % 10 == 0) std::printf("   t=%4.0fs ", 10.0 + i * 2.0);
            std::printf("%+7.3f", trace[i]);
            if (i % 10 == 9) std::printf("\n");
        }
        std::printf("\n");
        int flips = 0;
        for (size_t i = 1; i < trace.size(); ++i)
            if ((trace[i] > 0) != (trace[i-1] > 0)) ++flips;
        double mn = 1e9, mx = -1e9;
        for (double v : trace) { mn = std::min(mn,v); mx = std::max(mx,v); }
        std::printf("   sign flips in 140 s: %d  (mean period %.1f s)\n",
                    flips, flips ? 280.0 / flips : 0.0);
        std::printf("   swing: %+.3f .. %+.3f  -- it goes BOTH ways\n", mn, mx);
        delete s;
    }

    std::printf("\nB. WAVES: what it costs. 40 s per point, rms is the stable\n");
    std::printf("   measure; peak over a short window is too noisy to tune on.\n\n");
    std::printf("%9s%9s%9s%9s%9s%9s%10s%9s\n", "density", " 1-2", " 2-6",
                " 6-12", "12-25", "25-64", "rms |u|", "vs d=2.3");
    {
        double base_rms = 0.0;
        for (double d : {0.5, 1.0, 2.3, 3.5, 5.0, 7.0, 9.5, 13.0, 18.0}) {
            StringSim* s2 = make(d, g_comp);
            const long long ticks = (long long)(40.0 * kTick);
            const long long warm = ticks / 4, every = (long long)(kTick / 8);
            double acc[kNB] = {}, band[kNB], sq = 0.0;
            long long cnt = 0; int frames = 0;
            for (long long t = 0; t < ticks; ++t) {
                s2->tick();
                if (t < warm) continue;
                const auto& u = s2->positions();
                for (int i = 0; i < kN; ++i) { sq += u[i]*u[i]; ++cnt; }
                if ((t - warm) % every) continue;
                bands_of(u, kN, band);
                for (int b = 0; b < kNB; ++b) acc[b] += band[b];
                ++frames;
            }
            const double rms = std::sqrt(sq / cnt);
            if (d == 2.3) base_rms = rms;
            std::printf("%9.1f", d);
            for (int b = 0; b < kNB; ++b) std::printf("%9.5f", acc[b] / frames);
            std::printf("%10.5f%8.2fx\n", rms,
                        base_rms > 0 ? rms / base_rms : 0.0);
            delete s2;
        }
    }

    g_comp = true;
    std::printf("\nC. WITH the amplitude compensation applied.\n");
    std::printf("%9s%9s%9s%9s%9s%9s%10s%9s\n", "density", " 1-2", " 2-6",
                " 6-12", "12-25", "25-64", "rms |u|", "vs d=2.3");
    {
        double base_rms = 0.0;
        for (double d : {0.5, 1.0, 2.3, 3.5, 5.0, 7.0, 8.0}) {
            StringSim* s2 = make(d, true);
            const long long ticks = (long long)(40.0 * kTick);
            const long long warm = ticks / 4, every = (long long)(kTick / 8);
            double acc[kNB] = {}, band[kNB], sq = 0.0;
            long long cnt = 0; int frames = 0;
            for (long long t = 0; t < ticks; ++t) {
                s2->tick();
                if (t < warm) continue;
                const auto& u = s2->positions();
                for (int i = 0; i < kN; ++i) { sq += u[i]*u[i]; ++cnt; }
                if ((t - warm) % every) continue;
                bands_of(u, kN, band);
                for (int b = 0; b < kNB; ++b) acc[b] += band[b];
                ++frames;
            }
            const double rms = std::sqrt(sq / cnt);
            if (d == 2.3) base_rms = rms;
            std::printf("%9.1f", d);
            for (int b = 0; b < kNB; ++b) std::printf("%9.5f", acc[b]/frames);
            std::printf("%10.5f%8.2fx\n", rms, base_rms>0 ? rms/base_rms : 0.0);
            delete s2;
        }
    }

    return 0;
}
