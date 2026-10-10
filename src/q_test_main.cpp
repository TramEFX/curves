// Per-node Q: does matching bandwidth to spacing fix the overlap (B1), and
// does it buy the ringing texture?
//
// Two things are measured DIRECTLY from the running bank rather than
// inferred, because the overlap finding came from an rms ratio that cannot
// distinguish "deeper than asked" from "rippled":
//
//   1. the steady-state magnitude response at band CENTRES and at the
//      geometric MIDPOINTS between them -> the overlap factor K and the
//      comb ripple that replaces it when Q goes past matched;
//   2. the impulse-response decay -> the ring time, which is the whole
//      reason to want high Q at all.
//
// Reference: K ~= 0.152*N at Q = 1 (overlap_probe). Matched Q is the Q whose
// -3 dB bandwidth equals the band spacing.

#include "filter_bank.h"
#include <cmath>
#include <cstdio>
#include <ctime>
#include <string>
#include <vector>

using curves::FilterBank;

constexpr double kPi = 3.14159265358979323846;
constexpr double kFs = 48000.0;
constexpr double kSpanOct = 9.228818690495881;  // log2(600)

static double node_f(int i, int n) { return 30.0 * std::pow(600.0, double(i) / (n - 1)); }
static double db2lin(double db) { return std::pow(10.0, db / 20.0); }

// Q whose -3 dB bandwidth is exactly BW octaves: Q = sqrt(2^BW)/(2^BW - 1).
static double q_for_bw(double bw_oct) {
    const double t = std::pow(2.0, bw_oct);
    return std::sqrt(t) / (t - 1.0);
}
static double matched_q(int n) { return q_for_bw(kSpanOct / (n - 1)); }

struct Bank {
    FilterBank b;
    int n;
    std::vector<double> g;
    Bank(int nb, double q, double nominal_db, bool correct = false)
        : n(nb), g(nb, db2lin(nominal_db)) {
        b.prepare(nb, kFs, 2);
        for (int i = 0; i < nb; ++i) b.set_band_frequency(i, std::min(node_f(i, nb), 0.45 * kFs));
        b.set_q(q);
        b.set_overlap_correction(correct);
        b.set_steady(0.0, 0.25);
        b.set_spatial_off();
    }
    void run(int frames, const double* in, double* out) {
        for (int i = 0; i < frames; ++i) {
            if ((i % 32) == 0) { b.set_block_targets(g.data(), 32); b.set_spatial_off(); }
            float l = float(in[i]), r = l;
            b.process_frame(l, r, true);
            out[i] = l;
        }
    }
};

// Steady-state gain at f, by synchronous detection over whole cycles.
static double gain_at(int nb, double q, double nominal_db, double f, bool correct = false) {
    const double tau = q / (kPi * f);                       // bandpass decay
    const int settle = int(std::max(12.0 * tau, 0.25) * kFs);
    const double cyc = kFs / f;
    const int meas = int(std::floor(std::max(40.0, 0.25 * kFs) / cyc) * cyc);
    const int total = settle + meas;
    std::vector<double> in(total), out(total);
    const double w = 2.0 * kPi * f / kFs;
    for (int i = 0; i < total; ++i) in[i] = 0.1 * std::sin(w * i);
    Bank bk(nb, q, nominal_db, correct);
    bk.run(total, in.data(), out.data());
    double ss = 0.0, cc = 0.0;
    for (int i = settle; i < total; ++i) {
        ss += out[i] * std::sin(w * i);
        cc += out[i] * std::cos(w * i);
    }
    const double amp = 2.0 * std::sqrt(ss * ss + cc * cc) / meas;
    // SIGNED: an inverted cut has the same MAGNITUDE as a correct one, so
    // magnitude alone reported "ok" for a polarity flip. The in-phase part
    // carries the sign.
    return (ss < 0.0 ? -amp : amp) / 0.1;
}

// T60 of the added resonance: impulse in, envelope of (out - dry) out.
static double t60(int nb, double q, double nominal_db, int band) {
    const int total = int(std::max(1.0, 14.0 * q / (kPi * node_f(band, nb))) * kFs);
    std::vector<double> in(total, 0.0), out(total);
    in[0] = 1.0;
    Bank bk(nb, q, 0.0);             // OFF IS UNITY GAIN, not -inf dB:
    bk.g.assign(nb, 1.0);            // g = 1e-10 is a FULL NOTCH on every
    bk.g[band] = db2lin(nominal_db); // band, which is what I measured first
    bk.run(total, in.data(), out.data());
    double peak = 0.0;
    for (int i = 1; i < total; ++i) peak = std::max(peak, std::abs(out[i]));
    const double floor_a = peak * db2lin(-60.0);
    for (int i = total - 1; i > 0; --i)
        if (std::abs(out[i]) > floor_a) return double(i) / kFs;
    return 0.0;
}

// Nominal -> pre-corrected target: 1 + (g-1)/K, so that the bank's own
// 1 + K*(g'-1) lands back on g. K at MATCHED Q is a constant (~1.70),
// unlike K at Q=1 which scales with N -- that is the whole point.
static double gain_at_corrected(int nb, double q, double nominal_db, double f, double K) {
    const double g = db2lin(nominal_db);
    const double gp = 1.0 + (g - 1.0) / K;
    return gain_at(nb, q, 20.0 * std::log10(std::max(gp, 1e-9)), f);
}

// Spectral RESOLUTION: how much of a +-6 dB pattern alternating every P
// bands actually survives. This is the cost of Q = 1 that the ripple figure
// hides -- broad bands average neighbouring nodes together, so the string's
// detail is smeared whatever the depth correction does. 12 dB = perfect.
static double mtf_db(int nb, double q, int period, bool correct) {
    // Probe the CENTRE of each block, not its edge. Probing the first band
    // of a block put both probes hard against the opposite block and made
    // coarse patterns look worse than fine ones.
    const int s0 = (nb / 2) - ((nb / 2) % period);
    const int band_up = s0 + period / 4;
    const int band_dn = s0 + period / 2 + period / 4;
    Bank bk(nb, q, 0.0, correct);
    for (int i = 0; i < nb; ++i)
        bk.g[i] = db2lin(((i / (period / 2)) % 2 == 0) ? 6.0 : -6.0);
    const double f_up = node_f(band_up, nb), f_dn = node_f(band_dn, nb);
    const double tau = q / (kPi * std::min(f_up, f_dn));
    const int settle = int(std::max(12.0 * tau, 0.25) * kFs);
    double res[2];
    for (int w = 0; w < 2; ++w) {
        const double f = w ? f_dn : f_up;
        const double om = 2.0 * kPi * f / kFs;
        const double cyc = kFs / f;
        const int meas = int(std::floor(std::max(40.0, 0.25 * kFs) / cyc) * cyc);
        const int total = settle + meas;
        std::vector<double> in(total), out(total);
        for (int i = 0; i < total; ++i) in[i] = 0.1 * std::sin(om * i);
        Bank b2(nb, q, 0.0, correct);
        b2.g = bk.g;
        b2.run(total, in.data(), out.data());
        double ss = 0.0, cc = 0.0;
        for (int i = settle; i < total; ++i) {
            ss += out[i] * std::sin(om * i);
            cc += out[i] * std::cos(om * i);
        }
        res[w] = 20.0 * std::log10(2.0 * std::sqrt(ss * ss + cc * cc) / meas / 0.1);
    }
    return res[0] - res[1];
}

// LEVEL SAFETY. Ringing bands whose gain is being modulated by the string is
// the configuration most likely to blow up: energy stored in a high-Q band
// is still there when the gain moves. Driven with a sweeping +-18 dB shape
// (Range default) on noise, this reports peak, rms and any non-finite
// output. Flat Q is included because its UNCORRECTED depth is ~24x nominal,
// so it is not the safe baseline it looks like.
struct Levels { double peak, rms; bool finite; };
static Levels level_sweep(int nb, double q, bool correct, double depth_db, double secs) {
    Bank bk(nb, q, 0.0, correct);
    const int total = int(secs * kFs);
    unsigned int rs = 2246822519u;
    double peak = 0.0, sum = 0.0;
    bool finite = true;
    for (int i = 0; i < total; ++i) {
        if ((i % 32) == 0) {
            const double t = double(i) / kFs;
            for (int b = 0; b < nb; ++b)
                bk.g[b] = db2lin(depth_db * std::sin(2.0 * kPi * (3.0 * b / nb + 0.5 * t)));
            bk.b.set_block_targets(bk.g.data(), 32);
            bk.b.set_spatial_off();
        }
        rs = rs * 1664525u + 1013904223u;
        const float x = float((int(rs >> 9) & 0x3FFFFF) / 2097151.0 - 1.0) * 0.2f;
        float l = x, r = x;
        bk.b.process_frame(l, r, true);
        if (!std::isfinite(l)) { finite = false; break; }
        if (i > int(0.5 * kFs)) { peak = std::max(peak, std::abs(double(l))); sum += double(l) * l; }
    }
    return {peak, std::sqrt(sum / std::max(1, total - int(0.5 * kFs))), finite};
}

// WYSIWYG with Q on. The display draws gains(), the bank's nominal per-band
// value. The invariant says that IS what the audio goes through. With the
// bands adding, it is not: the overlap sits between the drawn number and the
// real response, so this measures the gap directly on a SMOOTH shape -- the
// kind the string actually makes -- rather than asserting it.
//
// The residual error after correction is ALL IN THE CUTS, and not because
// boosted neighbours leak in -- at the probe bands below, every neighbour is
// also a cut. It is the QUADRATURE term: out = x*(1 + A + jB), and the
// correction only makes A right. On a sloped shape the neighbours' imaginary
// parts no longer cancel (the band above and the band below carry different
// gains), so B is non-zero, and |1 + A + jB| is dominated by B exactly where
// 1 + A is small -- i.e. in a deep cut. This is the standard limit of a
// PARALLEL additive bank: it cannot notch deeply on a slope. Raising Q
// shrinks the neighbour weights and so shrinks B.
static bool g_verbose = false;
// argv[2] == "raw" turns the zero-mean normalisation off, to separate the
// bank's own bias from the shape's.
static bool g_zero_mean = true;
static double wysiwyg_err_db(int nb, double q, bool correct, double depth_db) {
    std::vector<double> shape(nb);
    for (int b = 0; b < nb; ++b)
        shape[b] = db2lin(depth_db * std::sin(2.0 * kPi * 2.0 * b / nb));
    double worst = 0.0;
    for (double frac : {0.3, 0.45, 0.6, 0.7}) {
        const int band = int(frac * (nb - 1));
        const double f = node_f(band, nb);
        const double tau = q / (kPi * f);
        const int settle = int(std::max(14.0 * tau, 0.3) * kFs);
        const double om = 2.0 * kPi * f / kFs;
        const double cyc = kFs / f;
        const int meas = int(std::floor(std::max(40.0, 0.25 * kFs) / cyc) * cyc);
        const int total = settle + meas;
        std::vector<double> in(total), out(total);
        for (int i = 0; i < total; ++i) in[i] = 0.1 * std::sin(om * i);
        Bank bk(nb, q, 0.0, correct);
        bk.g = shape;
        bk.run(total, in.data(), out.data());
        double ss = 0.0, cc = 0.0;
        for (int i = settle; i < total; ++i) {
            ss += out[i] * std::sin(om * i);
            cc += out[i] * std::cos(om * i);
        }
        const double actual = 20.0 * std::log10(2.0 * std::sqrt(ss * ss + cc * cc) / meas / 0.1);
        // what the canvas would draw for this band
        const double drawn = 20.0 * std::log10(bk.b.gains()[band]);
        if (g_verbose)
            std::printf("        band %4d  %7.0fHz  drawn %7.2fdB  actual %7.2fdB  err %6.2fdB\n",
                        band, f, drawn, actual, actual - drawn);
        worst = std::max(worst, std::abs(actual - drawn));
    }
    return worst;
}

// O10 acceptance: does the COMPUTED response curve equal the MEASURED one?
// Same smooth shape and probe bands as section I, but the "drawn" value is
// now FilterBank::response_curve instead of the nominal per-band gain.
static void o10_check(int nb, double q, bool correct, double depth_db,
                      double* worst_db, double* nominal_worst_db, int* window) {
    std::vector<double> shape(nb), big(nb), re(nb), im(nb);
    for (int b = 0; b < nb; ++b)
        shape[b] = db2lin(depth_db * std::sin(2.0 * kPi * 2.0 * b / nb));
    *worst_db = 0.0;
    *nominal_worst_db = 0.0;
    for (double frac : {0.3, 0.45, 0.6, 0.7}) {
        const int band = int(frac * (nb - 1));
        const double f = node_f(band, nb);
        const double tau = q / (kPi * f);
        const int settle = int(std::max(14.0 * tau, 0.3) * kFs);
        const double om = 2.0 * kPi * f / kFs;
        const double cyc = kFs / f;
        const int meas = int(std::floor(std::max(40.0, 0.25 * kFs) / cyc) * cyc);
        const int total = settle + meas;
        std::vector<double> in(total), out(total);
        for (int i = 0; i < total; ++i) in[i] = 0.1 * std::sin(om * i);
        Bank bk(nb, q, 0.0, correct);
        bk.g = shape;
        bk.run(total, in.data(), out.data());
        double ss = 0.0, cc = 0.0;
        for (int i = settle; i < total; ++i) {
            ss += out[i] * std::sin(om * i);
            cc += out[i] * std::cos(om * i);
        }
        const double actual = 20.0 * std::log10(2.0 * std::sqrt(ss * ss + cc * cc) / meas / 0.1);

        const auto& g = bk.b.gains();
        for (int b = 0; b < nb; ++b) big[b] = g[b] - 1.0;
        bk.b.response_curve(big.data(), 1.0, re.data(), im.data());
        const double computed =
            10.0 * std::log10((1.0 + re[band]) * (1.0 + re[band]) + im[band] * im[band]);
        const double nominal = 20.0 * std::log10(g[band]);
        *worst_db = std::max(*worst_db, std::abs(actual - computed));
        *nominal_worst_db = std::max(*nominal_worst_db, std::abs(actual - nominal));
        *window = bk.b.response_window();
    }
}

// Steady's detector against Band Q. The threshold is a fixed absolute level
// on the bandpass, and the SVF's v1 peaks at Q -- so without normalising,
// raising Q raises what the detector sees by the same factor and Steady
// slams shut. Probed with a sine ON a band centre, where the normalised
// bandpass has unity gain at any Q, so a correct detector reads the same
// duck at every Q.
static double steady_duck_db(int nb, double q, bool correct, double amp) {
    const int band = (nb - 1) / 2;
    const double f = node_f(band, nb);
    const double om = 2.0 * kPi * f / kFs;
    const int total = int(std::max(1.0, 14.0 * q / (kPi * f)) * kFs) + int(0.6 * kFs);
    Bank bk(nb, q, 0.0, correct);
    bk.g.assign(nb, db2lin(12.0));
    bk.b.set_steady(0.6, 0.25);
    for (int i = 0; i < total; ++i) {
        if ((i % 32) == 0) { bk.b.set_block_targets(bk.g.data(), 32); bk.b.set_spatial_off(); }
        float l = float(amp * std::sin(om * i)), r = l;
        bk.b.process_frame(l, r, true);
    }
    // 12 dB asked; how much is left after the duck?
    return 20.0 * std::log10(std::max(bk.b.gains()[band], 1e-9));
}

// Why the drawn curve suddenly has sharp dips in it.
//
// out = x*(1 + S) where S is the summed band response. With Band Q = Flat
// the bands add with K ~= 17 (128 nodes) to 70 (512), so for any band the
// string pulls below unity, 1 + K*(g-1) goes NEGATIVE -- it passes THROUGH
// ZERO on the way. The old curve drew the nominal per-band gain and never
// showed that. The honest curve draws |1 + S|, which is a deep narrow null
// exactly at the crossing, and the crossings sweep around as the string
// moves. That is the "jerky dips at various spots".
static void null_probe(int nb, double q, bool correct, double depth_db) {
    Bank bk(nb, q, 0.0, correct);
    std::vector<double> big(nb), re(nb), im(nb);
    // A smooth, realistic string shape -- nothing sharp in it at all.
    for (int i = 0; i < nb; ++i) {
        // TWO whole cycles. 1.5 cycles is not zero-mean in dB on its own
        // (the mean of sin over 1.5 periods is 2/3pi, so a +-18 dB shape
        // carries a +3.8 dB offset before the bank touches it) -- measuring
        // the bank's bias against that would be measuring the test.
        const double d = std::sin(2.0 * kPi * 2.0 * i / nb);
        bk.g[i] = db2lin(depth_db * d);
        big[i] = bk.g[i] - 1.0;
    }
    // Mirror the processor: zero-mean the gains before they reach the bank.
    if (g_zero_mean) {
        double m = 0.0;
        for (int k = 0; k < nb; ++k) m += bk.g[k];
        m /= nb;
        (void)m;
        // mirrors the processor: PRE-DISTORT, then floor at -30 dB
        std::vector<double> pin(nb), pout(nb);
        for (int k = 0; k < nb; ++k) pin[k] = bk.g[k] - 1.0;
        bk.b.predistort(pin.data(), pout.data());
        // mirrors the processor: blended by Band Q, which here is how far
        // Q has travelled from 1 toward matched, on a log scale.
        const double qm = matched_q(nb);
        const double pd = (qm > 1.0)
                              ? std::clamp(std::log(q / 1.0) / std::log(qm),
                                           0.0, 1.0)
                              : 1.0;
        for (int k = 0; k < nb; ++k) {
            const double b = pin[k] + pd * (pout[k] - pin[k]);
            // mirrors the processor: floor six dB below the request
            const double fl = std::max(1e-6,
                                       std::pow(10.0, -(depth_db + 6.0) / 20.0));
            bk.g[k] = std::max(1.0 + b, fl);
            big[k] = bk.g[k] - 1.0;
        }
    }
    bk.b.response_curve(big.data(), 1.0, re.data(), im.data());
    int crossings = 0;
    double worst = 1e9, nominal_worst = 1e9, best = -1e9, mean_db = 0.0;
    double prev = 1.0 + re[0];
    for (int i = 0; i < nb; ++i) {
        const double r = 1.0 + re[i];
        if (i > 0 && ((prev < 0.0) != (r < 0.0))) ++crossings;
        prev = r;
        const double mag = std::sqrt(r * r + im[i] * im[i]);
        worst = std::min(worst, 20.0 * std::log10(std::max(mag, 1e-12)));
        best = std::max(best, 20.0 * std::log10(std::max(mag, 1e-12)));
        mean_db += 20.0 * std::log10(std::max(mag, 1e-12));
        nominal_worst = std::min(nominal_worst, 20.0 * std::log10(bk.g[i]));
    }
    std::printf("%6d %10.2f %9s %12d %13.1fdB %13.1fdB %12.1fdB %11.2fdB\n",
                nb, q, correct ? "yes" : "no", crossings, nominal_worst,
                worst, best, mean_db / nb);
}

int main(int argc, char** argv) {

    if (argc > 1 && std::string(argv[1]) == "nulls") {
        g_zero_mean = !(argc > 2 && std::string(argv[2]) == "raw");
        std::printf("zero-mean normalisation: %s\n", g_zero_mean ? "ON" : "OFF");
        std::printf("=== L. WHERE THE SHARP DIPS COME FROM ===\n");
        std::printf("A SMOOTH +-18 dB string shape, nothing sharp in it.\n");
        std::printf("\"sign flips\" = times 1+Re crosses zero across the band map;\n");
        std::printf("each one is a magnitude NULL in the drawn curve.\n\n");
std::printf("%6s %10s %9s %12s %13s %13s %13s %12s\n", "bands", "Q",
                    "1/K corr", "sign flips", "nominal min", "drawn min",
                    "drawn max", "MEAN");
        for (int nb : {128, 512}) {
            const double qm = matched_q(nb);
            // Band Q now corrects FULLY at every setting, so every point
            // on the knob must be null-free, not just the top.
            null_probe(nb, 1.0, true, 18.0);
            null_probe(nb, std::pow(qm, 0.25), true, 18.0);
            null_probe(nb, std::pow(qm, 0.50), true, 18.0);
            null_probe(nb, qm, true, 18.0);
            null_probe(nb, qm * 4.0, true, 18.0);
            null_probe(nb, qm * 8.0, true, 18.0);
            // and at the depth the owner is actually running
            null_probe(nb, qm, true, 75.0);
            null_probe(nb, qm * 4.0, true, 75.0);
        }
        std::printf("\nUSABLE DEPTH: how far can the EQ be pushed before the\n");
        std::printf("quadrature leakage stops it being an EQ at all?\n\n");
        std::printf("%6s %10s %10s %13s %13s %12s\n", "bands", "Q",
                    "ask +-dB", "drawn min", "drawn max", "MEAN");
        {
            const int nb = 128;
            const double qm = matched_q(nb);
            for (double dep : {12.0, 18.0, 24.0, 30.0, 40.0, 54.0})
                null_probe(nb, qm * 4.0, true, dep);
        }
        
        // Why the worst dip sits at the TOP EDGE of the plot.
        // K is the sum of the neighbouring bands' responses, and the
        // outermost bands have no neighbours beyond the map -- so K falls
        // off at both ends. With the correction OFF, the response is
        // 1 + K(f)*big(f): as K sweeps down toward the edge it drags the
        // product through ZERO, and a zero is a bottomless null. That is
        // the cliff, and it is real, not a kernel artifact.
        std::printf("\nK across the band map (512 nodes, Flat Q = 1):\n");
        std::printf("%10s %10s %12s %14s %16s\n", "band", "freq",
                    "K there", "cut g=0.5 ->", "nulls at a cut of");
        {
            const int nb = 512;
            FilterBank probe;
            probe.prepare(nb, kFs, 2);
            for (int i = 0; i < nb; ++i)
                probe.set_band_frequency(i, std::min(node_f(i, nb), 0.45 * kFs));
            probe.set_q(1.0);
            for (int i : {0, 8, 32, 128, 256, 384, 480, 504, 511}) {
                const double f = node_f(i, nb);
                const double K = probe.overlap_factor(f);
                const double resp = 1.0 + K * (0.5 - 1.0);
                // A complete null needs 1 + K*big = 0, i.e. big = -1/K.
                // At K = 96 that is a nominal cut of NINE HUNDREDTHS of a dB.
                const double null_db =
                    20.0 * std::log10(std::max(1.0 - 1.0 / K, 1e-9));
                std::printf("%10d %9.0fHz %12.2f %10.2f %s %12.3f dB\n",
                            i, f, K, resp,
                            resp < 0.0 ? "(inverted)" : "          ", null_db);
            }
        }
        return 0;
    }
    std::printf("Matched Q (bandwidth == spacing):  128 -> %.2f   256 -> %.2f   512 -> %.2f\n",
                matched_q(128), matched_q(256), matched_q(512));

    std::printf("\n=== A. OVERLAP FACTOR K at band centres, nominal +6 dB ===\n");
    std::printf("K = (actual-1)/(G-1).  K=1 means the knob is honest.\n\n");
    std::printf("%6s %10s %9s %9s %9s %9s\n", "bands", "Q", "f=99Hz", "f=1kHz", "f=9.8kHz", "mean K");
    for (int nb : {128, 512}) {
        const double qm = matched_q(nb);
        for (double q : {1.0, qm * 0.5, qm, qm * 2.0, qm * 8.0}) {
            double ks[3];
            int bi = 0;
            for (double frac : {0.25, 0.5, 0.75}) {
                const int band = int(frac * (nb - 1) + 0.5);
                const double a = gain_at(nb, q, 6.0, node_f(band, nb));
                ks[bi++] = (a - 1.0) / (db2lin(6.0) - 1.0);
            }
            std::printf("%6d %10.2f %9.2f %9.2f %9.2f %9.2f\n", nb, q, ks[0], ks[1], ks[2],
                        (ks[0] + ks[1] + ks[2]) / 3.0);
        }
    }

    std::printf("\n=== B. RIPPLE: centre vs midpoint between centres, nominal +6 dB ===\n");
    std::printf("A comb appears once Q goes past matched: the gaps stop being filled.\n\n");
    std::printf("%6s %10s %11s %11s %10s\n", "bands", "Q", "centre dB", "midpt dB", "ripple");
    for (int nb : {128, 512}) {
        const double qm = matched_q(nb);
        for (double q : {1.0, qm, qm * 2.0, qm * 8.0}) {
            const int band = (nb - 1) / 2;
            const double fc = node_f(band, nb);
            const double fm = std::sqrt(fc * node_f(band + 1, nb));
            const double dc = 20.0 * std::log10(gain_at(nb, q, 6.0, fc));
            const double dm = 20.0 * std::log10(gain_at(nb, q, 6.0, fm));
            std::printf("%6d %10.2f %10.2fdB %10.2fdB %9.2fdB\n", nb, q, dc, dm, dc - dm);
        }
    }

    std::printf("\n=== C. CUTS: does a cut still invert? nominal -12 dB, 128 bands ===\n\n");
    std::printf("%10s %11s %11s\n", "Q", "actual dB", "polarity");
    {
        const double qm = matched_q(128);
        for (double q : {1.0, qm, qm * 4.0}) {
            const int band = (128 - 1) / 2;
            const double a = gain_at(128, q, -12.0, node_f(band, 128));
            std::printf("%10.2f %10.2fdB %11s\n", q,
                        20.0 * std::log10(std::abs(a)),
                        a < 0.0 ? "INVERTED" : "ok");
        }
    }

    std::printf("\n=== D. RING TIME (T60 of the added resonance), +6 dB on ONE band ===\n");
    std::printf("Constant Q means constant ring in CYCLES, so time falls as 1/f.\n\n");
    std::printf("%6s %10s %9s %11s %11s\n", "bands", "Q", "f", "T60 meas", "6.91Q/pi.f");
    for (int nb : {128}) {
        const double qm = matched_q(nb);
        for (double q : {1.0, qm, qm * 4.0, qm * 16.0})
            for (double frac : {0.1, 0.5, 0.9}) {
                const int band = int(frac * (nb - 1) + 0.5);
                const double f = node_f(band, nb);
                std::printf("%6d %10.2f %8.0fHz %9.0fms %9.0fms\n", nb, q, f,
                            1000.0 * t60(nb, q, 6.0, band), 1000.0 * 6.907 * q / (kPi * f));
            }
    }
    std::printf("\nn=== E. MATCHED Q + ONE SCALAR 1/K: is the knob honest now? ===\n");
    std::printf("K at matched Q is ~1.70 and does NOT scale with N, so a single\n");
    std::printf("constant corrects it. Target = nominal; both columns should agree.\n\n");
    std::printf("%6s %10s %11s %11s %10s\n", "bands", "nominal", "uncorrected", "corrected", "polarity");
    {
        const double K = 1.70;
        for (int nb : {128, 512}) {
            const double qm = matched_q(nb);
            const int band = (nb - 1) / 2;
            const double f = node_f(band, nb);
            for (double d : {12.0, 6.0, 3.0, -3.0, -6.0, -12.0}) {
                const double u = gain_at(nb, qm, d, f);
                const double c = gain_at_corrected(nb, qm, d, f, K);
                std::printf("%6d %9.1fdB %10.2fdB %10.2fdB %10s\n", nb, d,
                            20.0 * std::log10(std::abs(u)),
                            20.0 * std::log10(std::abs(c)),
                            c < 0.0 ? "INVERTED" : "ok");
            }
        }
    }
    std::printf("\n=== F. ANALYTIC K vs MEASURED, and the bank correcting itself ===\n");
    std::printf("K is computed from the band map, not fitted. The last column is\n");
    std::printf("the bank asked for a nominal dB with its own correction ON.\n\n");
    std::printf("%6s %10s %11s %11s %13s\n",
                "bands", "Q", "K analytic", "K measured", "+6dB corrected");
    for (int nb : {128, 512}) {
        const double qm = matched_q(nb);
        const int band = (nb - 1) / 2;
        const double f = node_f(band, nb);
        for (double q : {1.0, qm, qm * 2.0, qm * 8.0}) {
            FilterBank probe;
            probe.prepare(nb, kFs, 2);
            for (int i = 0; i < nb; ++i)
                probe.set_band_frequency(i, std::min(node_f(i, nb), 0.45 * kFs));
            probe.set_q(q);
            probe.set_overlap_correction(true);
            const double meas = (gain_at(nb, q, 6.0, f) - 1.0) / (db2lin(6.0) - 1.0);
            const double corr = gain_at(nb, q, 6.0, f, true);
            std::printf("%6d %10.2f %11.2f %11.2f %10.2fdB\n", nb, q,
                        probe.overlap_k(), meas, 20.0 * std::log10(std::abs(corr)));
        }
    }
    std::printf("\n=== G. SPECTRAL RESOLUTION: survival of a +-6 dB pattern (12 dB = all) ===\n");
    std::printf("The real cost of Q=1 is not depth -- the 1/K scalar fixes depth. It is\n");
    std::printf("that 1.4-octave bands AVERAGE adjacent nodes, smearing the string.\n\n");
    std::printf("%6s %10s %12s %12s %12s %12s\n",
                "bands", "Q", "every 2", "every 4", "every 8", "every 16");
    for (int nb : {128, 512}) {
        const double qm = matched_q(nb);
        for (double q : {1.0, qm, qm * 2.0}) {
            std::printf("%6d %10.2f", nb, q);
            for (int per : {2, 4, 8, 16})
                std::printf(" %11.2fdB", mtf_db(nb, q, per, true));
            std::printf("\n");
        }
    }
    std::printf("\n=== H. LEVEL SAFETY: sweeping +-18 dB shape on noise (in rms 0.115) ===\n");
    std::printf("Flat Q is NOT the safe baseline: uncorrected, its actual depth is ~24x\n");
    std::printf("nominal, so it is already the loudest configuration here.\n\n");
    std::printf("%6s %22s %10s %10s %9s %9s\n",
                "bands", "mode", "Q", "peak", "rms", "finite");
    for (int nb : {128, 512}) {
        const double qm = matched_q(nb);
        struct { const char* name; double q; bool corr; } modes[] = {
            {"Flat Q=1 (today)", 1.0, false},
            {"Flat Q=1 + 1/K", 1.0, true},
            {"Matched (Res 0)", qm, true},
            {"Matched x2 (Res .33)", qm * 2.0, true},
            {"Matched x4 (Res .67)", qm * 4.0, true},
            {"Matched x8 (Res 1)", qm * 8.0, true},
        };
        for (auto& m : modes) {
            const Levels L = level_sweep(nb, m.q, m.corr, 18.0, 6.0);
            std::printf("%6d %22s %10.2f %10.3f %9.4f %9s\n", nb, m.name, m.q,
                        L.peak, L.rms, L.finite ? "yes" : "NO -- NaN");
        }
    }
    std::printf("\n=== I. WYSIWYG: drawn curve vs MEASURED response, smooth +-12 dB shape ===\n");
    std::printf("Invariant 1 says the bright curve is what the audio goes through.\n");
    std::printf("Worst error over four probe bands, in dB. 0 = the curve is honest.\n\n");
    std::printf("%6s %24s %10s %14s\n", "bands", "mode", "Q", "worst error");
    for (int nb : {128, 512}) {
        const double qm = matched_q(nb);
        struct { const char* name; double q; bool corr; } modes[] = {
            {"Flat Q=1 (today)", 1.0, false},
            {"Flat Q=1 + 1/K", 1.0, true},
            {"Matched (Res 0)", qm, true},
            {"Matched x2 (Res .33)", qm * 2.0, true},
            {"Matched x4 (Res .67)", qm * 4.0, true},
            {"Matched x8 (Res 1)", qm * 8.0, true},
        };
        g_verbose = false;
        for (auto& m : modes)
            std::printf("%6d %24s %10.2f %11.2fdB\n", nb, m.name, m.q,
                        wysiwyg_err_db(nb, m.q, m.corr, 12.0));
    }
    std::printf("\n=== J. O10 -- COMPUTED response curve vs MEASURED (and vs the old curve) ===\n");
    std::printf("Same shape and probe bands as section I. \"nominal\" is what the curve\n");
    std::printf("drew before; \"computed\" is FilterBank::response_curve.\n\n");
    std::printf("%6s %22s %8s %14s %14s\n",
                "bands", "mode", "window", "OLD curve err", "NEW curve err");
    for (int nb : {128, 512}) {
        const double qm = matched_q(nb);
        struct { const char* name; double q; bool corr; } modes[] = {
            {"Flat Q=1 (today)", 1.0, false},
            {"Flat Q=1 + 1/K", 1.0, true},
            {"Matched (Res 0)", qm, true},
            {"Matched x2 (Res .33)", qm * 2.0, true},
            {"Matched x8 (Res 1)", qm * 8.0, true},
        };
        for (auto& m : modes) {
            double w = 0.0, nw = 0.0; int win = 0;
            o10_check(nb, m.q, m.corr, 12.0, &w, &nw, &win);
            std::printf("%6d %22s %8d %11.2fdB %11.2fdB\n", nb, m.name, win, nw, w);
        }
    }
    std::printf("\nCost of one response_curve call (publish runs at 60 Hz):\n");
    for (int nb : {128, 512}) {
        const double qm = matched_q(nb);
        for (double q : {1.0, qm}) {
            Bank bk(nb, q, 0.0, true);
            std::vector<double> big(nb, 0.3), re(nb), im(nb);
            const int reps = 2000;
            const clock_t t0 = clock();
            for (int r = 0; r < reps; ++r)
                bk.b.response_curve(big.data(), 1.0, re.data(), im.data());
            const double us = 1e6 * double(clock() - t0) / CLOCKS_PER_SEC / reps;
            std::printf("  %4d bands  Q %8.2f  window %4d  %8.1f us/call  %5.2f%% at 60 Hz\n",
                        nb, q, bk.b.response_window(), us, us * 60.0 * 1e-4);
        }
    }
    std::printf("\n=== K. STEADY vs Band Q: the detector must not be Q-sensitive ===\n");
    std::printf("Sine on a band centre, 12 dB asked, Steady 0.6. The normalised\n");
    std::printf("bandpass has unity gain at any Q, so these columns should AGREE.\n\n");
    std::printf("%6s %10s %12s %12s %12s\n",
                "bands", "input", "Q=1", "matched", "matched x4");
    for (int nb : {128, 512}) {
        const double qm = matched_q(nb);
        for (double amp : {0.05, 0.2, 0.6}) {
            std::printf("%6d %10.2f %9.2fdB %9.2fdB %9.2fdB\n", nb, amp,
                        steady_duck_db(nb, 1.0, false, amp),
                        steady_duck_db(nb, qm, true, amp),
                        steady_duck_db(nb, qm * 4.0, true, amp));
        }
    }
    return 0;
}
