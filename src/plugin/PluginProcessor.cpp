#include "PluginProcessor.h"
#include "PluginEditor.h"

#include <algorithm>
#include <cmath>

namespace {
// Log-spaced frequency map: node i -> f_i, shared by bank and display.
double node_frequency(int i, int n, double f_min, double f_max) {
    const double t = static_cast<double>(i) / static_cast<double>(n - 1);
    return f_min * std::pow(f_max / f_min, t);
}
}  // namespace

// ---- Flutter rate compensation ------------------------------------------
// Flutter Hz was measured swinging amplitude ~50x across its range: the
// shedding drives the string's fundamental resonantly near f0 = c/span, and
// above that the string's own mass and tension lowpass it away. That made
// the rate dial behave as much like a volume control as a rate control.
//
// The response shape is approximately UNIVERSAL in f/f0 (measured across
// tension 150/300/600 and 256/512 nodes: three of four configurations agree
// within 0.19 of each other once normalized, and the outlier deviates mostly
// below the reachable dial range). So one normalized table, evaluated at
// f/f0, adapts automatically to Tension, node count and cloth length.
//
// Table is peak-normalized response vs f/f0, log-interpolated.
namespace {
constexpr double kRateRatio[] = {0.15, 0.30, 0.50, 0.75, 1.00,
                                 1.50, 2.50, 4.00, 7.00, 12.0, 25.0};
constexpr double kRateResp[]  = {0.23, 0.43, 0.75, 0.93, 1.00,
                                 0.93, 0.63, 0.34, 0.117, 0.043, 0.010};
constexpr int kRateN = 11;
// Response at the DEFAULT operating point (3 Hz, tension 300, 512 nodes ->
// f/f0 = 5.11). Normalizing to this leaves the default feel untouched.
constexpr double kRateRef = 0.213;
// Boost is clamped: fully flattening the top would need ~100x, which is
// forcing the string cannot use. 8x gets the working range flat and lets the
// extreme top stay quiet, which is honest - fast flutter IS subtle.
constexpr double kRateCompMin = 0.25, kRateCompMax = 8.0;

// ---- excitation budget: Motion / Character ------------------------------
// Force and Flutter used to be two independent AMOUNT controls feeding the
// same string, so their contributions added and neither dial meant anything
// on its own. Measured: the two sources add roughly LINEARLY in peak
// displacement (10.6% mean error over a 4x4 grid), and each source's own
// peak-vs-dial curve is a clean power law:
//
//     wind: peak = 0.8720 * dial^1.679      (measured, 3.7% error)
//     shed: peak = 1.2756 * dial^0.707      (measured, 1.8% error)
//
// So the pair is replaced by one AMOUNT control and one BALANCE control:
//   Motion    -> target peak displacement, linearly
//   Character -> how that target is split between gusting and shedding
// and the two source dials are recovered by inverting the curves above.
// Motion therefore means the same thing at every Character setting, which
// is what dissolves the shared-budget problem rather than documenting it.
//
// kMotionPeak is the peak at Motion = 1. The binding limit is Character = 0,
// where wind alone tops out at 0.872, so 0.85 sits just under that and well
// under the +-1 clamp the bank and the display share.
constexpr double kWindA = 0.8720, kWindP = 1.679;
constexpr double kShedA = 1.2756, kShedP = 0.707;
constexpr double kMotionPeak = 0.85;
// Velocity blend scale: |v| / |u| measures about 10 at Tension 300, so
// 30/Tension brings the velocity term to roughly the same rms and peak as
// the displacement term.
constexpr double kVelScale = 30.0;

// Second-order correction. Linear addition gets most of the way, but two
// INCOHERENT sources do not add their peaks exactly - the peaks rarely
// coincide - so a mix under-delivers relative to either source alone. The
// deficit is asymmetric because wind's exponent (1.679) is much steeper than
// shedding's (0.707), so splitting costs more at the wind-dominant end.
// Measured delivered/target, averaged over Motion: 1.00 / 0.78 / 0.89 / 0.99
// at Character 0 / 0.33 / 0.66 / 1. These are the reciprocals, interpolated.
// Per-precision source gains. The two sources do NOT scale with node count
// the same way - measured A_wind 0.400/0.829/1.017 and A_shed 0.617/1.296/
// 1.193 at 128/256/512 nodes - so without this, Motion = 1 gave peak ~0.37
// at 128 nodes and clamped at 512. Indexed by Precision, which has exactly
// three values, so the table is exact rather than an interpolation.
// (This also supersedes the old (128/n)^0.25 wind term, which had the wrong
// magnitude; it is kept below and these gains are measured on top of it.)
constexpr double kNodeWindGain[3] = {1.590, 1.031, 0.913};
// Tension compensation exponent, per precision: wind_speed scales as
// (c/300)^kTensionP so that a given Motion means the same amplitude at any
// Tension. Theory says 1.0 -- at steady state drag balances tension,
// rhoCd*W^2 ~= c^2 k^2 u, so u is Tension-free only when W scales linearly
// with c -- and the measured optimum sits just under that because a
// faster-moving string sees less relative velocity.
//
// It is per-precision because the measurement says so. Peak against Tension
// is a HUMP, not a slope: it maxes near a fixed f0 = c/span, so the hump
// sits at a different Tension in each mode and no single exponent flattens
// the whole dial. These values flatten the UPPER half (300 -> 600), which is
// where "turning Tension up makes it quieter" is actually felt.
//
//   peak ratio, Tension 300 -> 600:   was (0.6)      now
//     128 nodes                        0.56x        0.79x   (p 0.85)
//     256 nodes                        0.61x        0.79x   (p 0.80)
//     512 nodes                        0.94x        0.94x   (p 0.60, unchanged
//                                                            -- not broken)
// Whole-dial spread at 128 nodes: 3.15x -> 1.46x.
//
// Note (c/300)^p == 1 at Tension 300 for ANY p, so the Motion calibration at
// the default Tension is untouched by this and needs no re-fit.
constexpr double kTensionP[3] = {0.85, 0.80, 0.60};
// Mid-band ripple amount, as a multiple of wind_speed. Calibrated against
// the measured spatial spectrum: see the Wind entry in SPEC-0.0.2.
constexpr double kMidAmp = 1.2;
// Mid-band shape. The envelope peak and the resonance lock sit at the SAME
// place by construction, so moving the lock moves where the detail lands.
// 0.5 put it mid-cloth and the ripples faded out over the last fifth
// (user: "the mid-level waves dampen a bit too much toward the end").
// Measured tip-fifth curvature at amount 1.2: 0.65 gives 11120 against
// 6870 for the old 0.5/2.0 shape, for 1.15x peak against 1.06x.
constexpr double kMidLockD = 0.65;
constexpr double kMidEnv = 1.4;

// --- mid-band travel speed -----------------------------------------------
// The ripples are resonant, so their natural travel speed is the string's
// own wave speed and nothing else. Detuning away from it is possible but
// costs amplitude, and MEASURED the resonance is broad enough to be useful:
// normalized response against the travel multiplier, at the shipped shape,
//
//   travel  0.35  0.50  0.65  0.80  1.00  1.20  1.45  1.80  2.40
//   R       0.286 0.346 0.476 0.794 1.000 0.593 0.286 0.166 0.114
//
// It is asymmetric: a SLOWER forcing still pushes the string quasi
// statically, while a faster one outruns it. Wind maps to 0.55 .. 1.25 (a
// 2.3x travel range) and the response is compensated out of it, exactly as
// Flutter Hz compensates its own rate, so Wind changes how fast the ripples
// run down the string rather than how loud they are.
// Wind's mid-band phase speed, as a multiple of the string's own wave
// speed. The resonance -- and with it the tip whip -- peaks sharply at 1.0:
// measured tip motion 4045 / 14327 / 10384 at travel 0.80 / 1.00 / 1.20,
// collapsing to 1465 by 0.35.
//
// The range used to be 0.55..1.25, whose log centre is 0.83 -- BELOW
// resonance. So most of the Wind knob sat detuned, peak whip hid at 73% of
// travel, and the bottom of the dial reached the tip with 1495 against a
// possible 14327. That is the "isn't gaining the movement towards the end"
// report. 0.80..1.25 is log-centred on 1.0, which puts maximum whip at the
// MIDDLE of the knob and lifts the worst case 2.7x, while keeping a 1.56x
// speed range so the waves still visibly travel faster with more wind.
constexpr double kMidTravelLo = 0.80, kMidTravelHi = 1.25;

// O1: Wrinkle sat OUTSIDE the Motion excitation budget, so it ADDED level
// instead of redistributing it -- measured end to end through the real
// budget, the control signal's RMS rises 2.76x from Wrinkle alone
// (0.0387 -> 0.1067 at Motion 1.0). That breaks the "one amount control"
// rule Wind, Waves and Flutter Hz all already follow.
//
// Calibrated on RMS over a 400-block window, NOT on peak: the peak of a
// chaotic, randomly-gusting cloth has huge run-to-run variance -- identical
// settings read 0.702 and 0.330 on two runs -- so a peak reading cannot
// separate a parameter effect from where the simulation happened to be.
//
// Flap is deliberately NOT compensated. Its own effect is smaller (1.6x)
// and its interaction with Wrinkle is non-monotonic across the sweep, so a
// fitted 2D table would be fitting noise; an earlier attempt at one made
// Flap REDUCE the motion, which is worse than the problem. Flap mostly
// moves energy toward the tip rather than adding it.
constexpr int kWfN = 5;
constexpr double kWfWr[kWfN] = {0.0, 0.975, 1.95, 2.925, 3.90};
constexpr double kWfMul[kWfN] = {1.000, 1.269, 1.478, 2.504, 2.757};

double wrinkle_compensation(double wrinkle_units) {
    const double w = std::clamp(wrinkle_units, kWfWr[0], kWfWr[kWfN - 1]);
    for (int i = 0; i + 1 < kWfN; ++i) {
        if (w > kWfWr[i + 1])
            continue;
        const double t = (w - kWfWr[i]) / (kWfWr[i + 1] - kWfWr[i]);
        return std::max(1.0, kWfMul[i] + t * (kWfMul[i + 1] - kWfMul[i]));
    }
    return kWfMul[kWfN - 1];
}
constexpr double kMidTv[] = {0.35, 0.50, 0.65, 0.80, 1.00, 1.20, 1.45, 1.80, 2.40};
constexpr double kMidTr[] = {0.286, 0.346, 0.476, 0.794, 1.000, 0.593, 0.286, 0.166, 0.114};
constexpr int kMidTN = 9;
constexpr double kMidRef = 0.82;   // R at the default Wind, so 0.5 is a no-op
// Clamped TIGHT, and the reason matters: this table measures DETAIL
// (curvature), but a slow-travelling forcing makes large DISPLACEMENT while
// making little detail. Compensating detail fully therefore inflates the
// peak at the slow end -- measured 1.98x the no-mid baseline at Wind 0 with
// a [0.5, 2.5] clamp. [0.80, 1.25] holds peak to <= 1.28x everywhere while
// still roughly doubling the low-Wind detail against no compensation.
constexpr double kMidCompMin = 0.80, kMidCompMax = 1.25;

// --- Waves: amplitude compensation ---------------------------------------
// Finer gust patterns couple to the string less efficiently, so turning
// Waves up used to drain it. Measured rms |u| against wave_density, 40 s
// per point at 512 nodes (rms, not peak -- peak over a short window is far
// too noisy to tune on):
//
//   density  0.5    1.0    2.3    3.5    5.0    7.0    9.5    13.0   18.0
//   rms      0.139  0.126  0.090  0.057  0.039  0.032  0.029  0.029  0.027
//
// i.e. 2.3x quieter from the default to the old top of the dial, which is
// why the knob needed "way, way up" before its texture change read as
// anything (user). The loss is divided out here so Waves changes CHARACTER
// at roughly constant level and Motion keeps owning amount.
//
// Note also that past density ~7 every band flattens: the knob is simply
// exhausted there, which is why the range stops at 8 rather than going
// further. What still moves above that is the mid band, which is
// deliberately independent of wave_density.
constexpr double kWaveD[] = {0.5, 1.0, 2.3, 3.5, 5.0, 7.0, 9.5, 13.0, 18.0};
constexpr double kWaveR[] = {1.548, 1.397, 1.000, 0.636, 0.431,
                             0.359, 0.325, 0.321, 0.304};
constexpr int kWaveN = 9;
constexpr double kWaveCompMin = 0.7, kWaveCompMax = 2.2;

// Returns a multiplier on the WIND DIAL. peak ~ dial^kWindP, so recovering a
// response ratio R needs the dial multiplied by R^(1/kWindP).
double wave_density_compensation(double density) {
    const double d = std::clamp(density, kWaveD[0], kWaveD[kWaveN - 1]);
    double r = kWaveR[kWaveN - 1];
    for (int i = 0; i + 1 < kWaveN; ++i) {
        if (d > kWaveD[i + 1])
            continue;
        const double f = std::log(d / kWaveD[i]) / std::log(kWaveD[i + 1] / kWaveD[i]);
        r = std::exp(std::log(kWaveR[i]) + f * std::log(kWaveR[i + 1] / kWaveR[i]));
        break;
    }
    return std::clamp(std::pow(1.0 / std::max(r, 1e-9), 1.0 / kWindP),
                      kWaveCompMin, kWaveCompMax);
}

double mid_resonance_compensation(double travel) {
    const double t = std::clamp(travel, kMidTv[0], kMidTv[kMidTN - 1]);
    double resp = kMidTr[kMidTN - 1];
    for (int i = 0; i + 1 < kMidTN; ++i) {
        if (t > kMidTv[i + 1])
            continue;
        const double f = std::log(t / kMidTv[i]) / std::log(kMidTv[i + 1] / kMidTv[i]);
        resp = std::exp(std::log(kMidTr[i]) + f * std::log(kMidTr[i + 1] / kMidTr[i]));
        break;
    }
    return std::clamp(kMidRef / std::max(resp, 1e-9), kMidCompMin, kMidCompMax);
}
constexpr double kNodeShedGain[3] = {2.796, 0.978, 1.099};

constexpr double kMixC[] = {0.0, 0.33, 0.66, 1.0};
// ...and the correction is itself per-precision, because the size of the
// mixing deficit depends on resolution too. Rows are Precision 0/1/2.
constexpr double kMixBoost[3][4] = {
    {0.969, 1.462, 1.106, 0.847},   // 128 nodes
    {1.007, 1.340, 1.017, 0.999},   // 256 nodes
    {0.968, 0.978, 0.805, 1.012},   // 512 nodes
};

double mix_boost(double c, int prec) {
    c = std::clamp(c, 0.0, 1.0);
    const double* row = kMixBoost[std::clamp(prec, 0, 2)];
    for (int i = 0; i + 1 < 4; ++i) {
        if (c > kMixC[i + 1])
            continue;
        const double t = (c - kMixC[i]) / (kMixC[i + 1] - kMixC[i]);
        return row[i] + t * (row[i + 1] - row[i]);
    }
    return row[3];
}

// Invert peak = A * dial^P for the dial that delivers `target`.
// The ceiling is 2, not 1, so a High magnitude on Motion can genuinely
// over-drive: at Medium the dial never exceeds 1 and the calibration holds
// exactly, while High deliberately pushes past the calibrated peak (and
// past the +-1 EQ clamp, visibly). Clamping at 1 made High a no-op at the
// top of the dial, which defeated the point of the magnitude selector.
double dial_for_peak(double target, double A, double P) {
    if (target <= 1e-9)
        return 0.0;
    return std::clamp(std::pow(target / A, 1.0 / P), 0.0, 2.0);
}

double flutter_rate_compensation(double ratio) {
    const double r = std::clamp(ratio, kRateRatio[0], kRateRatio[kRateN - 1]);
    double resp = kRateResp[kRateN - 1];
    for (int i = 0; i + 1 < kRateN; ++i) {
        if (r > kRateRatio[i + 1])
            continue;
        const double t = std::log(r / kRateRatio[i]) /
                         std::log(kRateRatio[i + 1] / kRateRatio[i]);
        resp = std::exp(std::log(kRateResp[i]) +
                        t * std::log(kRateResp[i + 1] / kRateResp[i]));
        break;
    }
    return std::clamp(kRateRef / std::max(resp, 1e-9), kRateCompMin, kRateCompMax);
}
}  // namespace

const char* CurvesAudioProcessor::mod_knob_id(int k) {
    static const char* kIds[kNumModKnobs] = {
        "damping", "stiffness", "tension", "smoothing", "drive", "steady",
        "leveler", "velocity", "wind", "waves", "motion", "character",
        "flutter_hz", "wrinkle", "mix", "flap", "width", "pan_depth",
        "resonance"};
    return kIds[juce::jlimit(0, (int)kNumModKnobs - 1, k)];
}

CurvesAudioProcessor::PowerMode CurvesAudioProcessor::power_mode(int k) {
    switch (k) {
        case kCharacter:            // a balance: its extremes already mean
        case kWaves:                // all-wind / all-flutter
        case kTension:              // a scale has no "off"
            return kPowerNone;
        case kWind:                 // gates the gust field entirely
        case kFlutterHz:            // gates vortex shedding entirely
        case kRangeDb:              // gates the EQ subscriber entirely, so
                                    // EQ and spatial can run independently
            return kPowerGate;
        default:
            return kPowerToMin;
    }
}

// The per-voice parameter set, added twice. Voice 0 keeps the historical
// parameter IDs so projects saved before Dual existed still recall; voice 1
// takes an "_r" suffix and an " R" label suffix.
//
// Both sets are REAL automatable parameters. The alternative -- keeping
// voice 1 in the state tree only -- would make the right channel
// un-automatable, which is not a plugin anyone wants. The cost is a long
// host parameter list and a rescan.
void CurvesAudioProcessor::add_voice_params(VoiceParams& p, int voice) {
    using APF = juce::AudioParameterFloat;
    using R = juce::NormalisableRange<float>;
    const juce::String sfx = voice == 0 ? juce::String() : juce::String("_r");
    const juce::String tag = voice == 0 ? juce::String() : juce::String(" R");
    auto ID = [&](const char* b) { return juce::String(b) + sfx; };
    auto NM = [&](const char* b) { return juce::String(b) + tag; };
    // Dial ranges calibrated by feel-testing (user): meaningful damping by
    // 45% of the old 0-40 dial -> max 18; meaningful force by 25% of the old
    // dial -> drive/ceiling compressed x0.25 (see processBlock).
    addParameter(p.damping = new APF(ID("damping"), NM("Damping"), R{0.05f, 18.0f}, 1.5f));
    addParameter(p.stiffness = new APF(ID("stiffness"), NM("Stiffness"), R{0.0f, 20.0f}, 4.0f));
    addParameter(p.tension = new APF(ID("tension"), NM("Tension"), R{50.0f, 600.0f}, 300.0f));
    // 30 -> 15 ms: the smoother used to advance once PER CHANNEL, so a stereo
    // instance ran at half the set time. Fixing that doubled the real
    // smoothing at any given setting; 15 ms restores the previous feel.
    addParameter(p.smoothing = new APF(ID("smoothing"), NM("Smoothing ms"), R{2.0f, 100.0f}, 15.0f));
    addParameter(p.drive = new APF(ID("drive"), NM("Max gain dB"), R{3.0f, 24.0f}, 18.0f));
    addParameter(p.steady = new APF(ID("steady"), NM("Steady (per-band)"), R{0.0f, 1.0f}, 0.35f));
    addParameter(p.physics = new APF(ID("physics"), NM("Physics"), R(0.0f, 3.0f, 1.0f), 0.0f));
    addParameter(p.velocity = new APF(ID("velocity"), NM("Velocity"), R{0.0f, 1.0f}, 0.0f));
    addParameter(p.wind = new APF(ID("wind"), NM("Wind"), R{0.0f, 1.0f}, 0.50f));
    addParameter(p.waves = new APF(ID("waves"), NM("Waves"), R{0.0f, 1.0f}, 0.24f));
    // Motion = total string excitation, calibrated so peak displacement
    // tracks the dial linearly regardless of Character. Default chosen to
    // match the previous Force 0.6 wind-only feel (peak ~0.37).
    addParameter(p.motion = new APF(ID("motion"), NM("Motion"), R{0.0f, 1.0f}, 0.45f));
    // Character = where that excitation comes from. 0 = all gust wind,
    // 1 = all self-sustaining vortex shedding. Default 0 reproduces the
    // previous behaviour exactly; raising it is a tuning decision.
    addParameter(p.character = new APF(ID("character"), NM("Character"), R{0.0f, 1.0f}, 0.0f));
    addParameter(p.direction = new APF(ID("direction"), NM("Direction"), R(0.0f, 1.0f, 1.0f), 0.0f));
    // Walls replace the old Cloth-length knob. Two independent boundaries,
    // dragged on the canvas, so any window anywhere along the spectrum can be
    // the active cloth. Length could only ever shorten from the free end and,
    // worse, did not actually work: measured, Length 0.6 still left 99% of
    // the string moving, because the "near-rigid tail" was not a hard wall.
    addParameter(p.wall_lo = new APF(ID("wall_lo"), NM("Wall Low"), R{0.0f, 1.0f}, 0.0f));
    addParameter(p.wall_hi = new APF(ID("wall_hi"), NM("Wall High"), R{0.0f, 1.0f}, 1.0f));
    // Flutter = vortex shedding. The wake oscillator is a limit cycle, so
    // this is the one control that makes the string move with NO gusting at
    // all. Default 0 (off) so the existing Flagpole feel is unchanged until
    // it is dialled in.

    // Nominal wake frequency. In Curves this is literally the rate the EQ
    // bands wobble at, so it is set in Hz and tuned by ear: below ~0.5 Hz it
    // is inert, 0.5-4 Hz is the expressive band, and above that the string's
    // own mass and tension filter it out (measured: peak 0.35 at 3 Hz but
    // only 0.03 at 12 Hz). Per-band rate varies around this by the shedding
    // gradient, which is what makes it read as cloth and not an LFO.
    // Range extended on user request (was 0.3-8.0). Measured consequence,
    // which the dial does NOT compensate for: amplitude swings ~50x across
    // this range. It peaks near 0.45 Hz, where the shedding resonantly
    // drives the string's fundamental (omega = c*2*pi/span lands about
    // there) - 4.35x the amplitude of 3 Hz - and damping barely touches it
    // (4.37 at damping 0.5, 3.48 at damping 12, so it is not a damping
    // problem to solve). Above ~12 Hz the string's own mass and tension
    // lowpass it away (0.08). Left uncompensated because the project
    // prefers true-to-simulation behaviour; gain-compensating the rate is a
    // one-line change if it starts feeling like a volume knob.
    addParameter(p.flutter_hz = new APF(ID("flutter_hz"), NM("Flutter Hz"), R{0.1f, 20.0f}, 3.0f));
    // Wrinkle: tip-localized fine detail - "how the last third resolves or
    // flaps". 0 = off, so the stage costs nothing until dialled in.
    addParameter(p.wrinkle = new APF(ID("wrinkle"), NM("Wrinkle"), R{0.0f, 1.0f}, 0.0f));
    // Flap: how hard the mid-band ripples drive the FREE END. The bump
    // envelope is zero there by construction, so without this the ripples
    // fade out over the last fifth -- they were never forced, not damped.
    addParameter(p.flap = new APF(ID("flap"), NM("Flap"), R{0.0f, 1.0f}, 0.45f));
    // Randomisers. 0 = the fixed constant sets, i.e. exactly the regular
    // behaviour; turning one up perturbs those constants from its seed.
    addParameter(p.wind_rnd = new APF(ID("wind_rnd"), NM("Wind Random"),
                                     R{0.0f, 1.0f}, 0.0f));
    addParameter(p.flutter_rnd = new APF(ID("flutter_rnd"), NM("Flutter Random"),
                                        R{0.0f, 1.0f}, 0.0f));
    // ---- SPATIAL subscriber. Neutral at zero, so it costs nothing and
    // changes nothing until it is turned up.
    addParameter(p.width = new APF(ID("width"), NM("Width dB"), R{0.0f, 24.0f}, 0.0f));
    addParameter(p.pan_depth = new APF(ID("pan_depth"), NM("Placement"), R{0.0f, 1.0f}, 0.0f));
    addParameter(p.stereo_lo = new APF(ID("stereo_lo"), NM("Spatial low cut"),
                                      R{20.0f, 2000.0f}, 120.0f));
    addParameter(p.stereo_hi = new APF(ID("stereo_hi"), NM("Spatial high cut"),
                                      R{1000.0f, 18000.0f}, 12000.0f));
    // ---- BAND Q. 0 = Flat: Q = 1 with no overlap correction, which is the
    // historical bank BIT-FOR-BIT. 1 = Matched: band width equals band
    // spacing, plus the analytic 1/K overlap correction. The flag exists so
    // the two can be A/B'd, because Matched changes every sound: at Q = 1
    // the bank passes NO spectral detail at all (measured 0.00 dB survival
    // of a +-6 dB pattern alternating every 2, 4, 8 or 16 bands, at both
    // 128 and 512 nodes) while delivering ~24x the asked-for depth on the
    // string's coarse average. Matched trades that for honest depth and
    // real resolution. See q_test.
    addParameter(p.q_mode = new APF(ID("q_mode"), NM("Band Q"), R{0.0f, 1.0f}, 0.0f));
    // Resonance pushes Q ABOVE matched: x1 to x8. Narrower bands ring
    // longer (T60 = 6.91*Q/(pi*f), so 754 ms at 58 Hz already at matched Q
    // on 128 nodes) and stop filling the gaps between each other, which
    // turns the response into a comb -- the ripple is 0.93 dB at matched
    // and 5.73 dB at x8. Both effects are the point: this is the knob that
    // buys the ringing, harmonic-comb texture.
    addParameter(p.resonance = new APF(ID("resonance"), NM("Resonance"), R{0.0f, 1.0f}, 0.0f));
    // Detail: smooth the SMALL waves harder while leaving the big ones
    // alone. Smoothing is one time constant for the whole control signal,
    // so slowing the flutter also slows the broad sway. This splits the
    // signal by spatial scale first -- a bidirectional one-pole across the
    // NODES, cutting around five waves per string -- and gives the fine
    // part up to 16x the smoothing. Zero is bit-identical to no split at
    // all: the stage is skipped outright, like mid_ripple.
    addParameter(p.detail = new APF(ID("detail"), NM("Detail"), R{0.0f, 1.0f}, 0.0f));
}

CurvesAudioProcessor::CurvesAudioProcessor()
    : AudioProcessor(BusesProperties()
                         .withInput("Input", juce::AudioChannelSet::stereo(), true)
                         .withOutput("Output", juce::AudioChannelSet::stereo(), true)) {
    using APF = juce::AudioParameterFloat;
    using R = juce::NormalisableRange<float>;
    // ---- GLOBAL parameters: output plumbing and the routing switch ----
    // Dry/wet. 1 = fully processed, 0 = bypass (so power-off = its minimum
    // is genuinely neutral). Applied after the bank and BEFORE the leveler,
    // so the leveler trims what you actually hear.
    addParameter(mix_ = new APF("mix", "Mix", R{0.0f, 1.0f}, 1.0f));
    addParameter(leveler_amt_ = new APF("leveler", "Leveler (output)", R{0.0f, 1.0f}, 0.5f));
    addParameter(test_src_ = new APF("test", "Test source",
                                     R(0.0f, 5.0f, 1.0f), 0.0f));
    addParameter(precision_ = new APF("precision", "Precision", R(0.0f, 2.0f, 1.0f), 0.0f));
    // Dual: two independent strings, voice 0 on LEFT and voice 1 on RIGHT,
    // each driving a MONO bank. Stereo then comes from the two physics
    // simulations genuinely diverging rather than from one string's
    // mid/side -- which is what the sandbox was aimed at from the start.
    //
    // A parameter rather than a UI flag because it changes the audio path.
    // Note that per-voice Width and Place go INERT in dual mode: a voice
    // sees one channel, so its side signal is zero by construction. The
    // editor dims them to say so.
    addParameter(dual_ = new APF("dual", "Dual L/R", R(0.0f, 1.0f, 1.0f), 0.0f));
    // Link: every per-voice edit is written to BOTH voices. Dial a sound in
    // on the pair, then unlink and let them diverge -- which is the usual
    // way anyone works a stereo pair. Note that fully linked voices are
    // identical, so Dual + Link sounds mono by definition; that is the
    // starting point, not the destination.
    addParameter(link_ = new APF("link", "Link L/R", R(0.0f, 1.0f, 1.0f), 0.0f));

    // ---- the two per-voice sets ----
    for (int v = 0; v < kNumVoices; ++v)
        add_voice_params(vp_[v], v);

    // Capture each modifiable knob's minimum; "off" resolves to it. Done
    // per voice against that voice's own parameter IDs -- the two sets have
    // identical ranges, but reading voice 1's minimum off voice 0's
    // parameter is the kind of shared lookup that would quietly couple them.
    for (int v = 0; v < kNumVoices; ++v)
        for (int k = 0; k < kNumModKnobs; ++k) {
            voices_[v].mods[k].lo = 0.0;
            // Global knobs exist only under their voice-0 id, so look them
            // up there whichever voice we are filling.
            const juce::String want =
                voice_param_id(mod_knob_id(k), is_global_knob(k) ? 0 : v);
            for (auto* prm : getParameters())
                if (auto* rp = dynamic_cast<juce::RangedAudioParameter*>(prm))
                    if (rp->paramID == want)
                        voices_[v].mods[k].lo = rp->getNormalisableRange().start;
        }
}

// Turning Dual on gives voice 1 a usable starting point: it copies voice 0's
// whole set, then re-rolls its seeds. Starting it at defaults would mean the
// right channel jumped to a different sound; starting it IDENTICAL would
// mean dual mode sounded like mono until you moved something. A copy plus
// fresh seeds is the same weather, different gusts -- audibly stereo
// immediately, and still recognisably the patch you had.
void CurvesAudioProcessor::set_dual(bool on) {
    if (dual_ == nullptr)
        return;
    if (on && dual_->get() <= 0.5f)
        copy_voice(0, 1);
    dual_->beginChangeGesture();
    dual_->setValueNotifyingHost(on ? 1.0f : 0.0f);
    dual_->endChangeGesture();
}

// Every per-voice parameter, by its BASE id. Used to copy one voice onto
// another and to let the editor rebind its knobs when you switch voices.
// An explicit list rather than walking the VoiceParams struct as memory:
// the struct-as-pointer-array trick works right up until someone adds a
// non-pointer field to it.
const char* const* CurvesAudioProcessor::voice_param_ids(int& count) {
    static const char* const kIds[] = {
        "damping", "stiffness", "tension", "smoothing", "drive", "steady",
        "physics", "velocity", "wind", "waves", "motion", "character",
        "direction", "wall_lo", "wall_hi", "flutter_hz", "wrinkle", "flap",
        "wind_rnd", "flutter_rnd", "width", "pan_depth", "stereo_lo",
        "stereo_hi", "q_mode", "resonance", "detail"};
    count = (int)(sizeof(kIds) / sizeof(kIds[0]));
    return kIds;
}

juce::AudioParameterFloat* CurvesAudioProcessor::find_param(
    const juce::String& id) const {
    for (auto* prm : getParameters())
        if (auto* rp = dynamic_cast<juce::AudioParameterFloat*>(prm))
            if (rp->paramID == id)
                return rp;
    return nullptr;
}

void CurvesAudioProcessor::set_link(bool on) {
    if (link_ == nullptr)
        return;
    // Mirror exactly, seeds included: a "linked" pair that still has
    // different weather is not linked.
    if (on && link_->get() <= 0.5f)
        copy_voice(ui_voice_, 1 - ui_voice_, false);
    link_->beginChangeGesture();
    link_->setValueNotifyingHost(on ? 1.0f : 0.0f);
    link_->endChangeGesture();
}

void CurvesAudioProcessor::copy_voice(int from, int to, bool reroll_seeds) {
    if (from == to)
        return;
    int n = 0;
    const char* const* ids = voice_param_ids(n);
    for (int i = 0; i < n; ++i) {
        auto* a = find_param(voice_param_id(ids[i], from));
        auto* b = find_param(voice_param_id(ids[i], to));
        if (a == nullptr || b == nullptr)
            continue;
        b->beginChangeGesture();
        b->setValueNotifyingHost(b->convertTo0to1(a->get()));
        b->endChangeGesture();
    }
    voices_[to].mods = voices_[from].mods;
    if (reroll_seeds) {
        voices_[to].flutter_seed = next_seed(voices_[from].flutter_seed + 7919);
        voices_[to].wind_seed = next_seed(voices_[from].wind_seed + 104729);
    } else {
        voices_[to].flutter_seed = voices_[from].flutter_seed.load();
        voices_[to].wind_seed = voices_[from].wind_seed.load();
    }
}

void CurvesAudioProcessor::prepareToPlay(double sampleRate, int) {
    fs_ = sampleRate;
    leveler_.prepare(fs_);
    last_precision_ = -1;  // processBlock rebuilds with the current setting
    for (auto& v : voices_)
        v.sim = nullptr;
}

void CurvesAudioProcessor::releaseResources() {}

bool CurvesAudioProcessor::isBusesLayoutSupported(const BusesLayout& layouts) const {
    const auto& in = layouts.getMainInputChannelSet();
    const auto& out = layouts.getMainOutputChannelSet();
    return in == out && (in == juce::AudioChannelSet::mono() ||
                         in == juce::AudioChannelSet::stereo());
}

void CurvesAudioProcessor::rebuild(int precision) {
    n_nodes_ = kPrecisionNodes[precision];
    sim_divisor_ = kPrecisionDivisor[precision];
    // The band map is identical for every voice (it depends only on node
    // count and sample rate), so it is computed once and shared; each
    // voice's bank still has the frequencies set on it individually.
    band_fc_.resize(n_nodes_);
    for (int i = 0; i < n_nodes_; ++i)
        band_fc_[i] = std::min(node_frequency(i, n_nodes_, kFMin, kFMax), 0.45 * fs_);
    // EVERY voice is rebuilt, not just the active ones: switching Dual on
    // mid-session must not find voice 1 holding a sim from the old node
    // count, which would be a silent buffer-overrun.
    for (int v = 0; v < kNumVoices; ++v)
        rebuild_voice(voices_[v], vp_[v]);
}

void CurvesAudioProcessor::rebuild_voice(Voice& v, const VoiceParams& p) {
    curves::StringParams sp;
    sp.tick_rate = fs_ / sim_divisor_;
    sp.wave_speed = p.tension->get();
    sp.damping = p.damping->get();
    sp.stiffness = p.stiffness->get();
    v.sim = std::make_unique<curves::StringSim>(n_nodes_, sp);
    v.sim_counter = 0;

    v.bank.prepare(n_nodes_, fs_, 2);
    v.last_q = -1.0;  // matched Q depends on node count; force a re-apply
    v.bank.set_steady(knob(v, kSteady, p.steady->get()), 0.25);
    rebuild_band_map(v);
    v.gain_targets.assign(n_nodes_, 1.0);
    // Display response scratch. Allocated here so publish_display never
    // allocates on the audio thread.
    v.resp_big.assign(n_nodes_, 0.0);
    v.resp_re.assign(n_nodes_, 0.0);
    v.resp_im.assign(n_nodes_, 0.0);
    v.smoothed_disp.fill(0.0);
    v.last_physics = -1;   // force re-apply on the current preset
    v.taper_lo = -1.0;     // and force the spatial taper to rebuild
    v.taper_hi = -1.0;
}

void CurvesAudioProcessor::apply_physics(Voice& v, int mode) {
    if (!v.sim)
        return;
    v.sim->anchor_low = false;
    v.sim->anchor_high = false;
    v.sim->force_model = curves::StringSim::kStaticField;
    v.sim->set_density_taper(0.0);
    std::vector<double> field(n_nodes_, 0.0);

    if (mode == kFlagpole) {
        // Cloth wind: the pole (high end) is anchored; the LEFT END IS FREE
        // — it moves and whips (user decision: pinning it kills the whip).
        // Cloth length sets where the animation terminates; beyond it the
        // wind is zero and damping is multiplied (termination behavior).
        v.sim->anchor_high = true;
        // Aero model: the wind is a VELOCITY field and the force comes from
        // relative velocity, so the string fights the wind and flap
        // amplitude is bounded by drag instead of by a limiter. Phase speed
        // scales with node count so Wind means the same thing in every
        // precision mode (at a fixed nodes/s the pattern would traverse a
        // 1024-node string 8x slower than a 128-node one).
        v.sim->set_cloth_aero(192.0 * n_nodes_ / 128.0,
                             static_cast<double>(n_nodes_), 1.5);
        v.sim->aero_rho_cd = 0.6;
        v.sim->set_density_taper(2.0);
        // Walls own the active extent now, so cloth_length stays at 1 and the
        // walls are the single owner of the geometry. (The old Cloth-length
        // knob both desynced across apply_physics and did not actually hold:
        // measured, length 0.6 still left 99% of the string moving.)
        v.sim->set_cloth_length(1.0);
        v.last_wall_lo = -1.0;   // force the walls to re-apply to the new sim
    } else if (mode == kGust) {
        // No anchors; smooth zero-mean static force field across the spectrum.
        const double w = 8.0 * (static_cast<double>(n_nodes_) / 128.0);
        for (int i = 0; i < n_nodes_; ++i)
            field[i] = -w * std::sin(3.14159265358979323846 * i / (n_nodes_ - 1));
    }
    // kFree / kElastic: no anchors, no forces — knobs do the talking.
    v.sim->set_force_field(field);
}

// Damps the SPATIAL deviation outside [lo, hi], leaving the EQ alone.
// Widening the bass is almost always wrong, and the log axis puts bass at
// the left end, so a flat width depth misbehaves down there. This is a
// taper on the DEVIATION, not a filter on the audio: outside the window
// width goes to 1 and pan to 0, i.e. those bands are simply left alone.
// One octave of smoothstep at each edge keeps the drawn curve continuous.
void CurvesAudioProcessor::rebuild_spatial_taper(Voice& v,
                                                 const VoiceParams& p) {
    v.spatial_w.assign(n_nodes_, 1.0);
    const double lo = std::max(1.0, (double)p.stereo_lo->get());
    const double hi = std::max(lo * 2.0, (double)p.stereo_hi->get());
    const double l2lo = std::log2(lo), l2hi = std::log2(hi);
    auto smooth = [](double t) {
        t = std::clamp(t, 0.0, 1.0);
        return t * t * (3.0 - 2.0 * t);
    };
    for (int i = 0; i < n_nodes_; ++i) {
        const double f = std::max(1.0, band_fc_[i]);
        const double l2 = std::log2(f);
        v.spatial_w[i] = smooth(l2 - l2lo) * smooth(l2hi - l2);
    }
    v.taper_lo = lo;
    v.taper_hi = hi;
}

// The Q whose -3 dB bandwidth equals the band spacing:
// Q = sqrt(2^BW)/(2^BW - 1) with BW = log2(fmax/fmin)/(N-1) octaves.
// 19.85 at 128 nodes, 39.86 at 256, 79.88 at 512.
double CurvesAudioProcessor::matched_q(int n) {
    const double bw = std::log2(kFMax / kFMin) / std::max(1, n - 1);
    const double t = std::pow(2.0, bw);
    return std::sqrt(t) / (t - 1.0);
}

// band_fc_ is filled by rebuild(); this only pushes it into one voice's bank.
void CurvesAudioProcessor::rebuild_band_map(Voice& v) {
    for (int i = 0; i < n_nodes_; ++i)
        v.bank.set_band_frequency(i, band_fc_[i]);
}

// Maps ONE voice's parameters onto its engine. This was the first two
// thirds of processBlock, reading flat members; it now reads a Voice and a
// VoiceParams, which is what makes two independent instances possible at
// all. Everything global (Mix, Leveler, Test Src, Precision) stayed behind.
void CurvesAudioProcessor::prepare_voice(Voice& v, const VoiceParams& p,
                                         int prec_idx, VoiceBlock& out) {
    // Physics preset: anchors + force field when it changes.
    const int phys = static_cast<int>(p.physics->get());
    if (phys != v.last_physics) {
        apply_physics(v, phys);
        v.last_physics = phys;
    }
    // 2. Live physics parameters (cheap, per block).
    // Declared up here because the Wind block below sets mid_flap and the
    // excitation budget further down divides by both (O1).
    const double wrinkle_units = 3.9 * knob(v, kWrinkle, p.wrinkle->get());
    const double flap_amt = knob(v, kFlap, p.flap->get());
    v.sim->set_damping(knob(v, kDamping, p.damping->get()));
    v.sim->set_stiffness(knob(v, kStiffness, p.stiffness->get()));
    v.sim->set_wave_speed(knob(v, kTension, p.tension->get()));
    // Wind = gust PHASE VELOCITY: how fast the flapping pattern travels
    // along the cloth.
    //
    // It is set RELATIVE TO THE STRING, as a multiple of the wave speed, for
    // the same reason Flutter Hz is normalized by f0: what you see depends
    // entirely on whether the string can FOLLOW the gust, and that is a
    // question about f_gust / f0, not about absolute nodes/s.
    //
    // The old mapping, (10 + 490*Wind) * n/128, held the gust frequency
    // constant across precision while f0 = c/span falls as 1/n. Measured,
    // that put the same Wind setting at wildly different places relative to
    // the string's resonance -- f_gust/f0 = 0.64 at 128 nodes but 2.54 at
    // 512, with the dial top reaching 6.7 where the string cannot follow at
    // all. At 512 the usable action was squeezed into the bottom quarter of
    // the travel and the rest was a slow fade, which is why the knob read as
    // doing nothing (user: "wind is not really affecting the string anymore
    // ... no difference at high levels or low levels"). Measured at 512:
    // peak 0.21 at Wind 0, 0.52 at Wind 0.15, 0.18 at Wind 1.
    //
    // Damping is NOT the lever here: a 10x damping change moves the observed
    // motion frequency by 4%, because drag dominates dissipation.
    //
    // ratio = 0.12 * 25^Wind spans 0.12 .. 3.0 of the fundamental, which is
    // the band where the string actually tracks the gust at every precision.
    // Wind 0.5 gives 0.60, the ratio the 128-node calibration was tuned at.
    {
        const double wind_knob = knob(v, kWind, p.wind->get());
        const double ratio = 0.12 * std::pow(25.0, wind_knob);
        const double c = std::max(1.0, knob(v, kTension, p.tension->get()));
        v.sim->wind_phase_speed = ratio * c;
        // Mid-band ripples ride the same knob: Wind picks their SIZE, 6 -> 20
        // waves across the cloth. Amount follows Motion (via wind_speed), so
        // there is no extra control -- measured, the gust sites cover only
        // 1.7-6 waves and wrinkle 25-64, and nothing reached the middle.
        v.sim->mid_waves = 6.0 * std::pow(20.0 / 6.0, wind_knob);
        v.sim->mid_lock_d = kMidLockD;
        v.sim->mid_env = kMidEnv;
        v.sim->mid_flap = flap_amt;
        v.sim->gust_spread = p.wind_rnd->get();
        v.sim->gust_seed = v.wind_seed.load(std::memory_order_relaxed);
        // Wind also sets how fast the ripples RUN DOWN the string, with the
        // detuning loss compensated so it stays a speed control.
        const double travel =
            kMidTravelLo * std::pow(kMidTravelHi / kMidTravelLo, wind_knob);
        v.sim->mid_travel = travel;
        v.sim->mid_ripple =
            mechanism_on(v, kWind) ? kMidAmp * mid_resonance_compensation(travel) : 0.0;
    }
    // Waves knob: spatial concentration — 0.5x (stretched) to 3x (packed)
    // Waves range extended 0.5-3.0 -> 0.5-5.0 (user: "not noticing waves
    // doing much"). Measured spatial centroid vs density: 1.03 / 1.21 /
    // 1.38 / 1.50 / 1.63 at 0.5 / 1.5 / 3 / 5 / 8, so the old top of 3 sat
    // well short of the usable range. It reverses past ~8, so 5 at Medium
    // (and ~9.5 at High magnitude) covers it.
    //
    // Side effect, deliberately not compensated: finer forcing couples into
    // the string less efficiently, so peak falls ~1.7x across the new range.
    // Motion is the amplitude control and absorbs it.
    // Range extended 5.0 -> 8.0 (user asked for more). Measured, every band
    // flattens past ~7, so 8 is where it genuinely stops doing anything
    // rather than an arbitrary stop. The default knob moved 0.40 -> 0.24
    // so the shipped density stays 2.3 and the stock character is intact.
    v.sim->wave_density = 0.5 + 7.5 * knob(v, kWaves, p.waves->get());
    v.sim->set_wind_origin(p.direction->get() > 0.5f ? 1 : 0);
    // Walls: only rebuild the geometry when they actually move.
    const double wlo = p.wall_lo->get(), whi = p.wall_hi->get();
    if (std::abs(wlo - v.last_wall_lo) > 1e-4 || std::abs(whi - v.last_wall_hi) > 1e-4) {
        v.sim->set_walls(wlo, whi);
        v.last_wall_lo = wlo;
        v.last_wall_hi = whi;
    }
    // --- excitation budget --------------------------------------------
    // One amount control (Motion) and one balance control (Character); the
    // individual source dials are solved for, not set directly.
    const double motion = knob(v, kMotion, p.motion->get());
    const double character = knob(v, kCharacter, p.character->get());
    // Wrinkle and Flap are now INSIDE the budget: the gust and shedding
    // sources are solved for a target that has already been divided by
    // what those two will add, so turning them up redistributes the
    // motion instead of piling on top of it (O1).
    const double target_peak =
        kMotionPeak * motion * mix_boost(character, prec_idx)
        / wrinkle_compensation(wrinkle_units);
    const double wind_dial = dial_for_peak((1.0 - character) * target_peak,
                                           kWindA, kWindP);
    const double shed_dial = dial_for_peak(character * target_peak,
                                           kShedA, kShedP);

    // Source strengths. Both dials come from the budget above; these two
    // lines only convert a dial into engine units.
    //
    // The tension term makes a given dial mean the same thing at any Tension
    // (see kTensionP: it is per-precision, and the old flat 0.6 left the
    // string audibly quieter at high Tension). The node term plus kNodeWindGain hold
    // it across precision modes - the two sources scale differently with
    // resolution, so each carries its own per-precision gain.
    const double tension_v = knob(v, kTension, p.tension->get());
    const double tension_norm =
        std::pow(std::max(1.0, tension_v) / 300.0, kTensionP[prec_idx]);
    const double node_norm = std::pow(128.0 / std::max(1, n_nodes_), 0.25);
    // Wind's power LED gates the gust MECHANISM: off means no gust forcing
    // at all, which is what "turn the wind off" has to mean. Previously
    // only its rate dropped to minimum and the gusts kept blowing slowly.
    const double gust_speed = 25.0 * wind_dial * tension_norm * node_norm
                              * kNodeWindGain[prec_idx]
                              * wave_density_compensation(v.sim->wave_density);
    v.sim->wind_speed = mechanism_on(v, kWind) ? gust_speed : 0.0;
    // Wrinkle keeps its own amplitude reference, so turning the gust field
    // off does not silence it as well - it is a separate mechanism with its
    // own power switch.
    v.sim->wrinkle_speed = gust_speed;
    // Flap limiter permanently off: aerodynamic drag IS the amplitude
    // mechanism now. Kept in the engine rather than deleted so the legacy
    // kFlagpoleWind model and the frozen validation scenarios still behave
    // exactly as their baselines record.
    v.sim->flap_limit = 1e9;
    // Flutter amount, with the rate dial's amplitude swing compensated out so
    // Flutter Hz is a RATE control rather than a rate-and-volume control.
    // f0 is the string's fundamental and moves with Tension, node count and
    // cloth length, so it is computed live rather than baked in.
    {
        const double span = std::max(1.0, (n_nodes_ - 1.0)
                                              * std::max(0.02, whi - wlo));
        const double f0 = std::max(1e-6, tension_v / span);
        const double comp = flutter_rate_compensation(
            knob(v, kFlutterHz, p.flutter_hz->get()) / f0);
        // Flutter Hz's power LED gates shedding entirely.
        v.sim->shed_gain = mechanism_on(v, kFlutterHz)
                              ? 90.0 * shed_dial * comp * kNodeShedGain[prec_idx]
                              : 0.0;
    }
    // Wrinkle 0..1 -> 0..3. Measured tip/pole detail ratio 1.11 (off) ->
    // 1.45 at 1.5 -> 1.82 at 3.0, with the POLE end unchanged (0.83 ->
    // 0.87), i.e. the effect stays where it was asked for.
    // +30% amplitude on user request: 3.0 -> 3.9 at the top of the dial.
    v.sim->wrinkle = wrinkle_units;
    v.sim->shed_freq_hz = knob(v, kFlutterHz, p.flutter_hz->get());
    v.sim->shed_spread = p.flutter_rnd->get();
    v.sim->shed_seed = v.flutter_seed.load(std::memory_order_relaxed);
    // Smoothing is now a one-pole on disp at the sim tick rate. Both of
    // these used the RAW knob and so quietly ignored their own magnitude
    // modifiers; they take knob() like every other control now.
    {
        const double tau = std::max(1e-4, knob(v, kSmoothing, p.smoothing->get()) * 1e-3);
        const double tick_rate = fs_ / std::max(1, sim_divisor_);
        v.smooth_alpha = 1.0 - std::exp(-1.0 / (tau * tick_rate));
        v.detail_amt = std::clamp((double)p.detail->get(), 0.0, 1.0);
        // Spatial cutoff as a FRACTION of the node count, so "small" means
        // the same number of waves across the string at every precision.
        const double tau_nodes = 0.03 * std::max(8, n_nodes_);
        v.detail_b = 1.0 - std::exp(-1.0 / tau_nodes);
    }
    v.bank.set_steady(knob(v, kSteady, p.steady->get()), 0.25);
    // Band Q. set_q is O(N) plus an O(N) overlap sum, so it runs only when
    // the setting actually moves -- not per block.
    {
        // Band Q is a GRADIENT, not a switch. Q blends geometrically from 1
        // to matched (pow hits both endpoints exactly), and the overlap
        // correction blends as K^amount alongside it -- so 0 is still the
        // historical bank bit-for-bit and 1 is still fully corrected, with
        // a continuous morph between. A 0/1 switch could not be dialled,
        // and snapping the correction on at the first touch would jump the
        // level by up to 96x.
        const double amt = std::clamp((double)p.q_mode->get(), 0.0, 1.0);
        const double res = knob(v, kResonance, p.resonance->get());
        const double q =
            std::pow(matched_q(n_nodes_), amt) * std::pow(2.0, 3.0 * res);
        if (std::abs(amt - v.last_q_amt) > 1e-9
            || std::abs(q - v.last_q) > 1e-9) {
            v.bank.set_q(q);
            v.bank.set_overlap_correction(amt);
            v.last_q_amt = amt;
            v.last_q = q;
        }
    }
    v.range_db.store(knob(v, kRangeDb, p.drive->get()), std::memory_order_relaxed);

    // 3. Per-block gain targets: a BLEND of position and velocity.
    //
    //    Velocity was additive and divided by Tension alone, which made it
    //    nearly inert: measured, it contributed only 3.4% of the rms
    //    displacement at full. |v|/|u| is about 10 at Tension 300, so the
    //    divisor was ~30x too large. Dividing by Tension is still right in
    //    principle - v/u scales with wave speed, so this keeps the knob
    //    tension-independent - only the constant was wrong.
    //
    //    It is now a blend rather than a sum. At 0 the EQ follows the
    //    string's shape; at 1 it follows how fast the string is moving,
    //    which reads as plucked rather than swaying. A blend is
    //    self-normalising: measured rms and peak of the velocity term with
    //    kVelScale = 30 are close to those of u, so neither extreme pushes
    //    the +-1 clamp the way an additive form would.
    v.vel_amt = knob(v, kVelocity, p.velocity->get());
    v.vel_scale = kVelScale / std::max(1.0, tension_v);
    out.range_db =
        mechanism_on(v, kRangeDb) ? v.range_db.load(std::memory_order_relaxed) : 0.0;
    // The spatial subscriber is neutral at zero depth, so it is skipped
    // outright rather than computed and multiplied by one.
    const double width_db = knob(v, kWidth, p.width->get());
    const double pan_amt = knob(v, kPanDepth, p.pan_depth->get());
    out.width_db = width_db;
    out.pan_amt = pan_amt;
    out.spatial_on = (width_db > 1e-6 || pan_amt > 1e-6);
    // Which subscriber the canvas is showing. The EQ wins when it is on,
    // because that is the one with an absolute dB meaning; with it gated
    // off the curve falls back to the WIDTH shape, which is still being
    // driven. Without this the canvas flatlined while the stereo image was
    // visibly moving -- the shape was in the side gain and nothing read it.
    // Ordered fallback to whichever subscriber is carrying the shape. The
    // EQ wins when on, because dB is the meaning the axis was built for;
    // then width, then placement. Only when NOTHING is applied is a flat
    // curve the honest answer.
    int src = 3;
    double depth = 1.0;
    if (mechanism_on(v, kRangeDb)) {
        src = 0;
        depth = v.range_db.load(std::memory_order_relaxed);
    } else if (width_db > 1e-6) {
        src = 1;
        depth = width_db;
    } else if (pan_amt > 1e-6) {
        src = 2;
        depth = 100.0 * pan_amt;   // placement reads as a percentage
    }
    // With Band Q = Flat the bands add and are NOT corrected, so the true
    // response reaches 1 + K*(g-1) rather than g -- about 44 dB when the
    // knob says 18 at 128 nodes. Now that the curve draws the true response
    // (O10), an axis labelled from the knob alone would leave it pegged at
    // the clamp, and the default configuration would look broken rather
    // than merely BE wrong. So the axis is widened by the same factor the
    // bank applies. It is a static quantity -- it moves only with Range,
    // node count and Q -- so the axis does not jitter with the string.
    //
    // In Matched mode the correction has already divided K out, so this is
    // exactly the knob value and the axis is unchanged from before. The
    // widening in Flat mode IS the defect, made legible: you asked for 18
    // and the axis says 44.
    // The axis widens by whatever the bank is NOT correcting. At Band Q 0
    // that is the full K; at 1 it is nothing and the axis equals the knob.
    if (src != 2 && n_nodes_ > 1) {
        const double resid = v.bank.overlap_k_residual();
        if (resid > 1.0001) {
            const double g = std::pow(10.0, depth / 20.0);
            const double act = 1.0 + resid * (g - 1.0);
            if (act > 1.0) depth = 20.0 * std::log10(act);
        }
    }
    v.display_src.store(src, std::memory_order_relaxed);
    v.display_depth.store(depth, std::memory_order_relaxed);
    if (out.spatial_on) {
        if (v.side_targets.size() != (std::size_t)n_nodes_) {
            v.side_targets.assign(n_nodes_, 1.0);
            v.pan_targets.assign(n_nodes_, 0.0);
        }
        if (v.taper_lo != (double)p.stereo_lo->get()
            || v.taper_hi != (double)p.stereo_hi->get()
            || (int)v.spatial_w.size() != n_nodes_)
            rebuild_spatial_taper(v, p);
    }
    // Targets are re-aimed per CHUNK inside the audio loop below, not once
    // per block -- see kGainChunk.
}

void CurvesAudioProcessor::processBlock(juce::AudioBuffer<float>& buffer,
                                        juce::MidiBuffer&) {
    juce::ScopedNoDenormals noDenormals;

    // 0. Precision mode: rebuild the sim/bank tuple for EVERY voice when it
    //    changes. This must run BEFORE the null check -- prepareToPlay
    //    leaves the sims null and the first block creates them here
    //    (early-returning first = permanent silence, which is exactly what
    //    happened at first M2 deploy).
    const int prec = static_cast<int>(precision_->get());
    if (prec != last_precision_) {
        rebuild(prec);
        last_precision_ = prec;
    }
    if (!voices_[0].sim) {
        buffer.clear();
        return;
    }

    const int n = buffer.getNumSamples();
    float* ch0 = buffer.getWritePointer(0);
    float* ch1 = buffer.getNumChannels() > 1 ? buffer.getWritePointer(1) : nullptr;
    const bool stereo = ch1 != nullptr;
    // Dual needs two output channels to mean anything, so on a mono bus it
    // falls back to one voice rather than ticking a second sim into nothing.
    const bool dual = dual_ != nullptr && dual_->get() > 0.5f && stereo;
    const int nv = dual ? kNumVoices : 1;
    active_voices_.store(nv, std::memory_order_relaxed);
    const int prec_idx = std::clamp(prec, 0, 2);

    // 1. GUI is a writer: apply the latest draw deposit (held string). It
    //    goes to the voice the canvas is SHOWING -- drawing on screen has to
    //    affect the string you can see.
    const uint32_t dv = draw_version_.load(std::memory_order_acquire);
    if (dv != last_draw_version_) {
        const juce::SpinLock::ScopedLockType lk(draw_lock_);
        std::vector<double> vals(draw_count_);
        for (int i = 0; i < draw_count_; ++i)
            vals[i] = static_cast<double>(draw_buffer_[i]);
        Voice& target = voices_[juce::jlimit(0, nv - 1, ui_voice_)];
        if (target.sim)
            target.sim->displace(vals);
        last_draw_version_ = dv;
    }

    // 2. Global knobs. Their modifiers live on voice 0 because these knobs
    //    are not voice-paged -- there is one Mix and one Leveler for the
    //    whole plugin, after both voices are summed into the output.
    leveler_.set_amount(knob(voices_[0], kLeveler, leveler_amt_->get()));
    mix_amount_ = knob(voices_[0], kMix, mix_->get());

    // 3. Per-voice parameter mapping.
    VoiceBlock vb[kNumVoices];
    for (int vi = 0; vi < nv; ++vi)
        prepare_voice(voices_[vi], vp_[vi], prec_idx, vb[vi]);

    // 3b. Reset request from the UI: flatten EVERY active string, touch
    //     nothing else. Wind, flutter and every knob stay exactly as they
    //     are, so the cloth starts moving again immediately from rest.
    if (reset_request_.exchange(false, std::memory_order_acquire)) {
        std::vector<double> flat(n_nodes_, 0.0);
        for (int vi = 0; vi < nv; ++vi)
            if (voices_[vi].sim)
                voices_[vi].sim->displace(flat);
    }

    // 4. Audio: tick the sim on its divisor grid, process per sample.
    //    When the internal test source is on, broadband pink noise replaces
    //    the input — the standalone then demos the string with no wiring.
    const int test_src =
        juce::jlimit(0, (int)kNumTestSrc - 1,
                     static_cast<int>(std::lround(test_src_->get())));
    const bool test_on = test_src != kTestOff;
    // Re-aim the gain ramp every kGainChunk samples rather than once per
    // host block.
    //
    // With one aim per block, the target was computed from the control
    // signal as it stood at the START of the block and the bank only
    // reached it at the END -- so the pipeline carried roughly a block and
    // a half of lag ON TOP OF the Smoothing knob, and that lag grew with
    // the host's buffer size. Measured max|sim - drawn| at Smoothing 2 ms
    // ("off"): 0.0164 at a 32-sample buffer rising to 0.0771 at 512, i.e.
    // most of the smoothing you could hear at a big buffer was not the knob
    // (user: "a hell of a lot of smoothing going on, even when smoothing is
    // off"). Worse, it made the plugin sound different at different buffer
    // sizes.
    //
    // A fixed chunk decouples it: one exp2 per node and one pow per band
    // per chunk, which at 32 samples is 1500 chunks/s -- cheap, and the
    // behaviour no longer depends on the host.
    constexpr int kGainChunk = 32;
    for (int base = 0; base < n; base += kGainChunk) {
      const int len = std::min(kGainChunk, n - base);
      for (int vi = 0; vi < nv; ++vi) {
      Voice& v = voices_[vi];
      const double range_db = vb[vi].range_db;
      const bool spatial_on = vb[vi].spatial_on;
      const double width_db = vb[vi].width_db;
      const double pan_amt = vb[vi].pan_amt;
      // Targets from the control signal as it stands RIGHT NOW. disp was
      // already smoothed at the tick rate in its own (log) domain, so Range
      // enters exactly once and memorylessly.
      for (int i = 0; i < n_nodes_; ++i) {
          const double d = std::clamp(v.smoothed_disp[i], -1.0, 1.0);
          const double db = d * range_db;
          // 10^(db/20) == 2^(db/6.0206); exp2 is markedly cheaper than pow.
          v.gain_targets[i] = std::exp2(db * (1.0 / 6.020599913279624));
          if (spatial_on) {
              // Width is a DEPTH ON THE SIDE, in the same dB unit as Range:
              // the side gain is the mid gain plus Width dB of the same
              // shape, so the width ratio is 2^(d*Width/6.0206). EQ and
              // spatial read ONE curve and differ only in how hard they
              // drive it -- which is what keeps a single drawn curve honest
              // for both.
              const double w = v.spatial_w[i];
              v.side_targets[i] = std::exp2((db + d * width_db * w)
                                           * (1.0 / 6.020599913279624));
              v.pan_targets[i] = d * pan_amt * w;
          }
      }
      v.bank.set_block_targets(v.gain_targets.data(), len);
      if (spatial_on)
          v.bank.set_spatial_targets(v.side_targets.data(), v.pan_targets.data(), len);
      else
          v.bank.set_spatial_off();
      }
      for (int s = base; s < base + len; ++s) {
        // Every active voice advances on the same divisor grid.
        for (int vi = 0; vi < nv; ++vi) {
        Voice& v = voices_[vi];
        if (++v.sim_counter >= sim_divisor_) {
            v.sim->tick();
            v.sim_counter = 0;
            // The string has new information: fold it into the smoothed
            // control signal. This is the only smoothing in the plugin.
            const auto& u = v.sim->positions();
            const auto& vel = v.sim->velocities();
            const double a = v.smooth_alpha, va = v.vel_amt, vs = v.vel_scale;
            if (v.detail_amt <= 0.0) {
                for (int i = 0; i < n_nodes_; ++i) {
                    const double disp = (1.0 - va) * u[i] + va * (vs * vel[i]);
                    v.smoothed_disp[i] += a * (disp - v.smoothed_disp[i]);
                }
            } else {
                // Split by spatial scale, then smooth each part on its own
                // time constant. The split is a bidirectional one-pole
                // across the NODES -- zero phase shift, O(N), any cutoff.
                const double b = v.detail_b;
                // 0.97 => the fine part follows ~33x slower than the broad part at
                // full. 0.94 (16x) measured only a 2.4x drop in detail rms,
                // which reads as subtle for a dedicated control.
                const double ad = a * (1.0 - 0.97 * v.detail_amt);
                for (int i = 0; i < n_nodes_; ++i)
                    v.disp_tmp[i] = (1.0 - va) * u[i] + va * (vs * vel[i]);
                double acc = v.disp_tmp[0];
                for (int i = 0; i < n_nodes_; ++i) {
                    acc += b * (v.disp_tmp[i] - acc);
                    v.blur_tmp[i] = acc;
                }
                acc = v.blur_tmp[n_nodes_ - 1];
                for (int i = n_nodes_ - 1; i >= 0; --i) {
                    acc += b * (v.blur_tmp[i] - acc);
                    v.blur_tmp[i] = acc;          // the BROAD component
                }
                double d2 = 0.0;
                for (int i = 0; i < n_nodes_; ++i) {
                    const double broad = v.blur_tmp[i];
                    const double det = v.disp_tmp[i] - broad;
                    v.sm_broad[i] += a * (broad - v.sm_broad[i]);
                    v.sm_detail[i] += ad * (det - v.sm_detail[i]);
                    v.smoothed_disp[i] = v.sm_broad[i] + v.sm_detail[i];
                    d2 += v.sm_detail[i] * v.sm_detail[i];
                }
                v.ctl_det2.store(v.ctl_det2.load(std::memory_order_relaxed) + d2,
                                 std::memory_order_relaxed);
            }
            // Track what the control signal is actually reaching, so O1 can
            // be measured rather than modelled.
            double pk = v.ctl_peak.load(std::memory_order_relaxed);
            double s2 = 0.0, f2 = 0.0;
            for (int i = 0; i < n_nodes_; ++i) {
                const double a2 = v.smoothed_disp[i] * v.smoothed_disp[i];
                s2 += a2;
                if (i > 0) {
                    const double d = v.smoothed_disp[i] - v.smoothed_disp[i - 1];
                    f2 += d * d;
                }
                pk = std::max(pk, std::abs(v.smoothed_disp[i]));
            }
            v.ctl_fine2.store(v.ctl_fine2.load(std::memory_order_relaxed) + f2,
                              std::memory_order_relaxed);
            v.ctl_peak.store(pk, std::memory_order_relaxed);
            v.ctl_sum2.store(v.ctl_sum2.load(std::memory_order_relaxed) + s2,
                             std::memory_order_relaxed);
            v.ctl_count.store(v.ctl_count.load(std::memory_order_relaxed)
                                  + double(n_nodes_),
                              std::memory_order_relaxed);
        }
        }
        if (test_on) {
            ch0[s] = test_sample(test_src, 0);
            if (ch1)
                ch1[s] = test_sample(test_src, 1);
        }
        const float dry_l = ch0[s];
        const float dry_r = stereo ? ch1[s] : dry_l;
        float l = dry_l;
        float r = dry_r;
        if (dual) {
            // DUAL: each voice sees ONE channel, as mono. Its side signal
            // is therefore zero by construction, which makes Width and
            // Place inert per voice -- the stereo comes from the two
            // strings diverging, not from one string's mid/side. The editor
            // dims those two knobs to say so.
            float a0 = dry_l, b0 = dry_l;
            voices_[0].bank.process_frame(a0, b0, false);
            float a1 = dry_r, b1 = dry_r;
            voices_[1].bank.process_frame(a1, b1, false);
            l = a0;
            r = a1;
        } else {
            voices_[0].bank.process_frame(l, r, stereo);
        }
        if (mix_amount_ < 1.0) {
            const float m = static_cast<float>(mix_amount_);
            l = dry_l + m * (l - dry_l);
            r = dry_r + m * (r - dry_r);
        }
        leveler_.process(l, r);
        ch0[s] = l;
        if (ch1)
            ch1[s] = r;
      }
    }

    // 5. GUI is a reader: publish positions for display, throttled to ~60 Hz.
    //    The editor repaints at 30 Hz, so publishing every block was ~6x the
    //    lock traffic and ~6x the per-node log10 work for no visible gain.
    display_accum_ += buffer.getNumSamples();
    const int publish_interval = static_cast<int>(fs_ / 60.0);
    if (display_accum_ >= publish_interval) {
        display_accum_ = 0;
        publish_display();
    }
}

// Publishes the voice the editor is SHOWING. Only one curve is drawn at a
// time -- which is the point of the L/R highlight: you look at one string's
// complete picture, knobs and curve together.
void CurvesAudioProcessor::publish_display() {
    // The audio thread must NEVER block on the GUI thread. The editor holds
    // this same spinlock while it copies the snapshot out, and a spinlock
    // owned by a lower-priority thread is a textbook priority inversion:
    // the audio thread burns its deadline spinning. Try, and bail if the
    // editor has it - dropping a frame is invisible at 30 Hz, a dropout is
    // not. (Diagnosed alongside the high-precision glitch report.)
    if (!display_lock_.tryEnter())
        return;
    const int nv = active_voices_.load(std::memory_order_relaxed);
    for (int vi = 0; vi < kNumVoices; ++vi) {
    Voice& v = voices_[vi];
    // A voice that is not running keeps its LAST frame rather than being
    // cleared: a canvas that blanks when Dual is switched off reads as a
    // bug, and the frame it is holding is the last true one.
    if (vi >= nv || !v.sim)
        continue;
    const auto& u = v.sim->positions();
    for (int i = 0; i < n_nodes_; ++i)
        display_buffer_[vi][i] = static_cast<float>(u[i]);
    // The EQ curve of truth: the bank's smoothed per-band gains (post
    // smoothing, post Steady), normalized so +1 = Max gain dB.
    //
    // Mix is folded in EXACTLY, not approximately. The bank emits
    // x + sum(v1_b * big_b) and Mix scales only the added part, so the
    // effective per-band gain is 1 + Mix*(g - 1). Without this the curve
    // drew the full EQ depth while Mix 0.5 applied half of it -- the same
    // class of WYSIWYG violation Steady had, introduced when Mix was added.
    const int src = v.display_src.load(std::memory_order_relaxed);
    const double depth = v.display_depth.load(std::memory_order_relaxed);
    const double mix = mix_amount_;
    // O10. The curve is the bank's TRUE summed response, not the nominal
    // per-band gain. The bands overlap and add, so the nominal value never
    // was the response -- measured 31 dB out at 128 nodes and 43 dB at 512.
    // response_curve() convolves the per-band excess with the bank's own
    // kernel, which is exact and costs 12 us at 128 nodes / 193 us at 512
    // per publish (0.07% / 1.17% of a core at the 60 Hz publish rate).
    // Verified against the measured magnitude response to 0.12 dB worst
    // case across every Band Q / Resonance setting -- q_test section J.
    if (src == 2) {
        // Placement is a mid->side CROSS-FEED, not a gain, so there is no
        // unity term. The in-phase part is the coherent leak and keeps the
        // sign the display needs; the quadrature part is a 90-degree-shifted
        // leak and does not read as position.
        const auto& p = v.bank.pans();
        v.bank.response_curve(p.data(), mix, v.resp_re.data(), v.resp_im.data());
        const double amt = std::max(1e-9, depth * 0.01);
        for (int i = 0; i < n_nodes_; ++i)
            display_eq_buffer_[vi][i] =
                static_cast<float>(std::clamp(v.resp_re[i] / amt, -1.0, 1.0));
    } else {
        // Mix scales only the added part, so it folds into the convolution
        // exactly -- the sum is linear in the per-band excess.
        const auto& g = (src == 1) ? v.bank.side_gains() : v.bank.gains();
        for (int i = 0; i < n_nodes_; ++i) v.resp_big[i] = g[i] - 1.0;
        v.bank.response_curve(v.resp_big.data(), mix, v.resp_re.data(),
                             v.resp_im.data());
        for (int i = 0; i < n_nodes_; ++i) {
            const double re = 1.0 + v.resp_re[i], im = v.resp_im[i];
            const double mag2 = std::max(re * re + im * im, 1e-12);
            const double db = 10.0 * std::log10(mag2);
            display_eq_buffer_[vi][i] =
                static_cast<float>(std::clamp(db / std::max(1.0, depth), -1.0, 1.0));
        }
    }
    }
    display_count_ = n_nodes_;
    display_version_.fetch_add(1, std::memory_order_release);
    display_lock_.exit();
}

void CurvesAudioProcessor::push_draw(const std::array<float, kMaxNodes>& nodes,
                                     int count) {
    {
        const juce::SpinLock::ScopedLockType lk(draw_lock_);
        draw_buffer_ = nodes;
        draw_count_ = std::clamp(count, 4, kMaxNodes);
    }
    draw_version_.fetch_add(1, std::memory_order_release);
}

void CurvesAudioProcessor::push_clear() {
    std::array<float, kMaxNodes> zeros{};
    push_draw(zeros, n_nodes_);
}

void CurvesAudioProcessor::pull_display(int voice,
                                        std::array<float, kMaxNodes>& out_pos,
                                        std::array<float, kMaxNodes>& out_eq,
                                        int& count) const {
    const int vi = juce::jlimit(0, kNumVoices - 1, voice);
    const juce::SpinLock::ScopedLockType lk(display_lock_);
    out_pos = display_buffer_[vi];
    out_eq = display_eq_buffer_[vi];
    count = display_count_;
}

void CurvesAudioProcessor::getStateInformation(juce::MemoryBlock& dest) {
    juce::ValueTree tree("CurvesState");
    for (auto* p : getParameters())
        if (auto* v = dynamic_cast<juce::RangedAudioParameter*>(p))
            tree.setProperty(juce::Identifier(v->paramID), v->getValue(), nullptr);
    tree.setProperty("uiTheme", ui_theme_, nullptr);
    tree.setProperty("uiVoice", ui_voice_, nullptr);
    // Per voice. Voice 0 keeps the original property names so a session
    // saved before Dual existed still restores its modifiers and seeds;
    // voice 1 gets suffixed ones, and their absence just leaves it at
    // defaults.
    for (int v = 0; v < kNumVoices; ++v) {
        const juce::String sfx = v == 0 ? juce::String() : juce::String("R");
        juce::String mods;
        for (int k = 0; k < kNumModKnobs; ++k)
            mods << (voices_[v].mods[k].on ? "1" : "0")
                 << juce::String(voices_[v].mods[k].mag);
        tree.setProperty("knobMods" + sfx, mods, nullptr);
        tree.setProperty("flutterSeed" + sfx, (int)voices_[v].flutter_seed, nullptr);
        tree.setProperty("windSeed" + sfx, (int)voices_[v].wind_seed, nullptr);
    }
    if (auto xml = tree.createXml())
        copyXmlToBinary(*xml, dest);
}

void CurvesAudioProcessor::setStateInformation(const void* data, int size) {
    if (auto xml = getXmlFromBinary(data, size))
        if (auto tree = juce::ValueTree::fromXml(*xml); tree.isValid()) {
            if (tree.hasProperty("uiTheme"))
                ui_theme_ = (int)tree.getProperty("uiTheme");
            if (tree.hasProperty("uiVoice"))
                set_ui_voice((int)tree.getProperty("uiVoice"));
            for (int v = 0; v < kNumVoices; ++v) {
                const juce::String sfx = v == 0 ? juce::String() : juce::String("R");
                if (tree.hasProperty("flutterSeed" + sfx))
                    voices_[v].flutter_seed =
                        (int)tree.getProperty("flutterSeed" + sfx);
                if (tree.hasProperty("windSeed" + sfx))
                    voices_[v].wind_seed = (int)tree.getProperty("windSeed" + sfx);
                if (tree.hasProperty("knobMods" + sfx)) {
                    const juce::String m =
                        tree.getProperty("knobMods" + sfx).toString();
                    for (int k = 0; k < kNumModKnobs && 2 * k + 1 < m.length(); ++k) {
                        voices_[v].mods[k].on = (m[2 * k] == '1');
                        voices_[v].mods[k].mag =
                            juce::jlimit(0, 2, static_cast<int>(m[2 * k + 1]) - '0');
                    }
                }
            }
            for (auto* p : getParameters())
                if (auto* v = dynamic_cast<juce::RangedAudioParameter*>(p))
                    if (tree.hasProperty(juce::Identifier(v->paramID)))
                        v->setValueNotifyingHost(tree.getProperty(juce::Identifier(v->paramID)));
        }
}

juce::AudioProcessorEditor* CurvesAudioProcessor::createEditor() {
    return new CurvesAudioProcessorEditor(*this);
}

// Paul Kellet's economical pink-noise approximation (~ -3 dB/oct), ±1 range
// scaled to a sane listening level.
const char* CurvesAudioProcessor::test_src_name(int i) {
    static const char* kNames[kNumTestSrc] = {"off", "pink", "white",
                                              "brown", "tone", "pulse"};
    return kNames[juce::jlimit(0, (int)kNumTestSrc - 1, i)];
}

float CurvesAudioProcessor::test_sample(int src, int ch) {
    switch (src) {
        case kTestWhite:
            return (random_.nextFloat() * 2.0f - 1.0f) * 0.35f;
        case kTestBrown: {
            // leaky integrator: -6 dB/oct, with the leak keeping it centred
            const float w = random_.nextFloat() * 2.0f - 1.0f;
            brown_[ch] = 0.996f * brown_[ch] + 0.04f * w;
            // 0.70, not an eyeballed number: a leaky integrator with a=0.996
            // driven by 0.04*uniform has sigma = 0.2585, so 3.2 would have
            // been rms 0.81 / peak 3.97 -- four times full scale. All five
            // textures are matched to rms ~0.18-0.20 (pulse lower by design,
            // it is sparse; its PEAK is what matters).
            return brown_[ch] * 0.70f;
        }
        case kTestTone: {
            // Sustained harmonic stack near A1, amplitudes 1/n so the
            // spectrum is sawtooth-like and every band has something in it.
            // The two channels are detuned slightly so stereo is not a
            // perfect mono sum.
            const double f0 = (ch == 0) ? 55.0 : 55.37;
            tone_phase_[ch] += f0 / fs_;
            if (tone_phase_[ch] >= 1.0)
                tone_phase_[ch] -= 1.0;
            double acc = 0.0;
            const int kHarm = 24;
            for (int n = 1; n <= kHarm; ++n) {
                if (f0 * n > 0.45 * fs_)
                    break;
                acc += std::sin(6.283185307179586 * tone_phase_[ch] * n) / n;
            }
            return static_cast<float>(acc * 0.22);
        }
        case kTestPulse: {
            // ~7 clicks/sec, each a short decaying burst so it excites the
            // whole band without being a raw unit impulse.
            const int period = std::max(1, static_cast<int>(fs_ / 7.0));
            if (++pulse_count_[ch] >= period) {
                pulse_count_[ch] = 0;
                pulse_env_[ch] = 1.0f;
            }
            pulse_env_[ch] *= 0.9993f;
            const float w = random_.nextFloat() * 2.0f - 1.0f;
            return w * pulse_env_[ch] * pulse_env_[ch] * 0.8f;
        }
        case kTestPink:
        default:
            return pink_noise(ch);
    }
}

float CurvesAudioProcessor::pink_noise(int channel) {
    auto& b = pink_[channel];
    const float white = random_.nextFloat() * 2.0f - 1.0f;
    b[0] = 0.99886f * b[0] + white * 0.0555179f;
    b[1] = 0.99332f * b[1] + white * 0.0750759f;
    b[2] = 0.96900f * b[2] + white * 0.1538520f;
    b[3] = 0.86650f * b[3] + white * 0.3104856f;
    b[4] = 0.55000f * b[4] + white * 0.5329522f;
    b[5] = -0.7616f * b[5] - white * 0.0168980f;
    const float pink = (b[0] + b[1] + b[2] + b[3] + b[4] + b[5] + b[6]
                        + white * 0.5362f) * 0.05f;
    b[6] = white * 0.115926f;
    return pink;
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter() {
    return new CurvesAudioProcessor();
}
