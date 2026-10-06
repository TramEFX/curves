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
constexpr double kMidTravelLo = 0.55, kMidTravelHi = 1.25;
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
        "flutter_hz", "wrinkle", "mix", "flap", "width", "pan_depth"};
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

CurvesAudioProcessor::CurvesAudioProcessor()
    : AudioProcessor(BusesProperties()
                         .withInput("Input", juce::AudioChannelSet::stereo(), true)
                         .withOutput("Output", juce::AudioChannelSet::stereo(), true)) {
    using APF = juce::AudioParameterFloat;
    using R = juce::NormalisableRange<float>;
    // Dial ranges calibrated by feel-testing (user): meaningful damping by
    // 45% of the old 0-40 dial -> max 18; meaningful force by 25% of the old
    // dial -> drive/ceiling compressed x0.25 (see processBlock).
    addParameter(damping_ = new APF("damping", "Damping", R{0.05f, 18.0f}, 1.5f));
    addParameter(stiffness_ = new APF("stiffness", "Stiffness", R{0.0f, 20.0f}, 4.0f));
    addParameter(tension_ = new APF("tension", "Tension", R{50.0f, 600.0f}, 300.0f));
    // 30 -> 15 ms: the smoother used to advance once PER CHANNEL, so a stereo
    // instance ran at half the set time. Fixing that doubled the real
    // smoothing at any given setting; 15 ms restores the previous feel.
    addParameter(smoothing_ = new APF("smoothing", "Smoothing ms", R{2.0f, 100.0f}, 15.0f));
    addParameter(drive_ = new APF("drive", "Max gain dB", R{3.0f, 24.0f}, 18.0f));
    // Dry/wet. 1 = fully processed, 0 = bypass (so power-off = its minimum
    // is genuinely neutral). Applied after the bank and BEFORE the leveler,
    // so the leveler trims what you actually hear.
    addParameter(mix_ = new APF("mix", "Mix", R{0.0f, 1.0f}, 1.0f));
    addParameter(steady_ = new APF("steady", "Steady (per-band)", R{0.0f, 1.0f}, 0.35f));
    addParameter(leveler_amt_ = new APF("leveler", "Leveler (output)", R{0.0f, 1.0f}, 0.5f));
    addParameter(test_src_ = new APF("test", "Test source",
                                     R(0.0f, 5.0f, 1.0f), 0.0f));
    addParameter(physics_ = new APF("physics", "Physics", R(0.0f, 3.0f, 1.0f), 0.0f));
    addParameter(precision_ = new APF("precision", "Precision", R(0.0f, 2.0f, 1.0f), 0.0f));
    addParameter(velocity_ = new APF("velocity", "Velocity", R{0.0f, 1.0f}, 0.0f));
    addParameter(wind_ = new APF("wind", "Wind", R{0.0f, 1.0f}, 0.50f));
    addParameter(waves_ = new APF("waves", "Waves", R{0.0f, 1.0f}, 0.24f));
    // Motion = total string excitation, calibrated so peak displacement
    // tracks the dial linearly regardless of Character. Default chosen to
    // match the previous Force 0.6 wind-only feel (peak ~0.37).
    addParameter(motion_ = new APF("motion", "Motion", R{0.0f, 1.0f}, 0.45f));
    // Character = where that excitation comes from. 0 = all gust wind,
    // 1 = all self-sustaining vortex shedding. Default 0 reproduces the
    // previous behaviour exactly; raising it is a tuning decision.
    addParameter(character_ = new APF("character", "Character", R{0.0f, 1.0f}, 0.0f));
    addParameter(direction_ = new APF("direction", "Direction", R(0.0f, 1.0f, 1.0f), 0.0f));
    // Walls replace the old Cloth-length knob. Two independent boundaries,
    // dragged on the canvas, so any window anywhere along the spectrum can be
    // the active cloth. Length could only ever shorten from the free end and,
    // worse, did not actually work: measured, Length 0.6 still left 99% of
    // the string moving, because the "near-rigid tail" was not a hard wall.
    addParameter(wall_lo_ = new APF("wall_lo", "Wall Low", R{0.0f, 1.0f}, 0.0f));
    addParameter(wall_hi_ = new APF("wall_hi", "Wall High", R{0.0f, 1.0f}, 1.0f));
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
    addParameter(flutter_hz_ = new APF("flutter_hz", "Flutter Hz", R{0.1f, 20.0f}, 3.0f));
    // Wrinkle: tip-localized fine detail - "how the last third resolves or
    // flaps". 0 = off, so the stage costs nothing until dialled in.
    addParameter(wrinkle_ = new APF("wrinkle", "Wrinkle", R{0.0f, 1.0f}, 0.0f));
    // Flap: how hard the mid-band ripples drive the FREE END. The bump
    // envelope is zero there by construction, so without this the ripples
    // fade out over the last fifth -- they were never forced, not damped.
    addParameter(flap_ = new APF("flap", "Flap", R{0.0f, 1.0f}, 0.45f));
    // Randomisers. 0 = the fixed constant sets, i.e. exactly the regular
    // behaviour; turning one up perturbs those constants from its seed.
    addParameter(wind_rnd_ = new APF("wind_rnd", "Wind Random",
                                     R{0.0f, 1.0f}, 0.0f));
    addParameter(flutter_rnd_ = new APF("flutter_rnd", "Flutter Random",
                                        R{0.0f, 1.0f}, 0.0f));
    // ---- SPATIAL subscriber. Neutral at zero, so it costs nothing and
    // changes nothing until it is turned up.
    addParameter(width_ = new APF("width", "Width dB", R{0.0f, 24.0f}, 0.0f));
    addParameter(pan_depth_ = new APF("pan_depth", "Placement", R{0.0f, 1.0f}, 0.0f));
    addParameter(stereo_lo_ = new APF("stereo_lo", "Spatial low cut",
                                      R{20.0f, 2000.0f}, 120.0f));
    addParameter(stereo_hi_ = new APF("stereo_hi", "Spatial high cut",
                                      R{1000.0f, 18000.0f}, 12000.0f));

    // Capture each modifiable knob's minimum; "off" resolves to it.
    for (int k = 0; k < kNumModKnobs; ++k) {
        knob_mods_[k].lo = 0.0;
        for (auto* p : getParameters())
            if (auto* rp = dynamic_cast<juce::RangedAudioParameter*>(p))
                if (rp->paramID == juce::String(mod_knob_id(k)))
                    knob_mods_[k].lo = rp->getNormalisableRange().start;
    }
}

void CurvesAudioProcessor::prepareToPlay(double sampleRate, int) {
    fs_ = sampleRate;
    leveler_.prepare(fs_);
    last_precision_ = -1;  // processBlock rebuilds with the current setting
    sim_ = nullptr;
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

    curves::StringParams p;
    p.tick_rate = fs_ / sim_divisor_;
    p.wave_speed = tension_->get();
    p.damping = damping_->get();
    p.stiffness = stiffness_->get();
    sim_ = std::make_unique<curves::StringSim>(n_nodes_, p);
    sim_counter_ = 0;

    bank_.prepare(n_nodes_, fs_, 2);
    bank_.set_steady(knob(kSteady, steady_->get()), 0.25);
    rebuild_band_map();
    gain_targets_.assign(n_nodes_, 1.0);
    smoothed_disp_.fill(0.0);
    last_physics_ = -1;  // force re-apply on the current preset
}

void CurvesAudioProcessor::apply_physics(int mode) {
    if (!sim_)
        return;
    sim_->anchor_low = false;
    sim_->anchor_high = false;
    sim_->force_model = curves::StringSim::kStaticField;
    sim_->set_density_taper(0.0);
    std::vector<double> field(n_nodes_, 0.0);

    if (mode == kFlagpole) {
        // Cloth wind: the pole (high end) is anchored; the LEFT END IS FREE
        // — it moves and whips (user decision: pinning it kills the whip).
        // Cloth length sets where the animation terminates; beyond it the
        // wind is zero and damping is multiplied (termination behavior).
        sim_->anchor_high = true;
        // Aero model: the wind is a VELOCITY field and the force comes from
        // relative velocity, so the string fights the wind and flap
        // amplitude is bounded by drag instead of by a limiter. Phase speed
        // scales with node count so Wind means the same thing in every
        // precision mode (at a fixed nodes/s the pattern would traverse a
        // 1024-node string 8x slower than a 128-node one).
        sim_->set_cloth_aero(192.0 * n_nodes_ / 128.0,
                             static_cast<double>(n_nodes_), 1.5);
        sim_->aero_rho_cd = 0.6;
        sim_->set_density_taper(2.0);
        // Walls own the active extent now, so cloth_length stays at 1 and the
        // walls are the single owner of the geometry. (The old Cloth-length
        // knob both desynced across apply_physics and did not actually hold:
        // measured, length 0.6 still left 99% of the string moving.)
        sim_->set_cloth_length(1.0);
        last_wall_lo_ = -1.0;   // force the walls to re-apply to the new sim
    } else if (mode == kGust) {
        // No anchors; smooth zero-mean static force field across the spectrum.
        const double w = 8.0 * (static_cast<double>(n_nodes_) / 128.0);
        for (int i = 0; i < n_nodes_; ++i)
            field[i] = -w * std::sin(3.14159265358979323846 * i / (n_nodes_ - 1));
    }
    // kFree / kElastic: no anchors, no forces — knobs do the talking.
    sim_->set_force_field(field);
}

// Damps the SPATIAL deviation outside [lo, hi], leaving the EQ alone.
// Widening the bass is almost always wrong, and the log axis puts bass at
// the left end, so a flat width depth misbehaves down there. This is a
// taper on the DEVIATION, not a filter on the audio: outside the window
// width goes to 1 and pan to 0, i.e. those bands are simply left alone.
// One octave of smoothstep at each edge keeps the drawn curve continuous.
void CurvesAudioProcessor::rebuild_spatial_taper() {
    spatial_w_.assign(n_nodes_, 1.0);
    const double lo = std::max(1.0, (double)stereo_lo_->get());
    const double hi = std::max(lo * 2.0, (double)stereo_hi_->get());
    const double l2lo = std::log2(lo), l2hi = std::log2(hi);
    auto smooth = [](double t) {
        t = std::clamp(t, 0.0, 1.0);
        return t * t * (3.0 - 2.0 * t);
    };
    for (int i = 0; i < n_nodes_; ++i) {
        const double f = std::max(1.0, band_fc_[i]);
        const double l2 = std::log2(f);
        spatial_w_[i] = smooth(l2 - l2lo) * smooth(l2hi - l2);
    }
    taper_lo_ = lo;
    taper_hi_ = hi;
}

void CurvesAudioProcessor::rebuild_band_map() {
    band_fc_.resize(n_nodes_);
    for (int i = 0; i < n_nodes_; ++i) {
        // Clamp so the top band never crosses Nyquist.
        band_fc_[i] = std::min(node_frequency(i, n_nodes_, kFMin, kFMax), 0.45 * fs_);
        bank_.set_band_frequency(i, band_fc_[i]);
    }
}

void CurvesAudioProcessor::processBlock(juce::AudioBuffer<float>& buffer,
                                        juce::MidiBuffer&) {
    juce::ScopedNoDenormals noDenormals;

    // 0. Precision mode: rebuild sim/bank tuple when it changes. This must
    //    run BEFORE the null check — prepareToPlay leaves sim_ null and the
    //    first block creates it here (early-returning first = permanent
    //    silence, which is exactly what happened at first M2 deploy).
    const int prec = static_cast<int>(precision_->get());
    if (prec != last_precision_) {
        rebuild(prec);
        last_precision_ = prec;
    }
    if (!sim_) {
        buffer.clear();
        return;
    }

    // Physics preset: anchors + force field when it changes.
    const int phys = static_cast<int>(physics_->get());
    if (phys != last_physics_) {
        apply_physics(phys);
        last_physics_ = phys;
    }

    // 1. GUI is a writer: apply the latest draw deposit (held string).
    const uint32_t dv = draw_version_.load(std::memory_order_acquire);
    if (dv != last_draw_version_) {
        const juce::SpinLock::ScopedLockType lk(draw_lock_);
        std::vector<double> vals(draw_count_);
        for (int i = 0; i < draw_count_; ++i)
            vals[i] = static_cast<double>(draw_buffer_[i]);
        sim_->displace(vals);
        last_draw_version_ = dv;
    }

    // 2. Live physics parameters (cheap, per block).
    sim_->set_damping(knob(kDamping, damping_->get()));
    sim_->set_stiffness(knob(kStiffness, stiffness_->get()));
    sim_->set_wave_speed(knob(kTension, tension_->get()));
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
        const double wind_knob = knob(kWind, wind_->get());
        const double ratio = 0.12 * std::pow(25.0, wind_knob);
        const double c = std::max(1.0, knob(kTension, tension_->get()));
        sim_->wind_phase_speed = ratio * c;
        // Mid-band ripples ride the same knob: Wind picks their SIZE, 6 -> 20
        // waves across the cloth. Amount follows Motion (via wind_speed), so
        // there is no extra control -- measured, the gust sites cover only
        // 1.7-6 waves and wrinkle 25-64, and nothing reached the middle.
        sim_->mid_waves = 6.0 * std::pow(20.0 / 6.0, wind_knob);
        sim_->mid_lock_d = kMidLockD;
        sim_->mid_env = kMidEnv;
        sim_->mid_flap = knob(kFlap, flap_->get());
        sim_->gust_spread = wind_rnd_->get();
        sim_->gust_seed = wind_seed_.load(std::memory_order_relaxed);
        // Wind also sets how fast the ripples RUN DOWN the string, with the
        // detuning loss compensated so it stays a speed control.
        const double travel =
            kMidTravelLo * std::pow(kMidTravelHi / kMidTravelLo, wind_knob);
        sim_->mid_travel = travel;
        sim_->mid_ripple =
            mechanism_on(kWind) ? kMidAmp * mid_resonance_compensation(travel) : 0.0;
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
    sim_->wave_density = 0.5 + 7.5 * knob(kWaves, waves_->get());
    sim_->set_wind_origin(direction_->get() > 0.5f ? 1 : 0);
    // Walls: only rebuild the geometry when they actually move.
    const double wlo = wall_lo_->get(), whi = wall_hi_->get();
    if (std::abs(wlo - last_wall_lo_) > 1e-4 || std::abs(whi - last_wall_hi_) > 1e-4) {
        sim_->set_walls(wlo, whi);
        last_wall_lo_ = wlo;
        last_wall_hi_ = whi;
    }
    // --- excitation budget --------------------------------------------
    // One amount control (Motion) and one balance control (Character); the
    // individual source dials are solved for, not set directly.
    const double motion = knob(kMotion, motion_->get());
    const double character = knob(kCharacter, character_->get());
    const int prec_idx = std::clamp(static_cast<int>(precision_->get()), 0, 2);
    const double target_peak =
        kMotionPeak * motion * mix_boost(character, prec_idx);
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
    const double tension_v = knob(kTension, tension_->get());
    const double tension_norm =
        std::pow(std::max(1.0, tension_v) / 300.0, kTensionP[prec_idx]);
    const double node_norm = std::pow(128.0 / std::max(1, n_nodes_), 0.25);
    // Wind's power LED gates the gust MECHANISM: off means no gust forcing
    // at all, which is what "turn the wind off" has to mean. Previously
    // only its rate dropped to minimum and the gusts kept blowing slowly.
    const double gust_speed = 25.0 * wind_dial * tension_norm * node_norm
                              * kNodeWindGain[prec_idx]
                              * wave_density_compensation(sim_->wave_density);
    sim_->wind_speed = mechanism_on(kWind) ? gust_speed : 0.0;
    // Wrinkle keeps its own amplitude reference, so turning the gust field
    // off does not silence it as well - it is a separate mechanism with its
    // own power switch.
    sim_->wrinkle_speed = gust_speed;
    // Flap limiter permanently off: aerodynamic drag IS the amplitude
    // mechanism now. Kept in the engine rather than deleted so the legacy
    // kFlagpoleWind model and the frozen validation scenarios still behave
    // exactly as their baselines record.
    sim_->flap_limit = 1e9;
    // Flutter amount, with the rate dial's amplitude swing compensated out so
    // Flutter Hz is a RATE control rather than a rate-and-volume control.
    // f0 is the string's fundamental and moves with Tension, node count and
    // cloth length, so it is computed live rather than baked in.
    {
        const double span = std::max(1.0, (n_nodes_ - 1.0)
                                              * std::max(0.02, whi - wlo));
        const double f0 = std::max(1e-6, tension_v / span);
        const double comp = flutter_rate_compensation(
            knob(kFlutterHz, flutter_hz_->get()) / f0);
        // Flutter Hz's power LED gates shedding entirely.
        sim_->shed_gain = mechanism_on(kFlutterHz)
                              ? 90.0 * shed_dial * comp * kNodeShedGain[prec_idx]
                              : 0.0;
    }
    // Wrinkle 0..1 -> 0..3. Measured tip/pole detail ratio 1.11 (off) ->
    // 1.45 at 1.5 -> 1.82 at 3.0, with the POLE end unchanged (0.83 ->
    // 0.87), i.e. the effect stays where it was asked for.
    // +30% amplitude on user request: 3.0 -> 3.9 at the top of the dial.
    sim_->wrinkle = 3.9 * knob(kWrinkle, wrinkle_->get());
    sim_->shed_freq_hz = knob(kFlutterHz, flutter_hz_->get());
    sim_->shed_spread = flutter_rnd_->get();
    sim_->shed_seed = flutter_seed_.load(std::memory_order_relaxed);
    // Smoothing is now a one-pole on disp at the sim tick rate. Both of
    // these used the RAW knob and so quietly ignored their own magnitude
    // modifiers; they take knob() like every other control now.
    {
        const double tau = std::max(1e-4, knob(kSmoothing, smoothing_->get()) * 1e-3);
        const double tick_rate = fs_ / std::max(1, sim_divisor_);
        smooth_alpha_ = 1.0 - std::exp(-1.0 / (tau * tick_rate));
    }
    bank_.set_steady(knob(kSteady, steady_->get()), 0.25);
    leveler_.set_amount(knob(kLeveler, leveler_amt_->get()));
    max_gain_db_.store(knob(kRangeDb, drive_->get()), std::memory_order_relaxed);

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
    mix_amount_ = knob(kMix, mix_->get());
    vel_amt_ = knob(kVelocity, velocity_->get());
    vel_scale_ = kVelScale / std::max(1.0, tension_v);
    const double range_db =
        mechanism_on(kRangeDb) ? max_gain_db_.load(std::memory_order_relaxed) : 0.0;
    // The spatial subscriber is neutral at zero depth, so it is skipped
    // outright rather than computed and multiplied by one.
    const double width_db = knob(kWidth, width_->get());
    const double pan_amt = knob(kPanDepth, pan_depth_->get());
    const bool spatial_on = (width_db > 1e-6 || pan_amt > 1e-6);
    if (spatial_on) {
        if (side_targets_.size() != (std::size_t)n_nodes_) {
            side_targets_.assign(n_nodes_, 1.0);
            pan_targets_.assign(n_nodes_, 0.0);
        }
        if (taper_lo_ != (double)stereo_lo_->get()
            || taper_hi_ != (double)stereo_hi_->get()
            || (int)spatial_w_.size() != n_nodes_)
            rebuild_spatial_taper();
    }
    // Targets are re-aimed per CHUNK inside the audio loop below, not once
    // per block -- see kGainChunk.

    // 3b. Reset request from the UI: flatten the string, touch nothing else.
    //     Wind, flutter and every knob stay exactly as they are, so the
    //     cloth starts moving again immediately from rest.
    if (reset_request_.exchange(false, std::memory_order_acquire)) {
        std::vector<double> flat(n_nodes_, 0.0);
        sim_->displace(flat);
    }

    // 4. Audio: tick the sim on its divisor grid, process per sample.
    //    When the internal test source is on, broadband pink noise replaces
    //    the input — the standalone then demos the string with no wiring.
    const int test_src =
        juce::jlimit(0, (int)kNumTestSrc - 1,
                     static_cast<int>(std::lround(test_src_->get())));
    const bool test_on = test_src != kTestOff;
    const int n = buffer.getNumSamples();
    float* ch0 = buffer.getWritePointer(0);
    float* ch1 = buffer.getNumChannels() > 1 ? buffer.getWritePointer(1) : nullptr;
    const bool stereo = ch1 != nullptr;
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
      // Targets from the control signal as it stands RIGHT NOW. disp was
      // already smoothed at the tick rate in its own (log) domain, so Range
      // enters exactly once and memorylessly.
      for (int i = 0; i < n_nodes_; ++i) {
          const double d = std::clamp(smoothed_disp_[i], -1.0, 1.0);
          const double db = d * range_db;
          // 10^(db/20) == 2^(db/6.0206); exp2 is markedly cheaper than pow.
          gain_targets_[i] = std::exp2(db * (1.0 / 6.020599913279624));
          if (spatial_on) {
              // Width is a DEPTH ON THE SIDE, in the same dB unit as Range:
              // the side gain is the mid gain plus Width dB of the same
              // shape, so the width ratio is 2^(d*Width/6.0206). EQ and
              // spatial read ONE curve and differ only in how hard they
              // drive it -- which is what keeps a single drawn curve honest
              // for both.
              const double w = spatial_w_[i];
              side_targets_[i] = std::exp2((db + d * width_db * w)
                                           * (1.0 / 6.020599913279624));
              pan_targets_[i] = d * pan_amt * w;
          }
      }
      bank_.set_block_targets(gain_targets_.data(), len);
      if (spatial_on)
          bank_.set_spatial_targets(side_targets_.data(), pan_targets_.data(), len);
      else
          bank_.set_spatial_off();
      for (int s = base; s < base + len; ++s) {
        if (++sim_counter_ >= sim_divisor_) {
            sim_->tick();
            sim_counter_ = 0;
            // The string has new information: fold it into the smoothed
            // control signal. This is the only smoothing in the plugin.
            const auto& u = sim_->positions();
            const auto& v = sim_->velocities();
            const double a = smooth_alpha_, va = vel_amt_, vs = vel_scale_;
            for (int i = 0; i < n_nodes_; ++i) {
                const double disp = (1.0 - va) * u[i] + va * (vs * v[i]);
                smoothed_disp_[i] += a * (disp - smoothed_disp_[i]);
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
        bank_.process_frame(l, r, stereo);
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

void CurvesAudioProcessor::publish_display() {
    // The audio thread must NEVER block on the GUI thread. The editor holds
    // this same spinlock while it copies the snapshot out, and a spinlock
    // owned by a lower-priority thread is a textbook priority inversion:
    // the audio thread burns its deadline spinning. Try, and bail if the
    // editor has it - dropping a frame is invisible at 30 Hz, a dropout is
    // not. (Diagnosed alongside the high-precision glitch report.)
    if (!display_lock_.tryEnter())
        return;
    const auto& u = sim_->positions();
    for (int i = 0; i < n_nodes_; ++i)
        display_buffer_[i] = static_cast<float>(u[i]);
    // The EQ curve of truth: the bank's smoothed per-band gains (post
    // smoothing, post Steady), normalized so +1 = Max gain dB.
    //
    // Mix is folded in EXACTLY, not approximately. The bank emits
    // x + sum(v1_b * big_b) and Mix scales only the added part, so the
    // effective per-band gain is 1 + Mix*(g - 1). Without this the curve
    // drew the full EQ depth while Mix 0.5 applied half of it -- the same
    // class of WYSIWYG violation Steady had, introduced when Mix was added.
    const auto& g = bank_.gains();
    const double range_db = max_gain_db_.load(std::memory_order_relaxed);
    const double mix = mix_amount_;
    for (int i = 0; i < n_nodes_; ++i) {
        const double g_eff = 1.0 + mix * (g[i] - 1.0);
        const double db = 20.0 * std::log10(std::max(g_eff, 1e-6));
        display_eq_buffer_[i] =
            static_cast<float>(std::clamp(db / std::max(1.0, range_db), -1.0, 1.0));
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

void CurvesAudioProcessor::pull_display(std::array<float, kMaxNodes>& out_pos,
                                        std::array<float, kMaxNodes>& out_eq,
                                        int& count) const {
    const juce::SpinLock::ScopedLockType lk(display_lock_);
    out_pos = display_buffer_;
    out_eq = display_eq_buffer_;
    count = display_count_;
}

void CurvesAudioProcessor::getStateInformation(juce::MemoryBlock& dest) {
    juce::ValueTree tree("CurvesState");
    for (auto* p : getParameters())
        if (auto* v = dynamic_cast<juce::RangedAudioParameter*>(p))
            tree.setProperty(juce::Identifier(v->paramID), v->getValue(), nullptr);
    tree.setProperty("uiTheme", ui_theme_, nullptr);
    juce::String mods;
    for (int k = 0; k < kNumModKnobs; ++k)
        mods << (knob_mods_[k].on ? "1" : "0") << juce::String(knob_mods_[k].mag);
    tree.setProperty("knobMods", mods, nullptr);
    tree.setProperty("flutterSeed", (int)flutter_seed_, nullptr);
    tree.setProperty("windSeed", (int)wind_seed_, nullptr);
    if (auto xml = tree.createXml())
        copyXmlToBinary(*xml, dest);
}

void CurvesAudioProcessor::setStateInformation(const void* data, int size) {
    if (auto xml = getXmlFromBinary(data, size))
        if (auto tree = juce::ValueTree::fromXml(*xml); tree.isValid()) {
            if (tree.hasProperty("uiTheme"))
                ui_theme_ = (int)tree.getProperty("uiTheme");
            if (tree.hasProperty("flutterSeed"))
                flutter_seed_ = (int)tree.getProperty("flutterSeed");
            if (tree.hasProperty("windSeed"))
                wind_seed_ = (int)tree.getProperty("windSeed");
            if (tree.hasProperty("knobMods")) {
                const juce::String m = tree.getProperty("knobMods").toString();
                for (int k = 0; k < kNumModKnobs && 2 * k + 1 < m.length(); ++k) {
                    knob_mods_[k].on = (m[2 * k] == '1');
                    knob_mods_[k].mag =
                        juce::jlimit(0, 2, static_cast<int>(m[2 * k + 1]) - '0');
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
