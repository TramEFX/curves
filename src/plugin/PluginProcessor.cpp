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
double dial_for_peak(double target, double A, double P) {
    if (target <= 1e-9)
        return 0.0;
    return std::clamp(std::pow(target / A, 1.0 / P), 0.0, 1.0);
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
    addParameter(smoothing_ = new APF("smoothing", "Smoothing ms", R{2.0f, 100.0f}, 30.0f));
    addParameter(drive_ = new APF("drive", "Max gain dB", R{3.0f, 24.0f}, 18.0f));
    addParameter(steady_ = new APF("steady", "Steady (per-band)", R{0.0f, 1.0f}, 0.35f));
    addParameter(leveler_amt_ = new APF("leveler", "Leveler (output)", R{0.0f, 1.0f}, 0.5f));
    addParameter(test_src_ = new APF("test", "Test noise", R{0.0f, 1.0f}, 0.0f));
    addParameter(physics_ = new APF("physics", "Physics", R(0.0f, 3.0f, 1.0f), 0.0f));
    addParameter(precision_ = new APF("precision", "Precision", R(0.0f, 2.0f, 1.0f), 0.0f));
    addParameter(velocity_ = new APF("velocity", "Velocity", R{0.0f, 1.0f}, 0.0f));
    addParameter(wind_ = new APF("wind", "Wind", R{0.0f, 1.0f}, 0.37f));
    addParameter(waves_ = new APF("waves", "Waves", R{0.0f, 1.0f}, 0.4f));
    // Motion = total string excitation, calibrated so peak displacement
    // tracks the dial linearly regardless of Character. Default chosen to
    // match the previous Force 0.6 wind-only feel (peak ~0.37).
    addParameter(motion_ = new APF("motion", "Motion", R{0.0f, 1.0f}, 0.45f));
    // Character = where that excitation comes from. 0 = all gust wind,
    // 1 = all self-sustaining vortex shedding. Default 0 reproduces the
    // previous behaviour exactly; raising it is a tuning decision.
    addParameter(character_ = new APF("character", "Character", R{0.0f, 1.0f}, 0.0f));
    addParameter(direction_ = new APF("direction", "Direction", R(0.0f, 1.0f, 1.0f), 0.0f));
    addParameter(length_ = new APF("length", "Cloth length", R{0.2f, 1.0f}, 1.0f));
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
    bank_.set_smoothing(smoothing_->get());
    bank_.set_steady(steady_->get(), 0.25);
    rebuild_band_map();
    gain_targets_.assign(n_nodes_, 1.0);
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
        sim_->set_cloth_length(length_->get());
    } else if (mode == kGust) {
        // No anchors; smooth zero-mean static force field across the spectrum.
        const double w = 8.0 * (static_cast<double>(n_nodes_) / 128.0);
        for (int i = 0; i < n_nodes_; ++i)
            field[i] = -w * std::sin(3.14159265358979323846 * i / (n_nodes_ - 1));
    }
    // kFree / kElastic: no anchors, no forces — knobs do the talking.
    sim_->set_force_field(field);
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
    sim_->set_damping(damping_->get());
    sim_->set_stiffness(stiffness_->get());
    sim_->set_wave_speed(tension_->get());
    // Knob semantics (project decision): Wind = how fast the cloth flaps
    // (gust phase velocity, 10..500 nodes/s). Force = amplitude LIMITER —
    // the flap-distance ceiling, independent of drive and resonance. The
    // drive amplitude itself is an internal constant.
    sim_->wind_phase_speed = (10.0 + 490.0 * wind_->get())
                             * (static_cast<double>(n_nodes_) / 128.0);
    // Waves knob: spatial concentration — 0.5x (stretched) to 3x (packed)
    sim_->wave_density = 0.5 + 2.5 * waves_->get();
    sim_->set_wind_origin(direction_->get() > 0.5f ? 1 : 0);
    if (std::abs(length_->get() - last_length_) > 1e-4f) {
        sim_->set_cloth_length(length_->get());
        last_length_ = length_->get();
    }
    // --- excitation budget --------------------------------------------
    // One amount control (Motion) and one balance control (Character); the
    // individual source dials are solved for, not set directly.
    const double motion = motion_->get();
    const double character = character_->get();
    const int prec_idx = std::clamp(static_cast<int>(precision_->get()), 0, 2);
    const double target_peak =
        kMotionPeak * motion * mix_boost(character, prec_idx);
    const double wind_dial = dial_for_peak((1.0 - character) * target_peak,
                                           kWindA, kWindP);
    const double shed_dial = dial_for_peak(character * target_peak,
                                           kShedA, kShedP);

    // Force = WIND SPEED (nodes/s) in the aero model. Drag rises as the
    // string's own velocity catches up to the wind, so amplitude is
    // compressive and monotonic across the whole dial with no ceiling to
    // tune, no dead zone, and no limiter gain that can latch.
    //
    // Calibration (measured, ORCHESTRATION §14) against a LONG observation
    // window: the gusts are quasi-periodic, so a short run under-reports the
    // peak — the first pass calibrated on 3.5 s and read 0.995 where the
    // 30 s peak was 1.41. Re-tuned so peak |u| stays under the ±1 clamp the
    // bank and the display share, in every precision mode and at the knob
    // extremes, which is what keeps the on-screen shape a faithful picture
    // of the EQ rather than a flat-topped one.
    //   wind_speed = 25 * force * (tension/300)^0.6 * (128/n)^0.25
    // The tension term makes the dial mean the same thing at any Tension
    // (spread 4.7x -> 1.4x); the node term holds it across precision modes.
    // No node-count scaling of a FORCE any more: wind_speed is a velocity in
    // the same units as the node velocities it is compared against.
    const double tension_norm = std::pow(std::max(1.0f, tension_->get()) / 300.0, 0.6);
    const double node_norm = std::pow(128.0 / std::max(1, n_nodes_), 0.25);
    sim_->wind_speed = 25.0 * wind_dial * tension_norm * node_norm
                       * kNodeWindGain[prec_idx];
    // Flap limiter retired in the aero path: drag IS the amplitude
    // mechanism. Left off rather than removed so the kFlagpoleWind model and
    // the six original validation scenarios keep their exact behavior.
    sim_->flap_limit = 1e9;
    // Flutter: shedding amount and rate, gain topping out at 30.
    //
    // AMPLITUDE BUDGET IS SHARED WITH FORCE, and this does not fully fit.
    // Giving the wake an outward travelling phase ramp made it COHERENT, so
    // it now ADDS to the gusts where the old incoherent version partly
    // cancelled them. Measured long-run peak at Force 1.0 (n = 256/512/1024):
    //   gain  0 -> 0.826 / 1.011 / 0.906
    //   gain  5 -> 0.841 / 0.931 / 0.943
    //   gain  8 -> 0.853 / 0.971 / 0.992
    //   gain 30 -> 1.104 (n=256), 1.585 (n=512)
    // So only ~5 would keep every precision mode under the clamp at Force
    // 1.0 - which would make Flutter too weak to be worth having. The range
    // is kept useful instead: Force and Flutter both near maximum WILL reach
    // the +-1 clamp and flat-top. That stays honest against the WYSIWYG
    // anchor (the display draws the clamped curve, so it is visible rather
    // than hidden) and it matches the standing project decision to favour
    // waves true to the simulation over an enforced ceiling. Flutter
    // defaults to 0, so nothing clamps unless both are dialled up.
    // Note also that n = 512 reads 1.011 at Flutter 0, so the Force base-25
    // calibration is itself marginal in that one precision mode.
    // Gain ceiling raised 30 -> 90 on user request. Measured peak at Force
    // 1.0: 1.585 at 30, 1.977 at 60, 2.233 at 90; alone (Force 0): 0.585 /
    // 0.876 / 1.096. So the upper half of the dial will clamp when Force is
    // also up - the shared-budget characteristic noted above, now with more
    // room to push into deliberately.
    // Flutter amount, with the rate dial's amplitude swing compensated out so
    // Flutter Hz is a RATE control rather than a rate-and-volume control.
    // f0 is the string's fundamental and moves with Tension, node count and
    // cloth length, so it is computed live rather than baked in.
    {
        const double span = std::max(1.0, (n_nodes_ - 1.0) *
                                              static_cast<double>(length_->get()));
        const double f0 = std::max(1e-6, static_cast<double>(tension_->get()) / span);
        const double comp = flutter_rate_compensation(
            static_cast<double>(flutter_hz_->get()) / f0);
        sim_->shed_gain = 90.0 * shed_dial * comp * kNodeShedGain[prec_idx];
    }
    // Wrinkle 0..1 -> 0..3. Measured tip/pole detail ratio 1.11 (off) ->
    // 1.45 at 1.5 -> 1.82 at 3.0, with the POLE end unchanged (0.83 ->
    // 0.87), i.e. the effect stays where it was asked for.
    sim_->wrinkle = 3.0 * wrinkle_->get();
    sim_->shed_freq_hz = flutter_hz_->get();
    bank_.set_smoothing(smoothing_->get());
    bank_.set_steady(steady_->get(), 0.25);
    leveler_.set_amount(leveler_amt_->get());
    max_gain_db_ = drive_->get();

    // 3. Per-block gain targets: position + optional velocity coupling.
    //    Velocity is normalized by tension so coupling is tension-independent.
    const double vel_amt = velocity_->get();
    const double ws = std::max(1.0, static_cast<double>(tension_->get()));
    const auto& u = sim_->positions();
    const auto& v = sim_->velocities();
    for (int i = 0; i < n_nodes_; ++i) {
        const double disp = u[i] + vel_amt * (v[i] / ws);
        const double db = std::clamp(disp, -1.0, 1.0) * max_gain_db_;
        // 10^(db/20) == 2^(db/6.0206). exp2 is markedly cheaper than pow and
        // this runs once per node per block (512 calls at high precision).
        gain_targets_[i] = std::exp2(db * (1.0 / 6.020599913279624));
    }
    bank_.set_targets(gain_targets_.data());

    // 4. Audio: tick the sim on its divisor grid, process per sample.
    //    When the internal test source is on, broadband pink noise replaces
    //    the input — the standalone then demos the string with no wiring.
    const bool test_on = test_src_->get() > 0.5f;
    const int n = buffer.getNumSamples();
    float* ch0 = buffer.getWritePointer(0);
    float* ch1 = buffer.getNumChannels() > 1 ? buffer.getWritePointer(1) : nullptr;
    for (int s = 0; s < n; ++s) {
        if (++sim_counter_ >= sim_divisor_) {
            sim_->tick();
            sim_counter_ = 0;
        }
        if (test_on) {
            ch0[s] = pink_noise(0);
            if (ch1)
                ch1[s] = pink_noise(1);
        }
        float l = bank_.process_sample(0, ch0[s]);
        float r = ch1 ? bank_.process_sample(1, ch1[s]) : l;
        leveler_.process(l, r);
        ch0[s] = l;
        if (ch1)
            ch1[s] = r;
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
    const auto& g = bank_.gains();
    for (int i = 0; i < n_nodes_; ++i) {
        const double db = 20.0 * std::log10(std::max(g[i], 1e-6));
        display_eq_buffer_[i] =
            static_cast<float>(std::clamp(db / std::max(1.0, max_gain_db_), -1.0, 1.0));
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
    if (auto xml = tree.createXml())
        copyXmlToBinary(*xml, dest);
}

void CurvesAudioProcessor::setStateInformation(const void* data, int size) {
    if (auto xml = getXmlFromBinary(data, size))
        if (auto tree = juce::ValueTree::fromXml(*xml); tree.isValid()) {
            if (tree.hasProperty("uiTheme"))
                ui_theme_ = (int)tree.getProperty("uiTheme");
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
