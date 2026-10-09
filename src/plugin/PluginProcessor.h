// Curves — M2: physics personality.
//
// The string pushes the parameters; the physics presets push the string.
// Force presets (Free / Flagpole / Gust / Elastic) set anchors and force
// fields on the sim; precision modes scale the sim/bank/tick tuple
// (128/256/512 nodes); velocity coupling lets subscribers react to string
// velocity as well as position.
//
// Threading: the GUI thread is a *writer* (draw deposits, preset/param
// changes) and a *reader* (display snapshot). Both directions cross with
// juce::SpinLock guarding small copies — fine at current rates, noted as a
// possible lock-free upgrade.

#pragma once

#include "../filter_bank.h"
#include "../output_leveler.h"
#include "../string_sim.h"
#include <juce_audio_processors/juce_audio_processors.h>
#include <array>
#include <atomic>
#include <cmath>
#include <vector>

class CurvesAudioProcessor : public juce::AudioProcessor {
public:
    static constexpr int kMaxNodes = 1024;
    // ONE VOICE = ONE STRING. Voice 0 alone is the historical single
    // instance; in Dual mode voice 0 drives LEFT and voice 1 drives RIGHT,
    // each as a mono bank. This is the "multiple instances in parallel"
    // the sandbox was aimed at from the start -- stereo from genuinely
    // independent physics rather than from one string's mid/side.
    static constexpr int kNumVoices = 2;

    CurvesAudioProcessor();

    void prepareToPlay(double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;
    bool isBusesLayoutSupported(const BusesLayout& layouts) const override;
    void processBlock(juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override { return true; }

    const juce::String getName() const override { return "Curves"; }
    bool acceptsMidi() const override { return false; }
    bool producesMidi() const override { return false; }
    bool isMidiEffect() const override { return false; }
    double getTailLengthSeconds() const override { return 0.0; }

    int getNumPrograms() override { return 1; }
    int getCurrentProgram() override { return 0; }
    void setCurrentProgram(int) override {}
    const juce::String getProgramName(int) override { return "Default"; }
    void changeProgramName(int, const juce::String&) override {}

    void getStateInformation(juce::MemoryBlock&) override;
    void setStateInformation(const void*, int) override;

    // ---- GUI-side interfaces (editor only; called on the message thread) ----

    // Writer: deposit a drawn shape (held string).
    void push_draw(const std::array<float, kMaxNodes>& nodes, int count);
    // Writer: flatten the string immediately.
    void push_clear();
    // Reader: latest string positions AND the actual EQ curve (the bank's
    // smoothed per-band gains, normalized to +-1 at Max gain dB) so the
    // display is exactly what the EQ is doing.
    void pull_display(std::array<float, kMaxNodes>& out_pos,
                      std::array<float, kMaxNodes>& out_eq,
                      int& count) const {
        pull_display(ui_voice_, out_pos, out_eq, count);
    }
    // Per voice, so the editor can show BOTH strings side by side in Dual.
    void pull_display(int voice, std::array<float, kMaxNodes>& out_pos,
                      std::array<float, kMaxNodes>& out_eq, int& count) const;
    // Display helpers.
    // Test source: off plus five textures, chosen to probe different things
    // about a MOVING EQ rather than just to make noise.
    //   Pink   - the reference, perceptually flat
    //   White  - bright, shows what the top octaves are doing
    //   Brown  - low-heavy, shows the bass end moving
    //   Tone   - a sustained harmonic stack; the EQ reads as timbre/formant
    //            movement on a pitched sound instead of as a noise filter
    //   Pulse  - sparse clicks; exposes ringing, smoothing and transient
    //            behaviour, which steady noise completely hides
    enum TestSrc { kTestOff = 0, kTestPink, kTestWhite, kTestBrown,
                   kTestTone, kTestPulse, kNumTestSrc };
    static const char* test_src_name(int i);

    // Randomiser seeds. Deliberately NOT AudioParameters: a seed is not a
    // continuous quantity to automate, and two more automation entries for
    // something you click is noise. They ride in the state tree, so they
    // still save with the preset.
    int flutter_seed() const { return uiv().flutter_seed; }
    int wind_seed() const { return uiv().wind_seed; }
    void reroll_flutter_seed() {
        const int n = next_seed(uiv().flutter_seed);
        uiv().flutter_seed = n;
        const int m = mirror_voice();
        if (m >= 0)
            voices_[m].flutter_seed = n;
    }
    void reroll_wind_seed() {
        const int n = next_seed(uiv().wind_seed);
        uiv().wind_seed = n;
        const int m = mirror_voice();
        if (m >= 0)
            voices_[m].wind_seed = n;
    }

    // ---- which voice the panels are editing -----------------------------
    // Pure appearance: it selects which parameter set the knobs are bound
    // to and which voice the canvas draws, and rides in the state tree like
    // the colour theme. Dual on/off IS a parameter, because it changes the
    // audio path and wants automation and preset recall.
    int ui_voice() const { return ui_voice_; }
    void set_ui_voice(int v) { ui_voice_ = juce::jlimit(0, kNumVoices - 1, v); }
    bool dual_on() const { return dual_ != nullptr && dual_->get() > 0.5f; }
    void set_dual(bool on);
    // LINK mirrors every per-voice edit onto the other voice, so a patch can
    // be dialled in on both at once and then unlinked to let them diverge.
    // Switching it ON copies the voice you are looking at onto the other --
    // otherwise "linked" would still show two different sounds.
    bool link_on() const { return link_ != nullptr && link_->get() > 0.5f; }
    void set_link(bool on);
    // The voice an edit should ALSO be written to, or -1. One place to ask,
    // so every edit route (knobs, randomisers, walls, LED modifiers) agrees.
    int mirror_voice() const {
        if (!link_on())
            return -1;
        const int other = 1 - juce::jlimit(0, kNumVoices - 1, ui_voice_);
        return other;
    }

    int active_nodes() const { return n_nodes_; }
    // How many voices the LAST processed block actually ran. Dual needs a
    // stereo bus to mean anything, so this is not simply the Dual parameter
    // -- and it is what a test should assert on.
    // Test hook: the largest |control signal| any node has reached since
    // the last call, for the voice given. This is the quantity that clamps
    // at +-1, so O1's compensation has to be judged on it END TO END --
    // through the real budget solve, not a mirror of it.
    double take_control_peak(int voice) {
        Voice& v = voices_[juce::jlimit(0, kNumVoices - 1, voice)];
        return v.ctl_peak.exchange(0.0, std::memory_order_relaxed);
    }
    // RMS is the statistic to CALIBRATE against. The peak of a chaotic,
    // randomly-gusting cloth over a finite window has enormous run-to-run
    // variance -- the same settings measured 0.702 and 0.330 on two runs --
    // so a peak reading cannot distinguish a parameter effect from where
    // the simulation happened to be. RMS converges.
    // Fine-scale share of the control signal: the node-to-node difference
    // is a high-pass across the spectrum axis, so this is how much of the
    // curve is SMALL waves. Detail should pull it down while leaving the
    // overall rms alone -- that is the whole claim, so it gets measured.
    // RMS of the DETAIL component itself, measured with the same spatial
    // split the smoother uses. The first-difference metric is diluted by
    // the broad curve's own slope, so it understates what Detail does.
    // Only accumulates while the split is running, i.e. Detail > 0.
    double take_detail_rms(int voice) {
        Voice& v = voices_[juce::jlimit(0, kNumVoices - 1, voice)];
        const double s2 = v.ctl_det2.exchange(0.0, std::memory_order_relaxed);
        const double n = v.ctl_count.load(std::memory_order_relaxed);
        return n > 0.0 ? std::sqrt(s2 / n) : 0.0;
    }
    double take_fine_share(int voice) {
        Voice& v = voices_[juce::jlimit(0, kNumVoices - 1, voice)];
        const double f = v.ctl_fine2.exchange(0.0, std::memory_order_relaxed);
        const double s2 = v.ctl_sum2.load(std::memory_order_relaxed);
        return s2 > 0.0 ? std::sqrt(f / s2) : 0.0;
    }
    double take_control_rms(int voice) {
        Voice& v = voices_[juce::jlimit(0, kNumVoices - 1, voice)];
        const double s2 = v.ctl_sum2.exchange(0.0, std::memory_order_relaxed);
        const double n = v.ctl_count.exchange(0.0, std::memory_order_relaxed);
        return n > 0.0 ? std::sqrt(s2 / n) : 0.0;
    }
    int active_voices() const {
        return active_voices_.load(std::memory_order_relaxed);
    }
    // Test hook: the gain target a voice is currently driving band i to.
    // Lets a test see whether two voices diverge BEFORE the bank, which is
    // the difference between a physics problem and a routing problem.
    double voice_gain_target(int voice, int i) const {
        const Voice& v = voices_[juce::jlimit(0, kNumVoices - 1, voice)];
        return (i >= 0 && i < (int)v.gain_targets.size()) ? v.gain_targets[i] : 0.0;
    }
    double q_matched() const { return uiv().last_q_amt; }
    // Effective Range dB: the knob AFTER its magnitude modifier, which is
    // what the audio path actually uses. The canvas labels its dB axis from
    // this, so the marks stay truthful at any magnitude setting.
    double range_db() const {
        return uiv().range_db.load(std::memory_order_relaxed);
    }
    // What the canvas axis should be labelled in. With the EQ gated off but
    // the spatial subscriber running, the curve is the WIDTH shape and the
    // axis means width dB, not EQ dB.
    double display_depth_db() const { return display_depth_db(ui_voice_); }
    double display_depth_db(int voice) const {
        return voices_[juce::jlimit(0, kNumVoices - 1, voice)]
            .display_depth.load(std::memory_order_relaxed);
    }
    // 0 = mid gain (dB), 1 = side gain (dB), 2 = placement (%), 3 = nothing
    // applied. Ordered fallback: whichever subscriber is actually carrying
    // the shape. Enumerating these one at a time is how the curve went
    // missing twice, so this is exhaustive by construction.
    int display_src() const { return display_src(ui_voice_); }
    int display_src(int voice) const {
        return voices_[juce::jlimit(0, kNumVoices - 1, voice)]
            .display_src.load(std::memory_order_relaxed);
    }
    int physics_mode() const { return uiv().last_physics; }
    // The parameter ID the editor should bind a knob to, for the voice it
    // is currently showing: "damping" for voice 0, "damping_r" for voice 1.
    // The base ids of all per-voice parameters, so the editor can rebind.
    static const char* const* voice_param_ids(int& count);
    static juce::String voice_param_id(const char* base, int voice) {
        return voice == 0 ? juce::String(base)
                          : juce::String(base) + juce::String("_r");
    }

    // ---- per-knob modifiers (on/off + magnitude) ------------------------
    // Deliberately NOT AudioParameters. Nineteen knobs x (bypass + range)
    // would add 38 entries to the host's automation list for what is a
    // tuning aid, so these ride in the state tree like the colour theme.
    // Promote any that prove essential to real parameters later.
    //
    // Only continuous knobs are modifiable; Physics, Precision, Direction
    // and Test Noise are modes or switches where "half range" is meaningless.
    enum ModKnob {
        kDamping = 0, kStiffness, kTension, kSmoothing, kRangeDb, kSteady,
        kLeveler, kVelocity, kWind, kWaves, kMotion, kCharacter,
        kFlutterHz, kWrinkle, kMix, kFlap, kWidth, kPanDepth, kResonance,
        kNumModKnobs
    };
    // Magnitude scales the knob's travel ABOVE ITS MINIMUM, so High can
    // exceed the nominal maximum - that is the point of it.
    static constexpr double kMagScale[3] = {0.5, 1.0, 2.0};
    // Not every knob wants a magnitude selector. Mix is a plain 0..100%
    // blend -- "half travel" and "double travel" are meaningless on it, and
    // 100% has to stay reachable -- so it carries the power LED only.
    static bool has_magnitude(int k) { return k != kMix; }
    // What "power off" MEANS depends on the knob, and getting this wrong was
    // a real bug: a blanket "off = the knob's minimum" rule left Character
    // off = 0 = ALL GUST WIND, Wind off = the slowest rate (0.091 Hz, an
    // 11-second period) and Waves off = the most stretched pattern. Together
    // that produced large slow-moving curves with "everything off".
    //
    //   kPowerToMin  off -> the knob's minimum, which IS neutral for these:
    //                Motion 0, Steady 0, Velocity 0, Wrinkle 0, Leveler 0,
    //                Stiffness 0, Smoothing 2 ms, Range 3 dB, Damping 0.05.
    //   kPowerGate   off -> the whole MECHANISM is silenced, not the value.
    //                Wind gates the gust field; Flutter Hz gates shedding.
    //   kPowerNone   no power LED: "off" is meaningless for a balance
    //                (Character) or a scale/geometry (Waves, Tension, Length).
    enum PowerMode { kPowerNone = 0, kPowerToMin, kPowerGate };
    static PowerMode power_mode(int k);
    struct KnobMod {
        bool on = true;
        int mag = 1;        // 0 = low, 1 = medium, 2 = high
        double lo = 0.0;    // the parameter's minimum, captured at construction
    };
    // Mix and Leveler are GLOBAL knobs that happen to sit in the modifier
    // enum, and the audio path reads their modifiers from voice 0. Routing
    // the editor through the highlighted voice instead would mean toggling
    // Mix's power LED with R selected wrote to voice 1 and did nothing.
    static bool is_global_knob(int k) { return k == kMix || k == kLeveler; }
    bool knob_on(int k) const { return knob_voice(k).mods[k].on; }
    int knob_mag(int k) const { return knob_voice(k).mods[k].mag; }
    // Modifiers mirror under Link too -- a linked pair whose power LEDs and
    // magnitudes drift apart is not mirrored, and these change the audio
    // just as much as the knob under them. Global knobs have one set.
    void set_knob_on(int k, bool v) {
        knob_voice(k).mods[k].on = v;
        const int m = mirror_voice();
        if (m >= 0 && !is_global_knob(k))
            voices_[m].mods[k].on = v;
    }
    void set_knob_mag(int k, int v) {
        const int c = juce::jlimit(0, 2, v);
        knob_voice(k).mods[k].mag = c;
        const int m = mirror_voice();
        if (m >= 0 && !is_global_knob(k))
            voices_[m].mods[k].mag = c;
    }
    static const char* mod_knob_id(int k);
    // Band width == band spacing. Depends on node count, so Precision
    // changes it; rebuild() invalidates last_q_ to force a re-apply.
    static double matched_q(int n);

    // Flatten the string without touching any modifier: zeroes displacement
    // and velocity but leaves wind, flutter and every knob exactly as they
    // are, so the cloth immediately starts moving again from rest. Replaces
    // the workaround of winding Damping up to kill energy.
    void request_reset() { reset_request_.store(true, std::memory_order_release); }

    // UI colour theme index. Not an automatable parameter (it is pure
    // appearance), but it rides along in the state tree so a session
    // reopens looking the way it was left.
    int ui_theme() const { return ui_theme_; }
    void set_ui_theme(int t) { ui_theme_ = t; }

private:
    // Physics presets (the "physics" parameter).
    enum PhysicsMode { kFree = 0, kFlagpole = 1, kGust = 2, kElastic = 3 };

    static constexpr double kFMin = 30.0, kFMax = 18000.0;
    static constexpr int kPrecisionNodes[3] = {128, 256, 512};
    static constexpr int kPrecisionDivisor[3] = {4, 2, 2};

    // Everything that belongs to ONE string. This was a flat set of members
    // on the processor; gathering it into a struct is what lets it exist
    // twice. Each voice owns its own sim, bank, control smoother, spatial
    // taper, seeds AND its own complete set of knob modifiers, because each
    // voice has its own complete set of knobs.
    struct Voice {
        std::unique_ptr<curves::StringSim> sim;
        curves::FilterBank bank;
        int sim_counter = 0;
        int last_physics = -1;
        double last_q = -1.0;
        double last_q_amt = -1.0;
        double last_wall_lo = -1.0, last_wall_hi = -1.0;
        std::vector<double> gain_targets, side_targets, pan_targets;
        std::vector<double> spatial_w;
        double taper_lo = -1.0, taper_hi = -1.0;
        std::vector<double> resp_big, resp_re, resp_im;   // display scratch
        std::array<double, kMaxNodes> smoothed_disp{};
        // Detail smoothing splits the control signal by SPATIAL scale and
        // smooths the fine part harder. Only touched when the knob is up.
        std::array<double, kMaxNodes> sm_broad{}, sm_detail{};
        std::array<double, kMaxNodes> disp_tmp{}, blur_tmp{};
        double detail_amt = 0.0, detail_b = 0.0;
        double smooth_alpha = 0.05, vel_amt = 0.0, vel_scale = 0.0;
        std::array<KnobMod, kNumModKnobs> mods{};
        std::atomic<int> flutter_seed{1};
        std::atomic<int> wind_seed{1};
        // Published per block for the editor's readouts.
        std::atomic<double> range_db{18.0};
        std::atomic<double> display_depth{18.0};
        std::atomic<int> display_src{0};
        std::atomic<double> ctl_peak{0.0};
        std::atomic<double> ctl_sum2{0.0};
        std::atomic<double> ctl_fine2{0.0};
        std::atomic<double> ctl_det2{0.0};
        std::atomic<double> ctl_count{0.0};
    };
    // The per-voice parameter set. Precision, Mix, Leveler, Test Src and
    // Dual are global and stay on the processor -- they are output plumbing
    // or the routing switch itself, not properties of a string.
    struct VoiceParams {
        juce::AudioParameterFloat* damping = nullptr;
        juce::AudioParameterFloat* stiffness = nullptr;
        juce::AudioParameterFloat* tension = nullptr;
        juce::AudioParameterFloat* smoothing = nullptr;
        juce::AudioParameterFloat* drive = nullptr;
        juce::AudioParameterFloat* steady = nullptr;
        juce::AudioParameterFloat* physics = nullptr;
        juce::AudioParameterFloat* velocity = nullptr;
        juce::AudioParameterFloat* wind = nullptr;
        juce::AudioParameterFloat* waves = nullptr;
        juce::AudioParameterFloat* motion = nullptr;
        juce::AudioParameterFloat* character = nullptr;
        juce::AudioParameterFloat* direction = nullptr;
        juce::AudioParameterFloat* wall_lo = nullptr;
        juce::AudioParameterFloat* wall_hi = nullptr;
        juce::AudioParameterFloat* flutter_hz = nullptr;
        juce::AudioParameterFloat* wrinkle = nullptr;
        juce::AudioParameterFloat* flap = nullptr;
        juce::AudioParameterFloat* wind_rnd = nullptr;
        juce::AudioParameterFloat* flutter_rnd = nullptr;
        juce::AudioParameterFloat* width = nullptr;
        juce::AudioParameterFloat* pan_depth = nullptr;
        juce::AudioParameterFloat* stereo_lo = nullptr;
        juce::AudioParameterFloat* stereo_hi = nullptr;
        juce::AudioParameterFloat* q_mode = nullptr;
        juce::AudioParameterFloat* resonance = nullptr;
        juce::AudioParameterFloat* detail = nullptr;
    };
    // What prepare_voice() hands the audio loop for this block. These used
    // to be locals in processBlock; with two voices they have to be per
    // voice, and a struct beats four out-parameters.
    struct VoiceBlock {
        bool spatial_on = false;
        double range_db = 0.0;
        double width_db = 0.0;
        double pan_amt = 0.0;
    };
    Voice voices_[kNumVoices];
    VoiceParams vp_[kNumVoices];
    float test_sample(int src, int channel);
    float pink_noise(int channel);
    void rebuild(int precision);
    void rebuild_voice(Voice& v, const VoiceParams& p);
    void apply_physics(Voice& v, int mode);
    void rebuild_band_map(Voice& v);
    void publish_display();
    // Maps a voice's parameters onto its engine; everything that used to be
    // the first two thirds of processBlock.
    void prepare_voice(Voice& v, const VoiceParams& p, int prec_idx,
                       VoiceBlock& out);
    void add_voice_params(VoiceParams& p, int voice);
    // Copies every parameter and modifier of one voice onto another, then
    // re-rolls the destination's seeds. Used when Dual is switched on.
    // reroll_seeds: fresh gusts on the destination (what Dual wants), vs an
    // exact mirror including seeds (what Link wants).
    void copy_voice(int from, int to, bool reroll_seeds = true);
    juce::AudioParameterFloat* find_param(const juce::String& id) const;

    std::atomic<int> active_voices_{1};
    int ui_voice_ = 0;               // which voice the panels are editing
    Voice& uiv() { return voices_[juce::jlimit(0, kNumVoices - 1, ui_voice_)]; }
    const Voice& uiv() const {
        return voices_[juce::jlimit(0, kNumVoices - 1, ui_voice_)];
    }
    // Declared here rather than beside knob_on() because the return type
    // needs Voice to be complete.
    Voice& knob_voice(int k) { return is_global_knob(k) ? voices_[0] : uiv(); }
    const Voice& knob_voice(int k) const {
        return is_global_knob(k) ? voices_[0] : uiv();
    }

    curves::OutputLeveler leveler_;
    int display_accum_ = 0;          // samples since the last display publish
    int ui_theme_ = 0;               // editor colour theme (appearance only)
    std::atomic<bool> reset_request_{false};
    // knob() and mechanism_on() read the modifiers of a SPECIFIC voice --
    // they used to read the processor's single set, which is exactly the
    // kind of shared state that would silently couple the two voices.
    static double knob(const Voice& v, int k, double raw) {
        const auto& m = v.mods[k];
        // Magnitude always applies; power-off only collapses to the minimum
        // for knobs where the minimum is genuinely neutral. kPowerGate knobs
        // keep their value and are silenced at the mechanism instead.
        if (!m.on && power_mode(k) == kPowerToMin)
            return m.lo;
        if (!has_magnitude(k))
            return raw;
        return m.lo + (raw - m.lo) * kMagScale[juce::jlimit(0, 2, m.mag)];
    }
    static bool mechanism_on(const Voice& v, int k) {
        return power_mode(k) != kPowerGate || v.mods[k].on;
    }
    double fs_ = 48000.0;
    int sim_divisor_ = 4;
    int n_nodes_ = 128;
    int last_precision_ = -1;

    // band center frequencies + per-block gain targets
    std::vector<double> band_fc_;

    // THE SMOOTHER. It lives here, on the string's own control signal, not
    // downstream on the linear gain where it used to sit.
    //
    // disp is already the log-amplitude coordinate: the x axis is log
    // frequency and disp maps LINEARLY to dB, so the whole chain is linear
    // physics on a log-log graph and Range dB is a single memoryless scale
    // at the very end. Smoothing the linear gain instead put Range inside a
    // recursive filter, which is not scale-invariant: measured, boosts
    // reached 90% in 46 ms and cuts in 102 ms at Range 18 (2.21x, widening
    // with Range), and the drawn curve moved by up to 0.30 of full scale
    // purely from a Range change. Smoothing disp makes all of that exact.
    //
    // Runs at the SIM TICK rate, which is where disp actually carries new
    // information; it is also an anti-alias filter ahead of the block-rate
    // sampling the bank sees.  (ORCHESTRATION.md:155 specified this from the
    // start -- "one one-pole smoother toward the current sim value".)
    double mix_amount_ = 1.0;      // dry/wet, cached per block
    // Read by the GUI thread for the dB axis; relaxed atomic because a
    // torn double would print a nonsense label.

    // draw mailbox (GUI -> audio); fixed max size, first n_nodes_ active
    juce::SpinLock draw_lock_;
    std::array<float, kMaxNodes> draw_buffer_{};
    int draw_count_ = 128;
    std::atomic<uint32_t> draw_version_{0};
    uint32_t last_draw_version_ = 0;

    // display mailbox (audio -> GUI)
    mutable juce::SpinLock display_lock_;
    // One snapshot PER VOICE: in Dual the editor draws both canvases, so
    // publishing only the highlighted one would leave the other frozen.
    std::array<float, kMaxNodes> display_buffer_[kNumVoices]{};
    std::array<float, kMaxNodes> display_eq_buffer_[kNumVoices]{};
    int display_count_ = 128;
    std::atomic<uint32_t> display_version_{0};

    // ---- GLOBAL parameters ------------------------------------------
    // Output plumbing and the routing switch. Everything else is per voice
    // and lives in vp_[].
    juce::AudioParameterFloat* mix_ = nullptr;         // dry/wet
    juce::AudioParameterFloat* leveler_amt_ = nullptr; // output RMS trim
    juce::AudioParameterFloat* test_src_ = nullptr;    // internal test source
    juce::AudioParameterFloat* precision_ = nullptr;   // node count mode 0..2
    // Dual IS a parameter: it changes the audio path, so it wants
    // automation and preset recall. Which voice you are LOOKING at does
    // not, and rides in the state tree instead.
    juce::AudioParameterFloat* dual_ = nullptr;
    juce::AudioParameterFloat* link_ = nullptr;
    // ---- SPATIAL subscriber -------------------------------------------
    // Width is expressed in dB, the same unit as Range, because that is
    // literally what it is: how far the SIDE gain departs from the mid at
    // full deflection. width ratio = 2^(disp*Width/6.0206), so the two
    // depths share one shape and differ only in how hard they drive it.
    juce::AudioParameterFloat* width_ = nullptr;
    juce::AudioParameterFloat* pan_depth_ = nullptr;
    // Per-band taper that damps the spatial deviation outside [lo, hi].
    // Cached: it only moves when the cuts or the band map do.
    void rebuild_spatial_taper(Voice& v, const VoiceParams& p);

    static int next_seed(int cur) {
        // any step that visits a lot of distinct values; the hash does the
        // actual scrambling, this only needs to not repeat quickly
        return static_cast<int>((static_cast<unsigned>(cur) * 1664525u
                                 + 1013904223u) & 0x7fffffffu);
    }
    // Paul Kellet pink-noise filter state, per channel.
    std::array<float, 7> pink_[2]{};
    float brown_[2]{};          // leaky integrator state
    double tone_phase_[2]{};    // harmonic stack phase
    int pulse_count_[2]{};      // samples since the last click
    float pulse_env_[2]{};
    juce::Random random_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(CurvesAudioProcessor)
};
