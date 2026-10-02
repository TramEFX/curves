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
#include <vector>

class CurvesAudioProcessor : public juce::AudioProcessor {
public:
    static constexpr int kMaxNodes = 1024;

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
                      int& count) const;
    // Display helpers.
    int active_nodes() const { return n_nodes_; }
    int physics_mode() const { return last_physics_; }

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
        kFlutterHz, kWrinkle, kNumModKnobs
    };
    // Magnitude scales the knob's travel ABOVE ITS MINIMUM, so High can
    // exceed the nominal maximum - that is the point of it.
    static constexpr double kMagScale[3] = {0.5, 1.0, 2.0};
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
    bool knob_on(int k) const { return knob_mods_[k].on; }
    int knob_mag(int k) const { return knob_mods_[k].mag; }
    void set_knob_on(int k, bool v) { knob_mods_[k].on = v; }
    void set_knob_mag(int k, int v) { knob_mods_[k].mag = juce::jlimit(0, 2, v); }
    static const char* mod_knob_id(int k);

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

    float pink_noise(int channel);
    void rebuild(int precision);
    void apply_physics(int mode);
    void rebuild_band_map();
    void publish_display();

    std::unique_ptr<curves::StringSim> sim_;
    curves::FilterBank bank_;
    curves::OutputLeveler leveler_;
    int display_accum_ = 0;          // samples since the last display publish
    int ui_theme_ = 0;               // editor colour theme (appearance only)
    std::array<KnobMod, kNumModKnobs> knob_mods_{};
    std::atomic<bool> reset_request_{false};
    double knob(int k, double raw) const {
        const auto& m = knob_mods_[k];
        // Magnitude always applies; power-off only collapses to the minimum
        // for knobs where the minimum is genuinely neutral. kPowerGate knobs
        // keep their value and are silenced at the mechanism instead.
        if (!m.on && power_mode(k) == kPowerToMin)
            return m.lo;
        return m.lo + (raw - m.lo) * kMagScale[juce::jlimit(0, 2, m.mag)];
    }
    bool mechanism_on(int k) const {
        return power_mode(k) != kPowerGate || knob_mods_[k].on;
    }
    double fs_ = 48000.0;
    int sim_counter_ = 0;
    int sim_divisor_ = 4;
    int n_nodes_ = 128;
    int last_physics_ = -1;
    int last_precision_ = -1;

    // band center frequencies + per-block gain targets
    std::vector<double> band_fc_;
    std::vector<double> gain_targets_{};
    double max_gain_db_ = 18.0;

    // draw mailbox (GUI -> audio); fixed max size, first n_nodes_ active
    juce::SpinLock draw_lock_;
    std::array<float, kMaxNodes> draw_buffer_{};
    int draw_count_ = 128;
    std::atomic<uint32_t> draw_version_{0};
    uint32_t last_draw_version_ = 0;

    // display mailbox (audio -> GUI)
    mutable juce::SpinLock display_lock_;
    std::array<float, kMaxNodes> display_buffer_{};
    std::array<float, kMaxNodes> display_eq_buffer_{};
    int display_count_ = 128;
    std::atomic<uint32_t> display_version_{0};

    juce::AudioParameterFloat* damping_ = nullptr;
    juce::AudioParameterFloat* stiffness_ = nullptr;
    juce::AudioParameterFloat* tension_ = nullptr;
    juce::AudioParameterFloat* smoothing_ = nullptr;
    juce::AudioParameterFloat* drive_ = nullptr;     // max gain dB
    juce::AudioParameterFloat* steady_ = nullptr;    // per-band dynamic EQ depth
    juce::AudioParameterFloat* leveler_amt_ = nullptr; // output RMS trim depth
    juce::AudioParameterFloat* test_src_ = nullptr;  // internal pink noise
    juce::AudioParameterFloat* physics_ = nullptr;   // force preset 0..3
    juce::AudioParameterFloat* precision_ = nullptr; // node count mode 0..2
    juce::AudioParameterFloat* velocity_ = nullptr;  // velocity coupling 0..1
    juce::AudioParameterFloat* wind_ = nullptr;      // gust phase velocity 0..1
    juce::AudioParameterFloat* waves_ = nullptr;     // wave concentration 0..1
    juce::AudioParameterFloat* motion_ = nullptr;    // total excitation 0..1
    juce::AudioParameterFloat* character_ = nullptr; // gust <-> shedding 0..1
    juce::AudioParameterFloat* direction_ = nullptr; // wind travel direction 0/1
    juce::AudioParameterFloat* wall_lo_ = nullptr;   // active window, low edge
    juce::AudioParameterFloat* wall_hi_ = nullptr;   // active window, high edge
    juce::AudioParameterFloat* flutter_hz_ = nullptr;// shedding rate, Hz
    juce::AudioParameterFloat* wrinkle_ = nullptr;   // tip fine detail 0..1
    double last_wall_lo_ = -1.0, last_wall_hi_ = -1.0;

    // Paul Kellet pink-noise filter state, per channel.
    std::array<float, 7> pink_[2]{};
    juce::Random random_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(CurvesAudioProcessor)
};
