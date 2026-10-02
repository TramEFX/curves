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
    juce::AudioParameterFloat* force_ = nullptr;     // wind speed 0..1
    juce::AudioParameterFloat* direction_ = nullptr; // wind travel direction 0/1
    juce::AudioParameterFloat* length_ = nullptr;    // cloth termination 0.2..1.0
    juce::AudioParameterFloat* flutter_ = nullptr;   // vortex shedding amount 0..1
    juce::AudioParameterFloat* flutter_hz_ = nullptr;// shedding rate, Hz
    float last_length_ = -1.0f;

    // Paul Kellet pink-noise filter state, per channel.
    std::array<float, 7> pink_[2]{};
    juce::Random random_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(CurvesAudioProcessor)
};
