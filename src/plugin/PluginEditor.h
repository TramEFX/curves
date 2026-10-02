// Curves — M2 editor: the drawable string canvas + control strip.
//
// Left-drag draws (deposits displacement into nodes, string is "held");
// release lets it ring. Right-click flattens. The displayed curve is the
// live sim state, published by the audio thread. Anchored ends get a pin
// marker. Physics preset selection also sets Damping/Stiffness/Tension.

#pragma once

#include "PluginProcessor.h"
#include <juce_audio_utils/juce_audio_utils.h>
#include <array>
#include <memory>
#include <vector>

class CurvesAudioProcessorEditor : public juce::AudioProcessorEditor,
                                   private juce::Timer,
                                   private juce::Slider::Listener {
public:
    explicit CurvesAudioProcessorEditor(CurvesAudioProcessor&);
    ~CurvesAudioProcessorEditor() override;

    void paint(juce::Graphics&) override;
    void resized() override;

    void mouseDown(const juce::MouseEvent&) override;
    void mouseDrag(const juce::MouseEvent&) override;
    void mouseUp(const juce::MouseEvent&) override;

private:
    // One rotary control bound to a raw AudioParameterFloat (no APVTS —
    // M1 keeps parameter plumbing minimal; APVTS refactor can come later).
    struct Knob {
        juce::Slider slider;
        juce::Label label;
        juce::RangedAudioParameter* param = nullptr;
    };

    int active_nodes() const { return active_count_; }

    // Canvas amplitude scale (scope-style +-div): the string may flap past
    // +-1; switching the display scale gives it headroom instead of hard
    // visual caps. Drawing resolution scales with it.
    void cycle_display_scale();
    float display_scale() const { return display_scale_; }

    // Pixel <-> node/value mapping (log frequency axis, ±1 amplitude).
    juce::Rectangle<float> plot_bounds() const;
    float node_to_x(float node) const;
    float x_to_node(float x) const;
    float value_to_y(float v) const;
    float y_to_value(float y) const;
    void deposit_stroke(float node_from, float val_from, float node_to, float val_to);
    void apply_physics_preset(int mode);

    void timerCallback() override;
    void sliderValueChanged(juce::Slider*) override;
    void push_current();

    CurvesAudioProcessor& processor_;

    // Draw/display buffers: fixed max size, first active_count_ entries used.
    std::array<float, CurvesAudioProcessor::kMaxNodes> draw_nodes_{};
    std::array<float, CurvesAudioProcessor::kMaxNodes> display_nodes_{};
    std::array<float, CurvesAudioProcessor::kMaxNodes> display_eq_{};
    juce::TextButton scale_button_{"+-1"};
    float display_scale_ = 1.0f;
    int active_count_ = 128;
    bool drawing_ = false;
    bool dirty_ = false;  // draw buffer changed since last display refresh
    float last_node_ = 0.f, last_val_ = 0.f;

    std::vector<std::unique_ptr<Knob>> knobs_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(CurvesAudioProcessorEditor)
};
