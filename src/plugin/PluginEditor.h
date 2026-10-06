// Curves — editor: the drawable string canvas + grouped control panels.
//
// Left-drag draws (deposits displacement into nodes, string is "held");
// release lets it ring. Right-click flattens. Anchored ends get a pin
// marker. Physics preset selection also sets Damping/Stiffness/Tension.
//
// THE HARD ANCHOR (project decision, never negotiable): the bright curve is
// drawn from the FILTER BANK'S smoothed gains, never from the sim, and uses
// the same ±1 clamp the audio path uses. What you see is exactly the EQ
// being applied. The sim is shown separately as a faint reference line and
// is labelled as such.
//
// Controls are grouped by scope rather than listed in one strip: GLOBAL
// (always in effect), STRING (the physics that always applies), and CLOTH &
// WIND (only meaningful in the Flagpole preset, so it dims when it is not).

#pragma once

#include "PluginProcessor.h"
#include <juce_audio_utils/juce_audio_utils.h>
#include <array>
#include <memory>
#include <vector>

// ---- colour themes -------------------------------------------------------
// Five schemes. Aqua is the default: neon aqua / electric blue.
struct CurvesTheme {
    const char* name;
    juce::uint32 bg;          // window background
    juce::uint32 plot_top;    // plot gradient, top
    juce::uint32 plot_bot;    // plot gradient, bottom
    juce::uint32 panel;       // group panel fill
    juce::uint32 panel_edge;  // group panel border
    juce::uint32 grid;        // minor grid
    juce::uint32 grid_key;    // zero line / key frequencies
    juce::uint32 curve;       // THE EQ CURVE (bright core)
    juce::uint32 glow;        // curve glow + knob arcs
    juce::uint32 fill;        // area fill under the curve
    juce::uint32 sim_ref;     // faint sim reference line
    juce::uint32 draw;        // deposit preview while dragging
    juce::uint32 anchor;      // anchor pin
    juce::uint32 text;        // titles
    juce::uint32 text_dim;    // labels / hints
};

const CurvesTheme& curves_theme(int index);
int curves_theme_count();

// Rotary look: a dark track arc, a glowing value arc in the theme colour,
// and a short pointer. Replaces the stock JUCE rotary so the panels read as
// one designed surface rather than default widgets.
class CurvesKnobLook : public juce::LookAndFeel_V4 {
public:
    void set_theme(const CurvesTheme* t) { theme_ = t; }
    void drawRotarySlider(juce::Graphics&, int x, int y, int w, int h,
                          float pos, float start, float end,
                          juce::Slider&) override;

private:
    const CurvesTheme* theme_ = nullptr;
};

// A small dial that does two jobs: DRAG sets the randomisation amount,
// a plain CLICK re-rolls the seed. Distinguished by drag distance, so a
// click that happens to wobble a pixel still counts as a click.
class SeedSlider : public juce::Slider {
public:
    std::function<void()> on_reroll;
    void mouseDown(const juce::MouseEvent& e) override {
        dragged_ = false;
        juce::Slider::mouseDown(e);
    }
    void mouseDrag(const juce::MouseEvent& e) override {
        if (e.getDistanceFromDragStart() > 3)
            dragged_ = true;
        juce::Slider::mouseDrag(e);
    }
    void mouseUp(const juce::MouseEvent& e) override {
        juce::Slider::mouseUp(e);
        if (!dragged_ && on_reroll)
            on_reroll();
    }

private:
    bool dragged_ = false;
};

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
    // Which panel a control belongs to. GLOBAL is always in effect; STRING
    // is the physics that always applies; CLOTH is Flagpole-only and dims
    // outside it.
    // SUBSCRIBERS is its own group because that is the architecture: each
    // subscriber reads the same control signal and carries its own depth
    // and limits. EQ and SPATIAL today; per-node Q and cutoff modulation
    // slot in here without a redesign.
    enum Group { kGlobal = 0, kString = 1, kCloth = 2, kSubscriber = 3,
                 kNumGroups = 4 };

    struct Knob {
        juce::Slider slider;
        juce::Label label;
        juce::RangedAudioParameter* param = nullptr;
        int group = kGlobal;
        int mod = -1;                    // ModKnob index, or -1 if not modifiable
        juce::Rectangle<int> power;      // on/off LED hit area
        juce::Rectangle<int> magnitude;  // low/med/high LED hit area
    };

    // LED: dark glass bead with a radial highlight, lit by a colour with a
    // bloom behind it. `lit` false renders the same bead unlit (black).
    void draw_led(juce::Graphics&, juce::Rectangle<int>, juce::Colour,
                  bool lit) const;

    int active_nodes() const { return active_count_; }

    void cycle_display_scale();
    void cycle_theme();
    const CurvesTheme& theme() const { return curves_theme(theme_index_); }
    void apply_theme();

    // Panel rectangles, computed in resized() and reused by paint() so the
    // frames and the controls inside them cannot drift apart.
    juce::Rectangle<int> panel_[kNumGroups];

    juce::Rectangle<float> plot_bounds() const;
    float node_to_x(float node) const;
    float x_to_node(float x) const;
    float value_to_y(float v) const;
    float y_to_value(float y) const;
    void deposit_stroke(float node_from, float val_from, float node_to, float val_to);
    void apply_physics_preset(int mode);
    void draw_panel(juce::Graphics&, juce::Rectangle<int>, const char* title,
                    bool active) const;

    void timerCallback() override;
    void sliderValueChanged(juce::Slider*) override;
    void push_current();

    CurvesAudioProcessor& processor_;

    std::array<float, CurvesAudioProcessor::kMaxNodes> draw_nodes_{};
    std::array<float, CurvesAudioProcessor::kMaxNodes> display_nodes_{};
    std::array<float, CurvesAudioProcessor::kMaxNodes> display_eq_{};
    juce::TextButton scale_button_{"+-1"};
    juce::TextButton reset_button_{"RESET"};
    juce::TextButton theme_button_{"AQUA"};
    CurvesKnobLook knob_look_;
    int theme_index_ = 0;
    float display_scale_ = 1.0f;
    int active_count_ = 128;
    bool drawing_ = false;
    bool dirty_ = false;
    int last_mode_ = -1;  // physics mode the cloth panel was last styled for
    Knob* test_knob_ = nullptr;   // Test Src, so its label can name the texture
    // Randomisers: small dials parked on the top-right of Wind and Flutter Hz.
    SeedSlider rnd_wind_, rnd_flutter_;
    Knob* wind_knob_ = nullptr;
    Knob* flutter_knob_ = nullptr;
    juce::Rectangle<int> rnd_wind_box_, rnd_flutter_box_;
    void setup_randomiser(SeedSlider&, const char* param_id, bool is_wind);
    int last_test_src_ = -1;
    // Wall being dragged on the canvas: -1 none, 0 low edge, 1 high edge.
    int dragging_wall_ = -1;
    float wall_x(int which) const;
    int wall_at(juce::Point<int> p) const;
    void set_wall(int which, float x_px);
    float last_node_ = 0.f, last_val_ = 0.f;

    std::vector<std::unique_ptr<Knob>> knobs_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(CurvesAudioProcessorEditor)
};
