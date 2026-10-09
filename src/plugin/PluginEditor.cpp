#include "PluginEditor.h"

#include <algorithm>
#include <cmath>

// ---- themes ---------------------------------------------------------------
// Aqua first: neon aqua / electric blue. The rest keep the same value
// structure (dark ground, one saturated accent) so the layout reads
// identically whichever is picked.
static const CurvesTheme kThemes[] = {
    // Fields, in order (all 15 must be present - an aggregate initializer
    // that is one short silently shifts every later field and leaves the
    // last one zero, i.e. TRANSPARENT; that is exactly how every text_dim
    // label in this editor went invisible once):
    //   bg, plot_top, plot_bot, panel, panel_edge,
    //   grid, grid_key, curve, glow, fill,
    //   sim_ref, draw, anchor, text, text_dim
    {"AQUA",    0xff070a0f, 0xff0d1620, 0xff070a0f, 0xff111823, 0xff1d2b3a,
                0xff1b2735, 0xff2c4a63, 0xff5bf7ff, 0xff00c8ff, 0xff00c8ff,
                0xff2e7d96, 0xffffb347, 0xffffd166, 0xffdff6ff, 0xff8fa8bd},
    {"MAGENTA", 0xff0b060d, 0xff1a0c1d, 0xff0b060d, 0xff1b0f20, 0xff3a1d3d,
                0xff2b1730, 0xff5e2a63, 0xffff6bf0, 0xffe000c8, 0xffe000c8,
                0xff8f3a86, 0xff7ce0ff, 0xffffd166, 0xffffe3fb, 0xffc296ba},
    {"AMBER",   0xff0c0905, 0xff1d1508, 0xff0c0905, 0xff1d1709, 0xff3d2f14,
                0xff2f2411, 0xff63491f, 0xffffc94d, 0xffff8a00, 0xffff8a00,
                0xff8f6a2e, 0xff6bdcff, 0xff6bdcff, 0xfffff0d6, 0xffc2aa85},
    {"LIME",    0xff070c06, 0xff0e1a0b, 0xff070c06, 0xff111d0f, 0xff213d1c,
                0xff1b2f18, 0xff3f6330, 0xffb6ff57, 0xff63d400, 0xff63d400,
                0xff5c8f3a, 0xffffa8f0, 0xffffd166, 0xffeaffd9, 0xffa3c290},
    {"VIOLET",  0xff08070f, 0xff120f22, 0xff08070f, 0xff14122a, 0xff2a2555,
                0xff201c40, 0xff443a7d, 0xffb49cff, 0xff6a3dff, 0xff6a3dff,
                0xff5a4f99, 0xff7cffd4, 0xffffd166, 0xffe8e2ff, 0xffa49ec2},
};

const CurvesTheme& curves_theme(int index) {
    const int n = (int)(sizeof(kThemes) / sizeof(kThemes[0]));
    const CurvesTheme& t = kThemes[juce::jlimit(0, n - 1, index)];
    // Guard the short-initializer trap described above: a missing field
    // leaves the tail zeroed, and a zero colour is fully transparent rather
    // than black, so it fails silently and invisibly.
    jassert(t.text_dim != 0 && t.text != 0 && t.draw != 0);
    return t;
}
int curves_theme_count() { return (int)(sizeof(kThemes) / sizeof(kThemes[0])); }

// ---- rotary look ----------------------------------------------------------

void CurvesKnobLook::drawRotarySlider(juce::Graphics& g, int x, int y, int w,
                                      int h, float pos, float start, float end,
                                      juce::Slider& s) {
    const CurvesTheme& t = theme_ ? *theme_ : curves_theme(0);
    auto area = juce::Rectangle<int>(x, y, w, h).toFloat().reduced(3.0f);
    const float radius = std::min(area.getWidth(), area.getHeight()) * 0.5f;
    const auto centre = area.getCentre();
    const float thick = juce::jmax(3.0f, radius * 0.30f);
    const float angle = start + pos * (end - start);
    const float enabled = s.isEnabled() ? 1.0f : 0.35f;

    juce::Path track;
    track.addCentredArc(centre.x, centre.y, radius - thick * 0.5f,
                        radius - thick * 0.5f, 0.0f, start, end, true);
    g.setColour(juce::Colour(t.panel_edge).withMultipliedAlpha(enabled));
    g.strokePath(track, juce::PathStrokeType(thick, juce::PathStrokeType::curved,
                                             juce::PathStrokeType::rounded));

    if (pos > 0.0005f) {
        juce::Path value;
        value.addCentredArc(centre.x, centre.y, radius - thick * 0.5f,
                            radius - thick * 0.5f, 0.0f, start, angle, true);
        // glow pass, then the bright core
        g.setColour(juce::Colour(t.glow).withAlpha(0.30f * enabled));
        g.strokePath(value, juce::PathStrokeType(thick * 2.0f,
                                                 juce::PathStrokeType::curved,
                                                 juce::PathStrokeType::rounded));
        g.setColour(juce::Colour(t.curve).withMultipliedAlpha(enabled));
        g.strokePath(value, juce::PathStrokeType(thick,
                                                 juce::PathStrokeType::curved,
                                                 juce::PathStrokeType::rounded));
    }

    const float inner = radius - thick - 1.0f;
    const juce::Point<float> tip(centre.x + std::sin(angle) * inner,
                                 centre.y - std::cos(angle) * inner);
    const juce::Point<float> base(centre.x + std::sin(angle) * inner * 0.35f,
                                  centre.y - std::cos(angle) * inner * 0.35f);
    g.setColour(juce::Colour(t.text).withMultipliedAlpha(enabled));
    g.drawLine({base, tip}, juce::jmax(1.5f, radius * 0.09f));
}

// ---- construction ---------------------------------------------------------

CurvesAudioProcessorEditor::CurvesAudioProcessorEditor(CurvesAudioProcessor& p)
    : AudioProcessorEditor(p), processor_(p) {
    theme_index_ = processor_.ui_theme();
    setSize(1000, 784);   // +84 for the second SUBSCRIBERS row
    setup_randomiser(rnd_wind_, "wind_rnd", true);
    setup_randomiser(rnd_flutter_, "flutter_rnd", false);

    active_count_ = processor_.active_nodes();
    startTimerHz(30);

    // Controls, grouped by SCOPE. GLOBAL is always in effect; STRING is the
    // physics that always applies; CLOTH & WIND is Flagpole-only.
    const struct { const char* id; const char* name; int group; } defs[] = {
        // GLOBAL -- and GLOBAL now means it. Physics, Smooth and Steady
        // used to sit here, but they belong to ONE voice, and leaving
        // per-voice knobs in a panel labelled GLOBAL would be a lie as soon
        // as the knobs became voice-paged.
        {"precision", "Precision", kGlobal},
        {"leveler", "Leveler", kGlobal},
        {"mix", "Mix", kGlobal},            {"test", "Test Src", kGlobal},
        // STRING -- the physics that always applies, per voice
        {"physics", "Physics", kString},
        {"damping", "Damping", kString},    {"stiffness", "Stiffness", kString},
        {"tension", "Tension", kString},    {"velocity", "Velocity", kString},
        // CLOTH & WIND
        {"motion", "Motion", kCloth},       {"wind", "Wind", kCloth},
        {"character", "Character", kCloth}, {"flutter_hz", "Flutter Hz", kCloth},
        {"waves", "Waves", kCloth},         {"wrinkle", "Wrinkle", kCloth},
        {"flap", "Flap", kCloth},           {"direction", "Direction", kCloth},
        // SUBSCRIBERS - what the one control signal drives. Smoothing is
        // the control smoother and Steady is the bank's dynamic EQ, so both
        // describe how the signal drives the subscribers rather than how
        // the string moves.
        {"drive", "EQ dB", kSubscriber},    {"smoothing", "Smooth", kSubscriber},
        {"steady", "Steady", kSubscriber},  {"width", "Width", kSubscriber},
        {"pan_depth", "Place", kSubscriber},
        {"stereo_lo", "Low Hz", kSubscriber},
        {"stereo_hi", "High Hz", kSubscriber},
        // Band Q is a property of the BANK, so it applies to every
        // subscriber at once, not just the EQ.
        {"q_mode", "Band Q", kSubscriber},
        {"resonance", "Resonance", kSubscriber},
    };
    for (const auto& d : defs) {
        for (auto* p2 : processor_.getParameters()) {
            auto* rp = dynamic_cast<juce::RangedAudioParameter*>(p2);
            if (rp == nullptr || rp->paramID != juce::String(d.id))
                continue;
            auto knob = std::make_unique<Knob>();
            knob->base_id = d.id;
            {
                int nvp = 0;
                const char* const* ids =
                    CurvesAudioProcessor::voice_param_ids(nvp);
                for (int i = 0; i < nvp; ++i)
                    if (juce::String(ids[i]) == juce::String(d.id)) {
                        knob->per_voice = true;
                        break;
                    }
            }
            const auto& range = rp->getNormalisableRange();
            // Discrete params (Physics/Precision/Direction) carry interval 1;
            // continuous ones must stay continuous.
            if (range.interval > 0.0f) {
                knob->slider.setRange(range.start, range.end, range.interval);
                // A stepped knob's travel must scale with how many STEPS it
                // has, not be the same 250 px JUCE gives a continuous one.
                // Band Q is a 2-position switch: with the default it needed
                // 125 px of vertical drag to cross the snap point -- about a
                // third of the window -- so a normal knob-sized drag moved
                // it part way and it snapped back. It read as broken, and
                // Direction (also 0..1) had it too.
                const int steps = juce::jmax(
                    1, (int)std::lround((range.end - range.start)
                                        / juce::jmax(1.0e-6f, range.interval)));
                knob->slider.setMouseDragSensitivity(steps * 34);
            } else
                knob->slider.setNormalisableRange(
                    juce::NormalisableRange<double>(range.start, range.end));
            knob->slider.setValue(rp->convertFrom0to1(rp->getValue()),
                                  juce::dontSendNotification);
            knob->slider.setSliderStyle(juce::Slider::RotaryVerticalDrag);
            knob->slider.setTextBoxStyle(juce::Slider::NoTextBox, true, 0, 0);
            knob->slider.setLookAndFeel(&knob_look_);
            knob->slider.addListener(this);
            knob->label.setText(d.name, juce::dontSendNotification);
            knob->label.setJustificationType(juce::Justification::centred);
            knob->label.setFont(juce::Font(12.0f));
            knob->label.setInterceptsMouseClicks(false, false);
            knob->param = rp;
            knob->group = d.group;
            for (int k = 0; k < CurvesAudioProcessor::kNumModKnobs; ++k)
                if (juce::String(CurvesAudioProcessor::mod_knob_id(k)) == rp->paramID) {
                    knob->mod = k;
                    break;
                }
            addAndMakeVisible(knob->slider);
            addAndMakeVisible(knob->label);
            const bool is_test = (rp->paramID == "test");
            const bool is_wind = (rp->paramID == "wind");
            const bool is_flut = (rp->paramID == "flutter_hz");
            knobs_.push_back(std::move(knob));
            if (is_test)
                test_knob_ = knobs_.back().get();
            if (is_wind)
                wind_knob_ = knobs_.back().get();
            if (is_flut)
                flutter_knob_ = knobs_.back().get();
            break;
        }
    }

    for (auto* b : {&scale_button_, &theme_button_, &reset_button_,
                    &dual_button_, &link_button_, &voice_l_, &voice_r_})
        addAndMakeVisible(*b);
    // Click to enable. Turning it on copies this voice onto the other and
    // re-rolls its seeds, so the right channel starts as the same weather
    // with different gusts -- audibly stereo at once, still the same patch.
    dual_button_.onClick = [this] {
        const bool now = !processor_.dual_on();
        processor_.set_dual(now);
        if (!now)
            processor_.set_ui_voice(0);
        rebind_voice();
        refresh_voice_buttons();
        repaint();
    };
    // Switching Link ON mirrors the voice you are looking at onto the other,
    // seeds included -- otherwise "linked" would still show two sounds.
    link_button_.onClick = [this] {
        const bool now = !processor_.link_on();
        processor_.set_link(now);
        rebind_voice();
        refresh_voice_buttons();
        repaint();
    };
    voice_l_.onClick = [this] {
        processor_.set_ui_voice(0);
        rebind_voice();
        refresh_voice_buttons();
        repaint();
    };
    voice_r_.onClick = [this] {
        // Picking R with Dual off would show a voice that is not running.
        if (!processor_.dual_on())
            processor_.set_dual(true);
        processor_.set_ui_voice(1);
        rebind_voice();
        refresh_voice_buttons();
        repaint();
    };
    // Flattens the string and leaves every modifier running, so the cloth
    // restarts from rest. Replaces winding Damping up to kill energy.
    reset_button_.onClick = [this] { processor_.request_reset(); };
    scale_button_.onClick = [this] { cycle_display_scale(); };
    theme_button_.onClick = [this] { cycle_theme(); };

    rebind_voice();        // a reopened editor may be showing voice 1
    apply_theme();
    resized();  // setSize() above ran before the knobs existed
}

CurvesAudioProcessorEditor::~CurvesAudioProcessorEditor() {
    stopTimer();
    // Sliders must not outlive the LookAndFeel they point at.
    for (auto& k : knobs_)
        k->slider.setLookAndFeel(nullptr);
    rnd_wind_.setLookAndFeel(nullptr);
    rnd_flutter_.setLookAndFeel(nullptr);
}

void CurvesAudioProcessorEditor::cycle_display_scale() {
    static constexpr float kScales[4] = {0.5f, 1.0f, 2.0f, 4.0f};
    static const char* kLabels[4] = {"+-0.5", "+-1", "+-2", "+-4"};
    const float* it = std::find(std::begin(kScales), std::end(kScales), display_scale_);
    const int idx = (it == std::end(kScales) - 1) ? 0 : int(it - std::begin(kScales)) + 1;
    display_scale_ = kScales[idx];
    scale_button_.setButtonText(kLabels[idx]);
    repaint();
}

void CurvesAudioProcessorEditor::cycle_theme() {
    theme_index_ = (theme_index_ + 1) % curves_theme_count();
    processor_.set_ui_theme(theme_index_);
    apply_theme();
}

void CurvesAudioProcessorEditor::apply_theme() {
    const auto& t = theme();
    knob_look_.set_theme(&t);
    theme_button_.setButtonText(t.name);
    for (auto* b : {&scale_button_, &theme_button_, &reset_button_,
                    &dual_button_, &link_button_, &voice_l_, &voice_r_}) {
        b->setColour(juce::TextButton::buttonColourId, juce::Colour(t.panel));
        b->setColour(juce::TextButton::textColourOffId, juce::Colour(t.curve));
    }
    for (auto& k : knobs_)
        k->label.setColour(juce::Label::textColourId, juce::Colour(t.text_dim));
    repaint();    // Last: the plain button colours above would otherwise wipe the
    // selected-voice and Link highlights.
    refresh_voice_buttons();

}

// ---- layout ---------------------------------------------------------------
// Panel heights live here as named constants because plot_bounds() has to
// subtract exactly the same amounts; keeping them in one place stops the
// canvas and the panels from overlapping when one of them changes.
namespace {
constexpr int kHeaderH = 34;
constexpr int kHintH = 24;
// Two rows now. SUBSCRIBERS carries 9 controls since Smooth, Steady, Band Q
// and Resonance joined it, and nine across at this width put the power and
// magnitude LEDs straight through the labels ("Steady" rendered as a blob,
// "Resonance" as "esonan"). Two rows of five is the readable shape.
constexpr int kGlobalH = 188;
constexpr int kPhysicsH = 188;
constexpr int kPanelGap = 10;
}  // namespace

void CurvesAudioProcessorEditor::setup_randomiser(SeedSlider& sl,
                                                  const char* param_id,
                                                  bool is_wind) {
    for (auto* p : processor_.getParameters()) {
        auto* rp = dynamic_cast<juce::RangedAudioParameter*>(p);
        if (rp == nullptr || rp->paramID != juce::String(param_id))
            continue;
        sl.setNormalisableRange(juce::NormalisableRange<double>(0.0, 1.0));
        sl.setValue(rp->convertFrom0to1(rp->getValue()),
                    juce::dontSendNotification);
        sl.setSliderStyle(juce::Slider::RotaryVerticalDrag);
        sl.setTextBoxStyle(juce::Slider::NoTextBox, true, 0, 0);
        sl.setLookAndFeel(&knob_look_);
        sl.addListener(this);
        sl.setTooltip("drag: how much randomisation   |   click: new seed");
        sl.on_reroll = [this, is_wind] {
            if (is_wind)
                processor_.reroll_wind_seed();
            else
                processor_.reroll_flutter_seed();
            repaint();
        };
        addAndMakeVisible(sl);
        break;
    }
}

void CurvesAudioProcessorEditor::resized() {
    // Laid out right-to-left with explicit gaps. Hand-picked offsets had
    // LINK sitting on top of RESET -- it rendered as "ESET".
    {
        const int y = 12, h = 24, gap = 6;
        int right = getWidth() - 12;
        auto put = [&](juce::Component& c, int w) {
            c.setBounds(right - w, y, w, h);
            right -= w + gap;
        };
        put(scale_button_, 68);
        put(theme_button_, 68);
        put(reset_button_, 68);
        put(link_button_, 62);
        put(dual_button_, 62);
        put(voice_r_, 26);
        put(voice_l_, 26);
    }
    if (knobs_.empty())
        return;  // setSize() runs before the knobs are created in the ctor

    auto area = getLocalBounds().reduced(14, 12);
    area.removeFromTop(kHeaderH);

    // Bottom-up: GLOBAL gets its own full-width row (the "one obvious
    // spot"), with STRING and CLOTH & WIND sharing the row above it.
    // Bottom row carries GLOBAL and SUBSCRIBERS side by side. GLOBAL is
    // down to 4 real globals and SUBSCRIBERS up to 9, so split 4:13.
    {
        auto bottom = area.removeFromBottom(kGlobalH);
        const int gw = (bottom.getWidth() - kPanelGap) * 2 / 7;
        panel_[kGlobal] = bottom.removeFromLeft(gw);
        bottom.removeFromLeft(kPanelGap);
        panel_[kSubscriber] = bottom;
    }
    area.removeFromBottom(kPanelGap);
    auto physics_row = area.removeFromBottom(kPhysicsH);

    // STRING holds 4 controls, CLOTH holds 8, so split 1:2.
    const int string_w = (physics_row.getWidth() - kPanelGap) / 3;
    panel_[kString] = physics_row.removeFromLeft(string_w);
    physics_row.removeFromLeft(kPanelGap);
    panel_[kCloth] = physics_row;

    auto place = [this](int group, juce::Rectangle<int> box, int cols) {
        std::vector<Knob*> in_group;
        for (auto& k : knobs_)
            if (k->group == group)
                in_group.push_back(k.get());
        if (in_group.empty())
            return;
        box = box.reduced(10, 8);
        box.removeFromTop(16);  // panel title
        const int rows = (int)((in_group.size() + cols - 1) / cols);
        const int cell_h = box.getHeight() / juce::jmax(1, rows);
        const int cell_w = box.getWidth() / juce::jmax(1, cols);
        for (size_t i = 0; i < in_group.size(); ++i) {
            const int r = (int)i / cols, c = (int)i % cols;
            juce::Rectangle<int> cell(box.getX() + c * cell_w,
                                      box.getY() + r * cell_h, cell_w, cell_h);
            const int side = juce::jmax(20, juce::jmin(cell.getWidth() - 10, cell_h - 20));
            in_group[i]->slider.setBounds(
                cell.getCentreX() - side / 2, cell.getY() + 2, side, side);
            in_group[i]->label.setBounds(cell.getX(), cell.getY() + side + 2,
                                         cell.getWidth(), 16);
            // LEDs sit on the LABEL row, hard left and hard right of the
            // cell: clear of the rotary arc (which they overlapped when
            // placed on the knob) and as far apart as the cell allows, so
            // magnitude-orange is not mistaken for power-yellow.
            // Inset well away from the cell edges: flush to the edge, the
            // power LED of one knob sat against the next knob's magnitude
            // LED and the pair read as one control.
            const int led = 10;
            const int ly = cell.getY() + side + 5;
            const int inset = juce::jmax(12, cell_w / 7);
            in_group[i]->magnitude = {cell.getX() + inset, ly, led, led};
            in_group[i]->power = {cell.getRight() - inset - led, ly, led, led};
        }
    };
    // GLOBAL holds 8 controls since Mix was added. Keep it ONE row -- it
    // is the "one obvious spot", and a second row made Test Noise wrap
    // out of the panel's height and clip.
    place(kGlobal, panel_[kGlobal], 2);
    place(kSubscriber, panel_[kSubscriber], 5);
    place(kString, panel_[kString], 3);
    place(kCloth, panel_[kCloth], 4);

    // Randomisers sit on the top-right corner of the knob they belong to,
    // overlapping it slightly so the pairing reads at a glance.
    auto park = [](SeedSlider& sl, Knob* host, juce::Rectangle<int>& box) {
        if (host == nullptr)
            return;
        const auto hb = host->slider.getBounds();
        // Half the host's diameter. A third was tried first and rendered as
        // a 13 px ring with a 3 px stroke -- present, but far too small to
        // aim at, which matters because a CLICK on it re-rolls the seed.
        const int d = juce::jmax(26, hb.getWidth() / 2);
        box = {hb.getRight() - 8, hb.getY() - 6, d, d};
        sl.setBounds(box);
    };
    park(rnd_wind_, wind_knob_, rnd_wind_box_);
    park(rnd_flutter_, flutter_knob_, rnd_flutter_box_);
}

// ---- geometry -------------------------------------------------------------

// The whole plot region, before it is split between voices.
static constexpr float kPlotGap = 8.0f;

juce::Rectangle<float> CurvesAudioProcessorEditor::plot_bounds(int voice) const {
    auto area = getLocalBounds().reduced(14, 12);
    area.removeFromTop(kHeaderH);
    area.removeFromBottom(kGlobalH + kPanelGap + kPhysicsH + kHintH);
    auto f = area.toFloat();
    if (!processor_.dual_on())
        return f;
    // Stacked, L above R, so the pair reads top-to-bottom the way the
    // channels are usually listed.
    const float h = (f.getHeight() - kPlotGap) * 0.5f;
    return voice == 0 ? f.withHeight(h)
                      : f.withTop(f.getY() + h + kPlotGap).withHeight(h);
}

// Everything geometric (node_to_x, value_to_y, the walls, drawing) resolves
// through this, so it answers for the voice being EDITED. That is what keeps
// a second canvas from needing a second copy of the interaction code.
juce::Rectangle<float> CurvesAudioProcessorEditor::plot_bounds() const {
    return plot_bounds(processor_.ui_voice());
}

int CurvesAudioProcessorEditor::plot_at(juce::Point<float> p) const {
    const int nv = processor_.dual_on() ? CurvesAudioProcessor::kNumVoices : 1;
    for (int v = 0; v < nv; ++v)
        if (plot_bounds(v).contains(p))
            return v;
    return -1;
}

float CurvesAudioProcessorEditor::node_to_x(float node) const {
    const auto b = plot_bounds();
    return b.getX() + (node / float(active_nodes() - 1)) * b.getWidth();
}

float CurvesAudioProcessorEditor::x_to_node(float x) const {
    const auto b = plot_bounds();
    return juce::jmap(x, b.getX(), b.getRight(), 0.0f, float(active_nodes() - 1));
}

float CurvesAudioProcessorEditor::value_to_y(float v) const {
    const auto b = plot_bounds();
    const float half = b.getHeight() * 0.5f;
    return b.getCentreY() - (v / display_scale_) * half;
}

float CurvesAudioProcessorEditor::y_to_value(float y) const {
    const auto b = plot_bounds();
    const float half = std::max(b.getHeight() * 0.5f, 1.0f);
    return juce::jlimit(-1.0f, 1.0f, ((b.getCentreY() - y) / half) * display_scale_);
}

// ---- walls ---------------------------------------------------------------
// Two draggable boundaries on the canvas. Outside them the string is held at
// rest, so any window anywhere along the spectrum can be the active cloth.

float CurvesAudioProcessorEditor::wall_x(int which) const {
    const auto b = plot_bounds();
    // Walls are PER VOICE. Looking them up by the bare id read voice 0's
    // walls while editing voice 1 -- the posts would not have matched the
    // string under them.
    const juce::String id = CurvesAudioProcessor::voice_param_id(
        which == 0 ? "wall_lo" : "wall_hi", processor_.ui_voice());
    for (auto* p : processor_.getParameters())
        if (auto* rp = dynamic_cast<juce::RangedAudioParameter*>(p))
            if (rp->paramID == id)
                return b.getX() + rp->convertFrom0to1(rp->getValue()) * b.getWidth();
    return b.getX();
}

void CurvesAudioProcessorEditor::set_wall(int which, float x_px) {
    const auto b = plot_bounds();
    const float f = juce::jlimit(0.0f, 1.0f, (x_px - b.getX()) / b.getWidth());
    write_voice_param(which == 0 ? "wall_lo" : "wall_hi", f);
    repaint();
}

int CurvesAudioProcessorEditor::wall_at(juce::Point<int> p) const {
    const auto b = plot_bounds();
    // Expanded HORIZONTALLY as well as vertically. A wall parked at an edge
    // sits exactly on the plot boundary, so with a vertical-only expansion
    // half its grab zone fell outside the test and clicks just beyond the
    // edge -- the natural place to aim -- did nothing. That read as the wall
    // being undraggable (user: "we lost the ability to drag the left side
    // wall in"). It was always reachable, just only from one side, and the
    // dB axis labels now sit on exactly that spot.
    if (!b.expanded(18.0f, 10.0f).contains(p.toFloat()))
        return -1;
    // Grab whichever edge is within reach; the nearer one wins a tie.
    const float dlo = std::abs(float(p.x) - wall_x(0));
    const float dhi = std::abs(float(p.x) - wall_x(1));
    const float reach = 18.0f;
    if (dlo > reach && dhi > reach)
        return -1;
    return (dlo <= dhi) ? 0 : 1;
}

// ---- drawing (the user holds the string) ---------------------------------

void CurvesAudioProcessorEditor::mouseDown(const juce::MouseEvent& e) {
    // LEDs first: they sit over the knob cells, and the sliders are separate
    // child components so these hit areas never overlap a live slider.
    if (!e.mods.isRightButtonDown()) {
        // getEventRelativeTo(this): e.getPosition() is relative to the
        // component the event originated on, which is ambiguous when a click
        // passes through a non-intercepting child (the labels do).
        const auto pt = e.getEventRelativeTo(this).getPosition();
        for (auto& k : knobs_) {
            if (k->mod < 0 || !k->slider.isEnabled())
                continue;
            if (CurvesAudioProcessor::power_mode(k->mod)
                    != CurvesAudioProcessor::kPowerNone
                && k->power.expanded(4).contains(pt)) {
                processor_.set_knob_on(k->mod, !processor_.knob_on(k->mod));
                repaint();
                return;
            }
            if (CurvesAudioProcessor::has_magnitude(k->mod)
                && k->magnitude.expanded(4).contains(pt)) {
                processor_.set_knob_mag(k->mod,
                                        (processor_.knob_mag(k->mod) + 1) % 3);
                repaint();
                return;
            }
        }
    }
    if (e.mods.isRightButtonDown()) {
        draw_nodes_.fill(0.0f);
        push_current();
        return;
    }
    // A wall under the cursor takes precedence over drawing.
    {
        const auto pt = e.getEventRelativeTo(this).getPosition();
        dragging_wall_ = wall_at(pt);
        if (dragging_wall_ >= 0) {
            set_wall(dragging_wall_, float(pt.x));   // click-to-position
            return;
        }
    }
    // Clicking the dimmed canvas HIGHLIGHTS that voice rather than drawing
    // on it. Switching first and drawing on the same click would deposit a
    // stroke on a string you had not chosen to edit yet.
    {
        const int hit = plot_at(e.position.toFloat());
        if (hit >= 0 && hit != processor_.ui_voice()) {
            processor_.set_ui_voice(hit);
            rebind_voice();
            refresh_voice_buttons();
            repaint();
            return;
        }
    }
    if (!plot_bounds().contains(e.position.toFloat()))
        return;
    drawing_ = true;
    // Deposit onto the string's *current* state, not a stale buffer.
    std::array<float, CurvesAudioProcessor::kMaxNodes> eq_snap{};
    processor_.pull_display(processor_.ui_voice(), draw_nodes_, eq_snap,
                            active_count_);
    last_node_ = juce::jlimit(0.0f, float(active_nodes() - 1), x_to_node(e.position.x));
    last_val_ = y_to_value(e.position.y);
    deposit_stroke(last_node_, last_val_, last_node_, last_val_);
    push_current();
}

void CurvesAudioProcessorEditor::mouseDrag(const juce::MouseEvent& e) {
    if (dragging_wall_ >= 0) {
        set_wall(dragging_wall_,
                 float(e.getEventRelativeTo(this).getPosition().x));
        return;
    }
    if (!drawing_)
        return;
    const float node = juce::jlimit(0.0f, float(active_nodes() - 1), x_to_node(e.position.x));
    const float val = y_to_value(e.position.y);
    deposit_stroke(last_node_, last_val_, node, val);
    last_node_ = node;
    last_val_ = val;
    push_current();
}

void CurvesAudioProcessorEditor::mouseUp(const juce::MouseEvent&) {
    dragging_wall_ = -1;
    drawing_ = false;  // release: the string rings from the held shape
}

void CurvesAudioProcessorEditor::deposit_stroke(float n0, float v0, float n1, float v1) {
    const int lo = std::max(0, int(std::floor(std::min(n0, n1))));
    const int hi = std::min(active_nodes() - 1, int(std::ceil(std::max(n0, n1))));
    for (int i = lo; i <= hi; ++i) {
        const float t = std::abs(n1 - n0) > 1e-6f ? (float(i) - n0) / (n1 - n0) : 1.0f;
        draw_nodes_[i] = juce::jlimit(-1.0f, 1.0f, juce::jmap(t, v0, v1));
    }
    dirty_ = true;
}

void CurvesAudioProcessorEditor::push_current() {
    processor_.push_draw(draw_nodes_, active_nodes());
}

// ---- physics presets (knob values + engine mode) --------------------------

// One set of widgets, two sets of parameters. Switching voice re-points
// every per-voice knob rather than building a second panel: the ranges are
// identical by construction (both sets come from add_voice_params), so only
// the pointer and the displayed value change.
void CurvesAudioProcessorEditor::rebind_voice() {
    const int voice = processor_.ui_voice();
    for (auto& k : knobs_) {
        if (!k->per_voice || k->base_id.isEmpty())
            continue;
        const juce::String want =
            CurvesAudioProcessor::voice_param_id(k->base_id.toRawUTF8(), voice);
        for (auto* p2 : processor_.getParameters()) {
            auto* rp = dynamic_cast<juce::RangedAudioParameter*>(p2);
            if (rp == nullptr || rp->paramID != want)
                continue;
            k->param = rp;
            k->slider.setValue(rp->convertFrom0to1(rp->getValue()),
                               juce::dontSendNotification);
            break;
        }
    }
}

juce::String CurvesAudioProcessorEditor::header_overlap_for_test() const {
    const struct { const juce::Component* c; const char* n; } b[] = {
        {&scale_button_, "SCALE"}, {&theme_button_, "THEME"},
        {&reset_button_, "RESET"}, {&link_button_, "LINK"},
        {&dual_button_, "DUAL"},   {&voice_r_, "R"}, {&voice_l_, "L"}};
    const int n = (int)(sizeof(b) / sizeof(b[0]));
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            if (b[i].c->getBounds().intersects(b[j].c->getBounds()))
                return juce::String(b[i].n) + " over " + b[j].n;
    return {};
}

int CurvesAudioProcessorEditor::bound_voice_of(const char* base) const {
    for (const auto& k : knobs_) {
        if (k->base_id != juce::String(base) || k->param == nullptr)
            continue;
        for (int v = 0; v < CurvesAudioProcessor::kNumVoices; ++v)
            if (k->param->paramID
                == CurvesAudioProcessor::voice_param_id(base, v))
                return v;
        return -1;
    }
    return -1;
}

bool CurvesAudioProcessorEditor::nudge_knob(const char* base, float value) {
    for (auto& k : knobs_) {
        if (k->base_id != juce::String(base))
            continue;
        k->slider.setValue(value, juce::sendNotificationSync);
        return true;
    }
    return false;
}

void CurvesAudioProcessorEditor::refresh_voice_buttons() {
    const auto& t = theme();
    const bool dual = processor_.dual_on();
    const int voice = processor_.ui_voice();
    dual_button_.setColour(juce::TextButton::buttonColourId,
                           juce::Colour(dual ? t.curve : t.panel)
                               .withMultipliedAlpha(dual ? 0.55f : 1.0f));
    dual_button_.setColour(juce::TextButton::textColourOffId,
                           juce::Colour(dual ? t.bg : t.curve));
    // Link is only meaningful with two voices running, so it dims with Dual
    // off rather than offering to mirror a voice nothing is driving.
    const bool link = processor_.link_on();
    link_button_.setColour(juce::TextButton::buttonColourId,
                           juce::Colour(link ? t.curve : t.panel)
                               .withMultipliedAlpha(link ? 0.55f
                                                         : (dual ? 1.0f : 0.4f)));
    link_button_.setColour(juce::TextButton::textColourOffId,
                           juce::Colour(link ? t.bg : t.curve)
                               .withMultipliedAlpha(dual ? 1.0f : 0.5f));
    // The selected voice is HIGHLIGHTED; with Dual off, R is dimmed because
    // there is no second string running to look at.
    for (int i = 0; i < 2; ++i) {
        auto* b = i == 0 ? &voice_l_ : &voice_r_;
        const bool sel = (voice == i);
        const bool live = (i == 0) || dual;
        b->setColour(juce::TextButton::buttonColourId,
                     juce::Colour(sel ? t.curve : t.panel)
                         .withMultipliedAlpha(sel ? 0.55f : (live ? 1.0f : 0.4f)));
        b->setColour(juce::TextButton::textColourOffId,
                     juce::Colour(sel ? t.bg : t.curve)
                         .withMultipliedAlpha(live ? 1.0f : 0.5f));
    }
}

void CurvesAudioProcessorEditor::apply_physics_preset(int mode) {
    // Writes to the voice currently being edited: with R selected, these
    // must land on "damping_r" and not on voice 0's "damping".
    auto set_param = [this](const char* id, float v) {
        write_voice_param(id, v);
        for (auto& k : knobs_)
            if (k->base_id == juce::String(id)) {
                k->slider.setValue(v, juce::dontSendNotification);
                break;
            }
    };
    switch (mode) {
        case 1:  // Flagpole: loose, slow settling, moderate tension
            set_param("damping", 1.2f);
            set_param("stiffness", 2.0f);
            set_param("tension", 300.0f);
            break;
        case 2:  // Gust: neutral defaults
            set_param("damping", 1.5f);
            set_param("stiffness", 4.0f);
            set_param("tension", 300.0f);
            break;
        case 3:  // Elastic: tight spring-back
            set_param("damping", 2.0f);
            set_param("stiffness", 12.0f);
            set_param("tension", 300.0f);
            break;
        default:  // Free: leave knobs alone
            break;
    }
}

// ---- display --------------------------------------------------------------

void CurvesAudioProcessorEditor::timerCallback() {
    // Both voices every frame: in Dual both canvases are live, and pulling
    // only the highlighted one would leave the other frozen.
    const int nv = processor_.dual_on() ? CurvesAudioProcessor::kNumVoices : 1;
    for (int v = 0; v < nv; ++v)
        processor_.pull_display(v, display_nodes_[v], display_eq_[v],
                                active_count_);
    if (!drawing_) {
        draw_nodes_ = display_nodes_[processor_.ui_voice()];
        dirty_ = false;
    }
    // Cloth & Wind only does anything in Flagpole, so grey it out elsewhere
    // rather than leaving nineteen live-looking knobs on screen.
    // Test Src is a six-position switch; name the position on its label so
    // the knob angle is not the only indication of which texture is live.
    if (test_knob_ != nullptr) {
        const int src = juce::jlimit(
            0, CurvesAudioProcessor::kNumTestSrc - 1,
            (int)std::lround(test_knob_->slider.getValue()));
        if (src != last_test_src_) {
            last_test_src_ = src;
            test_knob_->label.setText(
                juce::String("Test: ")
                    + CurvesAudioProcessor::test_src_name(src),
                juce::dontSendNotification);
        }
    }
    const int mode = processor_.physics_mode();
    if (mode != last_mode_) {
        last_mode_ = mode;
        const bool cloth_live = (mode == 1);
        for (auto& k : knobs_)
            if (k->group == kCloth) {
                k->slider.setEnabled(cloth_live);
                k->label.setAlpha(cloth_live ? 1.0f : 0.4f);
            }
        rnd_wind_.setEnabled(cloth_live);
        rnd_flutter_.setEnabled(cloth_live);
    }
    // In Dual, each voice sees ONE channel, so its side signal is zero by
    // construction and Width and Place can do nothing. Dimming them says so
    // rather than leaving two knobs that look live and are not -- the
    // "knob that does nothing" pattern, avoided for once in advance.
    const bool dual = processor_.dual_on();
    if (dual != last_dual_) {
        last_dual_ = dual;
        for (auto& k : knobs_)
            if (k->base_id == "width" || k->base_id == "pan_depth") {
                k->slider.setEnabled(!dual);
                k->label.setAlpha(dual ? 0.4f : 1.0f);
            }
        refresh_voice_buttons();
    }
    repaint();
}

// EVERY parameter write from the UI goes through here. It resolves the
// base id against the voice being edited, and under Link writes the other
// voice too. One route means a new control cannot quietly acquire the
// wrong-voice bug: the randomisers had exactly that -- they sat outside
// knobs_ and wrote the bare "wind_rnd", so they always moved voice 0.
void CurvesAudioProcessorEditor::write_voice_param(const char* base,
                                                   float value) {
    auto put = [this, value](const juce::String& id) {
        for (auto* p : processor_.getParameters())
            if (auto* rp = dynamic_cast<juce::RangedAudioParameter*>(p))
                if (rp->paramID == id) {
                    rp->beginChangeGesture();
                    rp->setValueNotifyingHost(rp->convertTo0to1(value));
                    rp->endChangeGesture();
                    return;
                }
    };
    put(CurvesAudioProcessor::voice_param_id(base, processor_.ui_voice()));
    const int m = processor_.mirror_voice();
    if (m >= 0)
        put(CurvesAudioProcessor::voice_param_id(base, m));
}

void CurvesAudioProcessorEditor::sliderValueChanged(juce::Slider* s) {
    // The randomisers are not in knobs_ (they are positioned by hand rather
    // than by the grid), so they need their own route to their parameters.
    if (s == &rnd_wind_ || s == &rnd_flutter_) {
        write_voice_param((s == &rnd_wind_) ? "wind_rnd" : "flutter_rnd",
                          (float)s->getValue());
        return;
    }
    for (auto& k : knobs_) {
        if (&k->slider != s || k->param == nullptr)
            continue;
        if (k->per_voice) {
            write_voice_param(k->base_id.toRawUTF8(), (float)s->getValue());
        } else {
            k->param->beginChangeGesture();
            k->param->setValueNotifyingHost(
                k->param->convertTo0to1((float)s->getValue()));
            k->param->endChangeGesture();
        }
        if (k->base_id == "physics")
            apply_physics_preset((int)s->getValue());
        break;
    }
}

// ---- painting -------------------------------------------------------------

void CurvesAudioProcessorEditor::draw_panel(juce::Graphics& g,
                                            juce::Rectangle<int> box,
                                            const juce::String& title,
                                            bool active) const {
    const auto& t = theme();
    const auto r = box.toFloat();
    g.setColour(juce::Colour(t.panel).withMultipliedAlpha(active ? 1.0f : 0.55f));
    g.fillRoundedRectangle(r, 8.0f);
    g.setColour(juce::Colour(t.panel_edge).withMultipliedAlpha(active ? 1.0f : 0.5f));
    g.drawRoundedRectangle(r.reduced(0.5f), 8.0f, 1.0f);
    // accent bar so each group reads as its own region at a glance
    g.setColour(juce::Colour(t.glow).withAlpha(active ? 0.55f : 0.18f));
    g.fillRoundedRectangle(r.getX() + 10.0f, r.getY() + 9.0f, 26.0f, 2.5f, 1.25f);
    // NOTE: use Graphics::setFont(float). Small text set via
    // Font(FontOptions(h)) did not render at all in this JUCE build, which
    // cost a debug round-trip - the panel titles, knob labels and hint line
    // were all invisible while the geometry was correct the whole time.
    g.setColour(juce::Colour(t.text).withMultipliedAlpha(active ? 0.92f : 0.5f));
    g.setFont(12.0f);
    g.drawText(title, box.getX() + 44, box.getY() + 5, box.getWidth() - 54, 15,
               juce::Justification::centredLeft);
}

void CurvesAudioProcessorEditor::draw_led(juce::Graphics& g,
                                          juce::Rectangle<int> box,
                                          juce::Colour colour,
                                          bool lit) const {
    const auto r = box.toFloat();
    const auto c = r.getCentre();
    const float rad = r.getWidth() * 0.5f;

    // bloom behind the bead, only when lit
    if (lit) {
        g.setColour(colour.withAlpha(0.14f));
        g.fillEllipse(r.expanded(rad * 1.5f));
        g.setColour(colour.withAlpha(0.30f));
        g.fillEllipse(r.expanded(rad * 0.75f));
    }
    // glass bead: lit colour, or near-black when off
    const juce::Colour body = lit ? colour : juce::Colour(0xff0d0f12);
    g.setGradientFill(juce::ColourGradient(
        body.brighter(lit ? 0.45f : 0.10f), c.x - rad * 0.35f, c.y - rad * 0.45f,
        body.darker(lit ? 0.35f : 0.20f), c.x + rad * 0.4f, c.y + rad * 0.5f, false));
    g.fillEllipse(r);
    // rim
    g.setColour(juce::Colours::black.withAlpha(0.55f));
    g.drawEllipse(r.reduced(0.4f), 0.9f);
    // specular highlight — what reads as "glass"
    g.setColour(juce::Colours::white.withAlpha(lit ? 0.70f : 0.18f));
    g.fillEllipse(c.x - rad * 0.42f, c.y - rad * 0.58f, rad * 0.62f, rad * 0.46f);
}

// Draws ONE string's canvas. Everything per-voice is read through `voice`;
// `active` is the highlighted one, which gets the walls, the anchor pin and
// full brightness. The other is a read-only view -- the knobs edit one voice
// at a time, so letting both look editable would be a lie.
void CurvesAudioProcessorEditor::draw_plot(juce::Graphics& g, int voice,
                                           juce::Rectangle<float> b,
                                           bool active) {
    const auto& t = theme();
    const int mode = processor_.physics_mode();
    // Local geometry against THIS canvas rather than the edited one.
    auto nx = [&](float node) {
        return b.getX() + (node / float(active_nodes() - 1)) * b.getWidth();
    };
    auto vy = [&](float v) {
        return b.getCentreY() - (v / display_scale_) * b.getHeight() * 0.5f;
    };
    juce::Graphics::ScopedSaveState keep(g);
    if (!active)
        g.setOpacity(0.5f);
    // ---- plot ground ----
    g.setGradientFill(juce::ColourGradient(juce::Colour(t.plot_top), b.getCentreX(),
                                           b.getY(), juce::Colour(t.plot_bot),
                                           b.getCentreX(), b.getBottom(), false));
    g.fillRoundedRectangle(b, 8.0f);
    g.setColour(juce::Colour(t.panel_edge));
    g.drawRoundedRectangle(b.reduced(0.5f), 8.0f, 1.0f);

    // ---- dB axis -------------------------------------------------------
    // Gridlines sit at ROUND dB values, not round fractions of the canvas,
    // so the marks read +6/+12/+18 rather than 0.25/0.5/0.75. They are drawn
    // from the EFFECTIVE Range (knob after its magnitude modifier) times the
    // zoom, so the axis stays truthful whatever magnitude is selected: at
    // High on an 18 dB knob the canvas really is +-36 dB and says so.
    // Axis follows whichever subscriber the curve is showing.
    const int ax_src = processor_.display_src(voice);
    const double range_db = processor_.display_depth_db(voice);
    const double full_db = range_db * double(display_scale_);
    // smallest step giving at most 4 lines per side
    static constexpr double kSteps[] = {1, 2, 3, 6, 12, 24, 25, 48, 50, 100};
    double step = kSteps[0];
    for (double cand : kSteps) {
        step = cand;
        if (full_db / cand <= 4.0 + 1e-9)
            break;
    }
    g.setFont(9.5f);
    for (double d = step; d <= full_db + 1e-9; d += step) {
        const float v = float(d / std::max(1.0, range_db));
        for (int sign = -1; sign <= 1; sign += 2) {
            const float y = vy(sign * v);
            g.setColour(juce::Colour(t.grid));
            g.drawLine(b.getX() + 26.0f, y, b.getRight(), y, 0.6f);
            g.setColour(juce::Colour(t.text_dim));
            const juce::String lbl =
                (sign > 0 ? "+" : "-")
                + (d < 10.0 && d != std::floor(d) ? juce::String(d, 1)
                                                  : juce::String(int(d)));
            g.drawText(lbl, int(b.getX()) + 2, int(y) - 6, 22, 12,
                       juce::Justification::centredRight);
        }
    }
    // log-frequency grid, labelled
    g.setFont(10.0f);
    const struct { int hz; const char* name; } kMarks[] = {
        {50, "50"}, {100, "100"}, {500, "500"},
        {1000, "1k"}, {5000, "5k"}, {10000, "10k"}};
    for (const auto& m : kMarks) {
        const double frac = std::log(double(m.hz) / 30.0) / std::log(18000.0 / 30.0);
        const float x = float(b.getX() + frac * b.getWidth());
        g.setColour(juce::Colour(t.grid));
        g.drawLine(x, b.getY() + 2.0f, x, b.getBottom() - 12.0f, 0.6f);
        g.setColour(juce::Colour(t.text_dim));
        g.drawText(m.name, int(x) - 16, int(b.getBottom()) - 13, 32, 11,
                   juce::Justification::centred);
    }
    // zero line, brighter
    g.setColour(juce::Colour(t.grid_key));
    g.drawLine(b.getX() + 26.0f, b.getCentreY(), b.getRight(), b.getCentreY(), 1.0f);
    g.setColour(juce::Colour(t.text_dim));
    g.setFont(9.5f);
    g.drawText("0", int(b.getX()) + 2, int(b.getCentreY()) - 6, 22, 12,
               juce::Justification::centredRight);
    // full-scale readout, so the canvas states its own magnitude
    g.setFont(10.0f);
    g.setColour(juce::Colour(t.text));
    static const char* kAxUnit[4] = {"+/-%.3g dB", "+/-%.3g dB width",
                                     "+/-%.3g%% place", "no subscriber on"};
    g.drawText(juce::String::formatted(kAxUnit[juce::jlimit(0, 3, ax_src)],
                                       full_db),
               int(b.getRight()) - 146, int(b.getY()) + 4, 140, 13,
               juce::Justification::centredRight);

    // ---- walls: shade the dead zones, then draw the posts ----
    // Only on the canvas being EDITED: the posts are a control, and two
    // sets of draggable posts where only one responds is worse than one.
    if (active) {
        const float xl = wall_x(0), xh = wall_x(1);
        g.setColour(juce::Colour(t.bg).withAlpha(0.72f));
        if (xl > b.getX() + 0.5f)
            g.fillRect(b.getX(), b.getY(), xl - b.getX(), b.getHeight());
        if (xh < b.getRight() - 0.5f)
            g.fillRect(xh, b.getY(), b.getRight() - xh, b.getHeight());
        for (int w = 0; w < 2; ++w) {
            const float x = (w == 0) ? xl : xh;
            const bool at_edge = (w == 0) ? (xl <= b.getX() + 0.5f)
                                          : (xh >= b.getRight() - 0.5f);
            const float a = at_edge ? 0.60f : 1.0f;   // dim when parked
            g.setColour(juce::Colour(t.anchor).withAlpha(0.18f * a));
            g.fillRect(x - 3.0f, b.getY(), 6.0f, b.getHeight());
            g.setColour(juce::Colour(t.anchor).withAlpha(a));
            g.fillRect(x - 1.0f, b.getY(), 2.0f, b.getHeight());
            // grip handles, top and bottom
            g.fillRoundedRectangle(x - 4.0f, b.getY() + 1.0f, 8.0f, 13.0f, 2.5f);
            g.fillRoundedRectangle(x - 4.0f, b.getBottom() - 14.0f, 8.0f, 13.0f, 2.5f);
        }
    }

    // anchor pin: in Flagpole only the pole (high end) is anchored; the low
    // end is the free cloth termination and whips.
    if (mode == 1 && active) {
        const float px = b.getRight() - 4.0f, py = b.getCentreY();
        g.setColour(juce::Colour(t.anchor).withAlpha(0.25f));
        g.fillEllipse(px - 9.0f, py - 9.0f, 18.0f, 18.0f);
        g.setColour(juce::Colour(t.anchor));
        g.fillEllipse(px - 4.0f, py - 4.0f, 8.0f, 8.0f);
    }

    // ---- faint reference: the raw sim string ----
    if (!(drawing_ && active)) {
        juce::Path sim_curve;
        for (int i = 0; i < active_nodes(); ++i) {
            const float x = nx(float(i));
            const float y = vy(display_nodes_[voice][i]);
            if (i == 0) sim_curve.startNewSubPath(x, y);
            else        sim_curve.lineTo(x, y);
        }
        g.setColour(juce::Colour(t.sim_ref).withAlpha(0.55f));
        g.strokePath(sim_curve, juce::PathStrokeType(1.2f));
    }

    // ---- THE EQ CURVE ----
    // HARD ANCHOR: this comes from the filter bank's smoothed gains, so the
    // shape on screen is exactly the EQ being applied. While drawing, the
    // deposit preview is shown instead, in the accent colour so it is never
    // mistaken for the applied curve.
    const auto& nodes = (drawing_ && active) ? draw_nodes_
                                            : display_eq_[voice];
    juce::Path curve;
    for (int i = 0; i < active_nodes(); ++i) {
        const float x = nx(float(i));
        const float y = vy(nodes[i]);
        if (i == 0) curve.startNewSubPath(x, y);
        else        curve.lineTo(x, y);
    }

    // One clean bright line, no glow and no area fill (owner's call): the
    // plot now reads as exactly two things -- bright = the EQ being applied,
    // faint = where the string actually is.
    const juce::Colour core(drawing_ ? t.draw : t.curve);
    g.setColour(core);
    g.strokePath(curve, juce::PathStrokeType(1.8f));
    // The highlight: a ring in the curve colour round the live canvas, and
    // an L/R tag on each. With two near-identical plots this is the only
    // thing saying which one the knobs are driving.
    if (processor_.dual_on()) {
        g.setColour(juce::Colour(active ? t.curve : t.text_dim)
                        .withAlpha(active ? 0.85f : 0.45f));
        g.drawRoundedRectangle(b.reduced(1.0f), 8.0f, active ? 1.8f : 1.0f);
        g.setFont(11.0f);
        g.drawText(voice == 0 ? "L" : "R", int(b.getX()) + 8,
                   int(b.getY()) + 4, 16, 13, juce::Justification::centredLeft);
    }
}

void CurvesAudioProcessorEditor::paint(juce::Graphics& g) {
    const auto& t = theme();
    g.fillAll(juce::Colour(t.bg));
    const auto b = plot_bounds();
    const int mode = processor_.physics_mode();

    // ---- header ----
    g.setColour(juce::Colour(t.curve));
    g.setFont(22.0f);
    g.drawText("CURVES", 16, 10, 220, 26, juce::Justification::centredLeft);
    g.setColour(juce::Colour(t.text_dim));
    g.setFont(12.0f);
    const char* mode_name[] = {"Free", "Flagpole", "Gust", "Elastic"};
    g.drawText(juce::String("physics: ") + mode_name[std::clamp(mode, 0, 3)]
                   + "   |   " + juce::String(active_nodes()) + " nodes",
               116, 10, 420, 26, juce::Justification::centredLeft);

    // One canvas per running voice. With Dual off this is the single plot
    // exactly as before; with Dual on the second string gets its own.
    const int nvis = processor_.dual_on() ? CurvesAudioProcessor::kNumVoices : 1;
    for (int vi = 0; vi < nvis; ++vi)
        draw_plot(g, vi, plot_bounds(vi), vi == processor_.ui_voice());

    // ---- hint line ----
    // Under the LAST canvas, not the active one: with Dual on and L
    // selected it was landing across the middle of the R plot.
    const auto hb = plot_bounds(nvis - 1);
    g.setColour(juce::Colour(t.text_dim));
    g.setFont(12.0f);
    g.drawText("bright = the EQ being applied   |   faint = the string   |   "
               "drag to draw   |   right-click to flatten   |   drag the posts to "
               "wall off   |   small dial: drag = random amount, click = new seed",
               int(hb.getX()), int(hb.getBottom()) + 4, int(hb.getWidth()), 18,
               juce::Justification::centred);

    // ---- panels ----
    // With two voices live, every per-voice panel says WHICH one it is
    // showing. Without that, two identical-looking panels differing only by
    // a highlighted L/R button is an easy way to edit the wrong channel.
    const juce::String vtag =
        processor_.dual_on()
            ? juce::String(processor_.ui_voice() == 0 ? "   [L]" : "   [R]")
                  + (processor_.link_on() ? juce::String(" LINKED")
                                          : juce::String())
            : juce::String();
    draw_panel(g, panel_[kString], "STRING" + vtag, true);
    draw_panel(g, panel_[kCloth], "CLOTH & WIND   (Flagpole)" + vtag, mode == 1);
    draw_panel(g, panel_[kGlobal], "GLOBAL", true);
    // Name the Q mode in the title the way CLOTH & WIND names the physics
    // mode: it changes every per-band depth, so it must be readable at a
    // glance rather than inferred from a 0/1 knob.
    draw_panel(g, panel_[kSubscriber],
               (processor_.q_matched() ? juce::String("SUBSCRIBERS   (Matched Q)")
                                      : juce::String("SUBSCRIBERS   (Flat Q)"))
                   + vtag,
               true);

    // Per-knob modifier LEDs. Power: yellow lit / black off. Magnitude:
    // green / orange / red for low / medium / high. Both are buttons.
    static const juce::Colour kMagCol[3] = {
        juce::Colour(0xff35e06a), juce::Colour(0xffff9a1f), juce::Colour(0xffff3b30)};
    for (auto& k : knobs_) {
        if (k->mod < 0)
            continue;   // modes and switches carry no modifiers
        const bool live = k->slider.isEnabled();
        const bool on = processor_.knob_on(k->mod);
        const int mag = processor_.knob_mag(k->mod);
        juce::Graphics::ScopedSaveState ss(g);
        if (!live)
            g.setOpacity(0.35f);
        const bool has_power =
            CurvesAudioProcessor::power_mode(k->mod) != CurvesAudioProcessor::kPowerNone;
        // Magnitude stays lit when there is no power switch to dim it.
        if (CurvesAudioProcessor::has_magnitude(k->mod))
            draw_led(g, k->magnitude, kMagCol[juce::jlimit(0, 2, mag)],
                     has_power ? on : true);
        if (has_power)
            draw_led(g, k->power, juce::Colour(0xffffd21f), on);
    }
}
