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
    setSize(1000, 700);
    active_count_ = processor_.active_nodes();
    startTimerHz(30);

    // Controls, grouped by SCOPE. GLOBAL is always in effect; STRING is the
    // physics that always applies; CLOTH & WIND is Flagpole-only.
    const struct { const char* id; const char* name; int group; } defs[] = {
        // GLOBAL
        {"physics", "Physics", kGlobal},    {"precision", "Precision", kGlobal},
        {"drive", "Range dB", kGlobal},     {"smoothing", "Smoothing", kGlobal},
        {"steady", "Steady", kGlobal},      {"leveler", "Leveler", kGlobal},
        {"test", "Test Noise", kGlobal},
        // STRING
        {"damping", "Damping", kString},    {"stiffness", "Stiffness", kString},
        {"tension", "Tension", kString},    {"velocity", "Velocity", kString},
        // CLOTH & WIND
        {"motion", "Motion", kCloth},       {"character", "Character", kCloth},
        {"wind", "Wind", kCloth},           {"flutter_hz", "Flutter Hz", kCloth},
        {"waves", "Waves", kCloth},         {"wrinkle", "Wrinkle", kCloth},
        {"direction", "Direction", kCloth}, {"length", "Length", kCloth},
    };
    for (const auto& d : defs) {
        for (auto* p2 : processor_.getParameters()) {
            auto* rp = dynamic_cast<juce::RangedAudioParameter*>(p2);
            if (rp == nullptr || rp->paramID != juce::String(d.id))
                continue;
            auto knob = std::make_unique<Knob>();
            const auto& range = rp->getNormalisableRange();
            // Discrete params (Physics/Precision/Direction) carry interval 1;
            // continuous ones must stay continuous.
            if (range.interval > 0.0f)
                knob->slider.setRange(range.start, range.end, range.interval);
            else
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
            knobs_.push_back(std::move(knob));
            break;
        }
    }

    for (auto* b : {&scale_button_, &theme_button_, &reset_button_})
        addAndMakeVisible(*b);
    // Flattens the string and leaves every modifier running, so the cloth
    // restarts from rest. Replaces winding Damping up to kill energy.
    reset_button_.onClick = [this] { processor_.request_reset(); };
    scale_button_.onClick = [this] { cycle_display_scale(); };
    theme_button_.onClick = [this] { cycle_theme(); };

    apply_theme();
    resized();  // setSize() above ran before the knobs existed
}

CurvesAudioProcessorEditor::~CurvesAudioProcessorEditor() {
    stopTimer();
    // Sliders must not outlive the LookAndFeel they point at.
    for (auto& k : knobs_)
        k->slider.setLookAndFeel(nullptr);
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
    for (auto* b : {&scale_button_, &theme_button_, &reset_button_}) {
        b->setColour(juce::TextButton::buttonColourId, juce::Colour(t.panel));
        b->setColour(juce::TextButton::textColourOffId, juce::Colour(t.curve));
    }
    for (auto& k : knobs_)
        k->label.setColour(juce::Label::textColourId, juce::Colour(t.text_dim));
    repaint();
}

// ---- layout ---------------------------------------------------------------
// Panel heights live here as named constants because plot_bounds() has to
// subtract exactly the same amounts; keeping them in one place stops the
// canvas and the panels from overlapping when one of them changes.
namespace {
constexpr int kHeaderH = 34;
constexpr int kHintH = 24;
constexpr int kGlobalH = 104;
constexpr int kPhysicsH = 188;
constexpr int kPanelGap = 10;
}  // namespace

void CurvesAudioProcessorEditor::resized() {
    scale_button_.setBounds(getWidth() - 86, 12, 74, 24);
    theme_button_.setBounds(getWidth() - 172, 12, 78, 24);
    reset_button_.setBounds(getWidth() - 256, 12, 76, 24);
    if (knobs_.empty())
        return;  // setSize() runs before the knobs are created in the ctor

    auto area = getLocalBounds().reduced(14, 12);
    area.removeFromTop(kHeaderH);

    // Bottom-up: GLOBAL gets its own full-width row (the "one obvious
    // spot"), with STRING and CLOTH & WIND sharing the row above it.
    panel_[kGlobal] = area.removeFromBottom(kGlobalH);
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
    place(kGlobal, panel_[kGlobal], 7);
    place(kString, panel_[kString], 2);
    place(kCloth, panel_[kCloth], 4);
}

// ---- geometry -------------------------------------------------------------

juce::Rectangle<float> CurvesAudioProcessorEditor::plot_bounds() const {
    auto area = getLocalBounds().reduced(14, 12);
    area.removeFromTop(kHeaderH);
    area.removeFromBottom(kGlobalH + kPanelGap + kPhysicsH + kHintH);
    return area.toFloat();
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
            if (k->magnitude.expanded(4).contains(pt)) {
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
    if (!plot_bounds().contains(e.position.toFloat()))
        return;
    drawing_ = true;
    // Deposit onto the string's *current* state, not a stale buffer.
    std::array<float, CurvesAudioProcessor::kMaxNodes> eq_snap{};
    processor_.pull_display(draw_nodes_, eq_snap, active_count_);
    last_node_ = juce::jlimit(0.0f, float(active_nodes() - 1), x_to_node(e.position.x));
    last_val_ = y_to_value(e.position.y);
    deposit_stroke(last_node_, last_val_, last_node_, last_val_);
    push_current();
}

void CurvesAudioProcessorEditor::mouseDrag(const juce::MouseEvent& e) {
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

void CurvesAudioProcessorEditor::apply_physics_preset(int mode) {
    auto set_param = [this](const char* id, float v) {
        for (auto& k : knobs_) {
            if (k->param == nullptr || k->param->paramID != juce::String(id))
                continue;
            k->param->beginChangeGesture();
            k->param->setValueNotifyingHost(k->param->convertTo0to1(v));
            k->param->endChangeGesture();
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
    processor_.pull_display(display_nodes_, display_eq_, active_count_);
    if (!drawing_) {
        draw_nodes_ = display_nodes_;
        dirty_ = false;
    }
    // Cloth & Wind only does anything in Flagpole, so grey it out elsewhere
    // rather than leaving nineteen live-looking knobs on screen.
    const int mode = processor_.physics_mode();
    if (mode != last_mode_) {
        last_mode_ = mode;
        const bool cloth_live = (mode == 1);
        for (auto& k : knobs_)
            if (k->group == kCloth) {
                k->slider.setEnabled(cloth_live);
                k->label.setAlpha(cloth_live ? 1.0f : 0.4f);
            }
    }
    repaint();
}

void CurvesAudioProcessorEditor::sliderValueChanged(juce::Slider* s) {
    for (auto& k : knobs_) {
        if (&k->slider != s || k->param == nullptr)
            continue;
        k->param->beginChangeGesture();
        k->param->setValueNotifyingHost(
            k->param->convertTo0to1((float)s->getValue()));
        k->param->endChangeGesture();
        if (k->param->paramID == juce::String("physics"))
            apply_physics_preset((int)s->getValue());
        break;
    }
}

// ---- painting -------------------------------------------------------------

void CurvesAudioProcessorEditor::draw_panel(juce::Graphics& g,
                                            juce::Rectangle<int> box,
                                            const char* title,
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

    // ---- plot ground ----
    g.setGradientFill(juce::ColourGradient(juce::Colour(t.plot_top), b.getCentreX(),
                                           b.getY(), juce::Colour(t.plot_bot),
                                           b.getCentreX(), b.getBottom(), false));
    g.fillRoundedRectangle(b, 8.0f);
    g.setColour(juce::Colour(t.panel_edge));
    g.drawRoundedRectangle(b.reduced(0.5f), 8.0f, 1.0f);

    // horizontal divisions at fractions of the current scale
    g.setColour(juce::Colour(t.grid));
    for (float frac : {0.25f, 0.5f, 0.75f}) {
        const float dy = b.getHeight() * 0.5f * frac;
        g.drawLine(b.getX(), b.getCentreY() - dy, b.getRight(), b.getCentreY() - dy, 0.6f);
        g.drawLine(b.getX(), b.getCentreY() + dy, b.getRight(), b.getCentreY() + dy, 0.6f);
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
    g.drawLine(b.getX(), b.getCentreY(), b.getRight(), b.getCentreY(), 1.0f);

    // anchor pin: in Flagpole only the pole (high end) is anchored; the low
    // end is the free cloth termination and whips.
    if (mode == 1) {
        const float px = b.getRight() - 4.0f, py = b.getCentreY();
        g.setColour(juce::Colour(t.anchor).withAlpha(0.25f));
        g.fillEllipse(px - 9.0f, py - 9.0f, 18.0f, 18.0f);
        g.setColour(juce::Colour(t.anchor));
        g.fillEllipse(px - 4.0f, py - 4.0f, 8.0f, 8.0f);
    }

    // ---- faint reference: the raw sim string ----
    if (!drawing_) {
        juce::Path sim_curve;
        for (int i = 0; i < active_nodes(); ++i) {
            const float x = node_to_x(float(i));
            const float y = value_to_y(display_nodes_[i]);
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
    const auto& nodes = drawing_ ? draw_nodes_ : display_eq_;
    juce::Path curve;
    for (int i = 0; i < active_nodes(); ++i) {
        const float x = node_to_x(float(i));
        const float y = value_to_y(nodes[i]);
        if (i == 0) curve.startNewSubPath(x, y);
        else        curve.lineTo(x, y);
    }

    // area fill between the curve and the zero line
    {
        juce::Path area = curve;
        area.lineTo(b.getRight(), b.getCentreY());
        area.lineTo(b.getX(), b.getCentreY());
        area.closeSubPath();
        const juce::Colour fill(drawing_ ? t.draw : t.fill);
        g.setGradientFill(juce::ColourGradient(
            fill.withAlpha(0.26f), b.getCentreX(), b.getY(),
            fill.withAlpha(0.02f), b.getCentreX(), b.getCentreY(), false));
        g.fillPath(area);
    }

    // three-pass glow, then the bright core
    const juce::Colour core(drawing_ ? t.draw : t.curve);
    const juce::Colour glow(drawing_ ? t.draw : t.glow);
    g.setColour(glow.withAlpha(0.10f));
    g.strokePath(curve, juce::PathStrokeType(9.0f));
    g.setColour(glow.withAlpha(0.22f));
    g.strokePath(curve, juce::PathStrokeType(5.0f));
    g.setColour(core.withAlpha(0.85f));
    g.strokePath(curve, juce::PathStrokeType(2.6f));
    g.setColour(juce::Colours::white.withAlpha(drawing_ ? 0.25f : 0.45f));
    g.strokePath(curve, juce::PathStrokeType(1.0f));

    // ---- hint line ----
    g.setColour(juce::Colour(t.text_dim));
    g.setFont(12.0f);
    g.drawText("bright = the EQ being applied   |   faint = the string   |   "
               "drag to draw   |   right-click to flatten",
               int(b.getX()), int(b.getBottom()) + 4, int(b.getWidth()), 18,
               juce::Justification::centred);

    // ---- panels ----
    draw_panel(g, panel_[kString], "STRING", true);
    draw_panel(g, panel_[kCloth], "CLOTH & WIND   (Flagpole)", mode == 1);
    draw_panel(g, panel_[kGlobal], "GLOBAL", true);

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
        draw_led(g, k->magnitude, kMagCol[juce::jlimit(0, 2, mag)],
                 has_power ? on : true);
        if (has_power)
            draw_led(g, k->power, juce::Colour(0xffffd21f), on);
    }
}
