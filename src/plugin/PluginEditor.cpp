#include "PluginEditor.h"

#include <algorithm>
#include <cmath>

CurvesAudioProcessorEditor::CurvesAudioProcessorEditor(CurvesAudioProcessor& p)
    : AudioProcessorEditor(p), processor_(p) {
    setSize(760, 560);
    active_count_ = processor_.active_nodes();
    startTimerHz(30);

    // Control strip: one rotary per parameter, bound to the raw params.
    const struct { const char* id; const char* name; } defs[] = {
        {"damping", "Damping"},    {"stiffness", "Stiffness"},
        {"tension", "Tension"},    {"smoothing", "Smoothing"},
        {"drive", "Range Â±dB"},    {"steady", "Steady"},
        {"leveler", "Leveler"},    {"physics", "Physics"},
        {"precision", "Precision"}, {"velocity", "Velocity"},
        {"wind", "Wind"},          {"waves", "Waves"},
        {"force", "Force"},
        {"direction", "Direction"}, {"length", "Length"},
        {"test", "Test noise"},
    };
    for (const auto& d : defs) {
        for (auto* p2 : processor_.getParameters()) {
            auto* rp = dynamic_cast<juce::RangedAudioParameter*>(p2);
            if (rp == nullptr || rp->paramID != juce::String(d.id))
                continue;
            auto knob = std::make_unique<Knob>();
            const auto& range = rp->getNormalisableRange();
            // Range object: parameter ranges with interval 0 must stay
            // continuous; discrete params (Physics/Precision) carry interval 1.
            if (range.interval > 0.0f)
                knob->slider.setRange(range.start, range.end, range.interval);
            else
                knob->slider.setNormalisableRange(
                    juce::NormalisableRange<double>(range.start, range.end));
            knob->slider.setValue(rp->convertFrom0to1(rp->getValue()),
                                  juce::dontSendNotification);
            knob->slider.setSliderStyle(juce::Slider::RotaryVerticalDrag);
            knob->slider.setTextBoxStyle(juce::Slider::NoTextBox, true, 0, 0);
            knob->slider.setColour(juce::Slider::rotarySliderFillColourId,
                                   juce::Colour(0xff54d6ff));
            knob->slider.setColour(juce::Slider::thumbColourId,
                                   juce::Colour(0xffe8e6e0));
            knob->slider.setColour(juce::Slider::backgroundColourId,
                                   juce::Colour(0xff2a2e35));
            knob->slider.addListener(this);
            knob->label.setText(d.name, juce::dontSendNotification);
            knob->label.setJustificationType(juce::Justification::centred);
            knob->label.setFont(juce::Font(juce::FontOptions(12.0f)));
            knob->label.setColour(juce::Label::textColourId, juce::Colour(0xffa8b0b8));
            knob->param = rp;
            addAndMakeVisible(knob->slider);
            addAndMakeVisible(knob->label);
            knobs_.push_back(std::move(knob));
            break;
        }
    }
    scale_button_.setColour(juce::TextButton::buttonColourId, juce::Colour(0xff2a2e35));
    scale_button_.setColour(juce::TextButton::textColourOffId, juce::Colour(0xff54d6ff));
    addAndMakeVisible(scale_button_);
    scale_button_.onClick = [this] { cycle_display_scale(); };

    resized();  // lay out the knobs now — setSize above ran before they existed
}

void CurvesAudioProcessorEditor::cycle_display_scale() {
    static constexpr float kScales[4] = {0.5f, 1.0f, 2.0f, 4.0f};
    static const char* kLabels[4] = {"+-0.5", "+-1", "+-2", "+-4"};
    const float* it = std::find(std::begin(kScales), std::end(kScales), display_scale_);
    const int idx = (it == std::end(kScales) - 1) ? 0 : int(it - std::begin(kScales)) + 1;
    display_scale_ = kScales[idx];
    scale_button_.setButtonText(kLabels[idx]);
}

CurvesAudioProcessorEditor::~CurvesAudioProcessorEditor() { stopTimer(); }

void CurvesAudioProcessorEditor::resized() {
    scale_button_.setBounds(getWidth() - 84, 36, 76, 22);
    if (knobs_.empty())
        return;  // setSize() runs before the knobs are created in the ctor
    auto strip = getLocalBounds().removeFromBottom(110).reduced(8, 6);
    const int cell_w = strip.getWidth() / (int)knobs_.size();
    for (auto& k : knobs_) {
        auto cell = strip.removeFromLeft(cell_w);
        const int side = std::min(58, cell.getWidth() - 4);
        k->slider.setBounds(cell.getX() + (cell.getWidth() - side) / 2,
                            cell.getY() + 2, side, side);
        k->label.setBounds(cell.getX(), cell.getY() + side + 4, cell.getWidth(), 18);
    }
}

// ---- geometry ------------------------------------------------------------

juce::Rectangle<float> CurvesAudioProcessorEditor::plot_bounds() const {
    return getLocalBounds().toFloat().reduced(30.0f).withTrimmedBottom(144.0f);
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
    // Release: stop calling displace(); the string rings from the held shape.
    drawing_ = false;
}

void CurvesAudioProcessorEditor::deposit_stroke(float n0, float v0, float n1, float v1) {
    // Linear interpolation between the previous and current drag points,
    // deposited onto nodes in the swept range. (Line-law shaping beyond
    // linear is an M3 generator feature, not a canvas feature.)
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
        // While not drawing, the sim owns the curve; keep the local draw
        // buffer in sync so the next stroke deposits onto the live shape.
        draw_nodes_ = display_nodes_;
        dirty_ = false;
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

void CurvesAudioProcessorEditor::paint(juce::Graphics& g) {
    g.fillAll(juce::Colour(0xff14161a));
    const auto b = plot_bounds();

    // grid: zero line + log-frequency reference lines
    g.setColour(juce::Colour(0xff2a2e35));
    g.drawLine(b.getX(), b.getCentreY(), b.getRight(), b.getCentreY(), 1.0f);
    for (int f : {100, 1000, 10000}) {
        const double t = std::log(double(f) / 30.0) / std::log(18000.0 / 30.0);
        const float x = float(b.getX() + t * b.getWidth());
        g.drawLine(x, b.getY(), x, b.getBottom(), 0.5f);
    }

    // anchor pin: only the pole (high end) is anchored in Flagpole mode —
    // the left end is the free termination of the cloth and whips freely
    const int mode = processor_.physics_mode();
    if (mode == 1 /* Flagpole */) {
        g.setColour(juce::Colour(0xffffb347));
        g.fillEllipse(b.getRight() - 5.0f, b.getCentreY() - 5.0f, 10.0f, 10.0f);
    }

    // reference: the live string the wind/physics move (faint)
    if (!drawing_) {
        juce::Path sim_curve;
        for (int i = 0; i < active_nodes(); ++i) {
            const float x = node_to_x(float(i));
            const float y = value_to_y(display_nodes_[i]);
            if (i == 0)
                sim_curve.startNewSubPath(x, y);
            else
                sim_curve.lineTo(x, y);
        }
        g.setColour(juce::Colour(0x4054d6ff));
        g.strokePath(sim_curve, juce::PathStrokeType(1.4f));
    }

    // the main curve = the ACTUAL EQ (bank's smoothed + Steady-modulated
    // gains): what you see is what the filter bank is doing. While drawing,
    // show the deposit preview instead.
    const auto& nodes = drawing_ ? draw_nodes_ : display_eq_;
    juce::Path curve;
    for (int i = 0; i < active_nodes(); ++i) {
        const float x = node_to_x(float(i));
        const float y = value_to_y(nodes[i]);
        if (i == 0)
            curve.startNewSubPath(x, y);
        else
            curve.lineTo(x, y);
    }

    if (drawing_) {
        g.setColour(juce::Colour(0x60ffb347));
        g.strokePath(curve, juce::PathStrokeType(5.0f));
        g.setColour(juce::Colour(0xffffb347));
        g.strokePath(curve, juce::PathStrokeType(2.0f));
    } else {
        g.setColour(juce::Colour(0xff54d6ff));
        g.strokePath(curve, juce::PathStrokeType(2.2f));
    }

    g.setColour(juce::Colour(0xff707880));
    g.setFont(13.0f);
    const char* mode_name[] = {"Free", "Flagpole", "Gust", "Elastic"};
    g.drawText(juce::String("bright curve = actual EQ  |  faint curve = the string  |  draw: hold + drag  |  right-click: flatten  |  physics: ")
                   + mode_name[std::clamp(mode, 0, 3)],
               getLocalBounds().toFloat().reduced(30.0f)
                   .withTrimmedTop(plot_bounds().getBottom())
                   .withTrimmedBottom(116.0f),
               juce::Justification::centred);
}
