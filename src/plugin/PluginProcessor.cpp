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
    addParameter(force_ = new APF("force", "Force", R{0.0f, 1.0f}, 0.6f));
    addParameter(direction_ = new APF("direction", "Direction", R(0.0f, 1.0f, 1.0f), 0.0f));
    addParameter(length_ = new APF("length", "Cloth length", R{0.2f, 1.0f}, 1.0f));
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
    // Force = WIND SPEED (nodes/s) in the aero model. Drag rises as the
    // string's own velocity catches up to the wind, so amplitude is
    // compressive and monotonic across the whole dial with no ceiling to
    // tune, no dead zone, and no limiter gain that can latch.
    //
    // Calibration (measured, ORCHESTRATION §14): peak |u| reaches ~0.95 at
    // force = 1.0, i.e. just under the ±1 clamp the bank and the display
    // share — so the EQ curve stays round instead of flat-topping, which is
    // what keeps the on-screen shape a faithful picture of the EQ.
    //   wind_speed = 37 * force * (tension/300)^0.6 * (128/n)^0.25
    // The tension term makes the dial mean the same thing at any Tension
    // (spread 4.7x -> 1.4x); the node term holds it across precision modes.
    // No node-count scaling of a FORCE any more: wind_speed is a velocity in
    // the same units as the node velocities it is compared against.
    const double tension_norm = std::pow(std::max(1.0f, tension_->get()) / 300.0, 0.6);
    const double node_norm = std::pow(128.0 / std::max(1, n_nodes_), 0.25);
    sim_->wind_speed = 37.0 * force_->get() * tension_norm * node_norm;
    // Flap limiter retired in the aero path: drag IS the amplitude
    // mechanism. Left off rather than removed so the kFlagpoleWind model and
    // the six original validation scenarios keep their exact behavior.
    sim_->flap_limit = 1e9;
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
        gain_targets_[i] = std::pow(10.0, db / 20.0);
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

    // 5. GUI is a reader: publish positions for display.
    publish_display();
}

void CurvesAudioProcessor::publish_display() {
    const juce::SpinLock::ScopedLockType lk(display_lock_);
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
    if (auto xml = tree.createXml())
        copyXmlToBinary(*xml, dest);
}

void CurvesAudioProcessor::setStateInformation(const void* data, int size) {
    if (auto xml = getXmlFromBinary(data, size))
        if (auto tree = juce::ValueTree::fromXml(*xml); tree.isValid())
            for (auto* p : getParameters())
                if (auto* v = dynamic_cast<juce::RangedAudioParameter*>(p))
                    if (tree.hasProperty(juce::Identifier(v->paramID)))
                        v->setValueNotifyingHost(tree.getProperty(juce::Identifier(v->paramID)));
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
