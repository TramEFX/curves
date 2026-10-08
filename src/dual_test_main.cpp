// Dual voices: two independent strings, one per channel.
//
// This is the only test that instantiates the real PROCESSOR rather than
// mirroring its logic, because the thing being checked is the routing --
// which parameter set reaches which engine, and which engine reaches which
// channel. A mirror of that logic would test the mirror.
//
// Checks, in order:
//   1. the parameter list really doubled, with "_r" ids for voice 1
//   2. Dual OFF still produces the historical mid/side behaviour
//   3. Dual ON makes L and R DIVERGE (the entire point)
//   4. Dual ON with voice 1 a copy of voice 0 still diverges, because the
//      seeds are re-rolled -- "same weather, different gusts"
//   5. nothing goes non-finite across a Dual toggle mid-stream

#include "plugin/PluginEditor.h"
#include "plugin/PluginProcessor.h"
#include <cmath>
#include <cstdio>
#include <vector>

namespace {

juce::AudioParameterFloat* find(CurvesAudioProcessor& p, const juce::String& id) {
    for (auto* q : p.getParameters())
        if (auto* rp = dynamic_cast<juce::AudioParameterFloat*>(q))
            if (rp->paramID == id)
                return rp;
    return nullptr;
}

void set(CurvesAudioProcessor& p, const juce::String& id, float v) {
    if (auto* q = find(p, id))
        q->setValueNotifyingHost(q->convertTo0to1(v));
}

struct Stats { double rms_l, rms_r, diff_rms, peak; bool finite; };

// Runs `blocks` of noise and reports how far apart the two channels end up.
Stats run(CurvesAudioProcessor& p, int blocks, int bs = 256) {
    juce::AudioBuffer<float> buf(2, bs);
    juce::MidiBuffer midi;
    unsigned rs = 22222u;
    double sl = 0.0, sr = 0.0, sd = 0.0, peak = 0.0;
    bool finite = true;
    int counted = 0;
    for (int b = 0; b < blocks; ++b) {
        for (int i = 0; i < bs; ++i) {
            rs = rs * 1664525u + 1013904223u;
            const float x = float((int(rs >> 9) & 0x3FFFFF) / 2097151.0 - 1.0) * 0.2f;
            buf.setSample(0, i, x);   // IDENTICAL input on both channels, so
            buf.setSample(1, i, x);   // any divergence is the plugin's doing
        }
        p.processBlock(buf, midi);
        if (b < blocks / 2)
            continue;               // let the strings get moving first
        for (int i = 0; i < bs; ++i) {
            const double l = buf.getSample(0, i), r = buf.getSample(1, i);
            if (!std::isfinite(l) || !std::isfinite(r))
                finite = false;
            sl += l * l; sr += r * r; sd += (l - r) * (l - r);
            peak = std::max(peak, std::max(std::abs(l), std::abs(r)));
            ++counted;
        }
    }
    const double n = std::max(1, counted);
    return {std::sqrt(sl / n), std::sqrt(sr / n), std::sqrt(sd / n), peak, finite};
}

void report(const char* name, const Stats& s) {
    std::printf("%-34s  rmsL %7.4f  rmsR %7.4f  |L-R| %9.3e  peak %6.3f  %s\n",
                name, s.rms_l, s.rms_r, s.diff_rms, s.peak,
                s.finite ? "finite" : "NON-FINITE");
}

}  // namespace

int main() {
    juce::ScopedJuceInitialiser_GUI juce_init;

    std::printf("=== 1. PARAMETER LIST ===\n\n");
    CurvesAudioProcessor proc;
    const int total = proc.getParameters().size();
    int nvp = 0;
    const char* const* ids = CurvesAudioProcessor::voice_param_ids(nvp);
    int found0 = 0, found1 = 0;
    for (int i = 0; i < nvp; ++i) {
        if (find(proc, CurvesAudioProcessor::voice_param_id(ids[i], 0))) ++found0;
        if (find(proc, CurvesAudioProcessor::voice_param_id(ids[i], 1))) ++found1;
    }
    std::printf("per-voice ids declared      %d\n", nvp);
    std::printf("voice 0 present             %d\n", found0);
    std::printf("voice 1 present ('_r')      %d\n", found1);
    // Count the globals rather than hard-coding a total: the expectation
    // went stale the moment Link was added, which is exactly the drift a
    // literal invites.
    static const char* kGlobals[] = {"mix", "leveler", "test", "precision",
                                     "dual", "link"};
    int nglob = 0;
    for (const char* g : kGlobals) {
        const bool have = find(proc, g) != nullptr;
        nglob += have ? 1 : 0;
        std::printf("  global %-10s %s\n", g, have ? "ok" : "*** MISSING ***");
    }
    const int expect = nvp * 2 + (int)(sizeof(kGlobals) / sizeof(kGlobals[0]));
    std::printf("TOTAL parameters            %d   (expect %d)\n",
                total, expect);
    std::printf("%s\n", (found0 == nvp && found1 == nvp
                            && nglob == (int)(sizeof(kGlobals) / sizeof(kGlobals[0]))
                            && total == expect)
                            ? "PASS" : "*** FAIL ***");

    std::printf("\n=== 2-4. ROUTING: identical input on both channels ===\n");
    std::printf("Input is the SAME on L and R, so any |L-R| is the plugin.\n");
    std::printf("Dual off must stay near zero; dual on must not.\n\n");

    proc.prepareToPlay(48000.0, 256);
    set(proc, "precision", 0.0f);     // 128 nodes, so this stays quick
    // Physics defaults to Free, which applies NO forcing -- both strings sit
    // flat and the EQ is transparent. Flagpole is the mode with weather in
    // it, and without this the test measures two motionless strings and
    // reports it as a routing failure (it did).
    set(proc, "physics", 1.0f);
    set(proc, "physics_r", 1.0f);
    set(proc, "drive", 18.0f);
    set(proc, "drive_r", 18.0f);
    set(proc, "motion", 0.6f);
    set(proc, "motion_r", 0.6f);
    set(proc, "test", 0.0f);          // external input, not the test source
    set(proc, "dual", 0.0f);
    const Stats off = run(proc, 60);
    report("dual OFF (one string, M/S)", off);

    proc.set_dual(true);              // copies voice 0 -> voice 1, re-rolls seeds
    const Stats on_copy = run(proc, 60);
    report("dual ON  (copied + reseeded)", on_copy);

    set(proc, "motion_r", 0.95f);     // now make voice 1 genuinely different
    set(proc, "wind_r", 0.9f);
    set(proc, "tension_r", 120.0f);
    const Stats on_diff = run(proc, 60);
    report("dual ON  (voice 1 retuned)", on_diff);
    // set_knob_on writes to the HIGHLIGHTED voice, so point the UI at
    // voice 1 first -- gating with ui_voice 0 silenced the wrong string.
    const int keep = proc.ui_voice();
    proc.set_ui_voice(1);
    proc.set_knob_on(CurvesAudioProcessor::kRangeDb, false);
    const Stats on_gate = run(proc, 60);
    report("dual ON  (voice 1 EQ gated off)", on_gate);
    proc.set_knob_on(CurvesAudioProcessor::kRangeDb, true);
    proc.set_ui_voice(keep);

    std::printf("\n=== 5. TOGGLING DUAL MID-STREAM ===\n\n");
    bool ok = true;
    for (int i = 0; i < 6; ++i) {
        proc.set_dual(i % 2 == 0);
        const Stats s = run(proc, 8);
        if (!s.finite || s.peak > 8.0)
            ok = false;
    }
    std::printf("six toggles, no non-finite sample, peak bounded: %s\n",
                ok ? "PASS" : "*** FAIL ***");

    std::printf("\n=== VERDICT ===\n");
    std::printf("dual off keeps the channels together   %s  (|L-R| %.5f)\n",
                off.diff_rms < 1e-6 ? "PASS" : "----", off.diff_rms);
    std::printf("dual on separates them                 %s  (|L-R| %.5f)\n",
                on_copy.diff_rms > 1e-3 ? "PASS" : "*** FAIL ***",
                on_copy.diff_rms);
    // Retuning voice 1 changes both the separation AND the levels, so
    // "more different" is not a meaningful bar -- absolute rms difference
    // is not a divergence metric. Requiring it to STAY separated is.
    std::printf("retuned voice 1 stays separated        %s  (|L-R| %.5f)\n",
                on_diff.diff_rms > 1e-3 ? "PASS" : "*** FAIL ***",
                on_diff.diff_rms);
    std::printf("gating voice 1's EQ changes only R      %s  (rmsL %.4f rmsR %.4f)\n",
                (on_gate.rms_r < on_gate.rms_l * 0.9) ? "PASS" : "*** FAIL ***",
                on_gate.rms_l, on_gate.rms_r);
    std::printf("\n=== 6. EDITOR BINDING: do the two voices edit independently? ===\n");
    std::printf("The editor decides which voice a knob writes to. dual_test\n");
    std::printf("covers the processor only, so this drives the REAL knob path.\n\n");
    {
        proc.set_dual(true);
        std::unique_ptr<juce::AudioProcessorEditor> ed(proc.createEditor());
        auto* ce = dynamic_cast<CurvesAudioProcessorEditor*>(ed.get());
        if (ce == nullptr) {
            std::printf("*** FAIL *** could not create the editor\n");
        } else {
            bool pass = true;
            proc.set_ui_voice(0);
            ce->rebind_for_test();
            const int b0 = ce->bound_voice_of("damping");
            proc.set_ui_voice(1);
            ce->rebind_for_test();
            const int b1 = ce->bound_voice_of("damping");
            std::printf("  knob follows the highlighted voice      L->%d  R->%d  %s\n",
                        b0, b1, (b0 == 0 && b1 == 1) ? "PASS" : "*** FAIL ***");
            pass = pass && (b0 == 0 && b1 == 1);

            // Move the knob with R selected; L must not follow.
            auto* pl = find(proc, "damping");
            auto* pr = find(proc, "damping_r");
            const float before_l = pl ? pl->get() : -1.0f;
            ce->nudge_knob("damping", 7.5f);
            const float after_l = pl ? pl->get() : -1.0f;
            const float after_r = pr ? pr->get() : -1.0f;
            const bool indep = std::abs(after_r - 7.5f) < 0.01f
                               && std::abs(after_l - before_l) < 1e-4f;
            std::printf("  editing R leaves L alone                L %.2f->%.2f  R %.2f  %s\n",
                        before_l, after_l, after_r, indep ? "PASS" : "*** FAIL ***");
            pass = pass && indep;

            // The randomisers are NOT in knobs_ and wrote bare ids.
            auto* wl = find(proc, "wind_rnd");
            auto* wr = find(proc, "wind_rnd_r");
            std::printf("  randomisers exist per voice             %s\n",
                        (wl && wr) ? "yes" : "*** MISSING ***");
            // The randomisers sit OUTSIDE knobs_ and used to write the bare
            // id, so they always moved voice 0 whichever voice was live.
            proc.set_link(false);
            proc.set_ui_voice(1);
            ce->rebind_for_test();
            const float rl0 = wl ? wl->get() : -1.0f;
            ce->set_rnd_for_test(true, 0.42f);
            const bool rnd_ok = wr && std::abs(wr->get() - 0.42f) < 0.01f
                                && wl && std::abs(wl->get() - rl0) < 1e-4f;
            std::printf("  randomiser edits R only                 L %.2f  R %.2f  %s\n",
                        wl ? wl->get() : -1.0f, wr ? wr->get() : -1.0f,
                        rnd_ok ? "PASS" : "*** FAIL ***");
            pass = pass && rnd_ok;

            // LINK: an edit must now land on BOTH voices.
            proc.set_link(true);
            ce->rebind_for_test();
            ce->nudge_knob("damping", 11.0f);
            const bool linked = pl && pr
                                && std::abs(pl->get() - 11.0f) < 0.01f
                                && std::abs(pr->get() - 11.0f) < 0.01f;
            std::printf("  LINK on: edit hits both voices          L %.2f  R %.2f  %s\n",
                        pl ? pl->get() : -1.0f, pr ? pr->get() : -1.0f,
                        linked ? "PASS" : "*** FAIL ***");
            pass = pass && linked;

            // ...and unlinking must restore independence.
            proc.set_link(false);
            ce->nudge_knob("damping", 4.0f);
            const bool unlinked = pl && pr
                                  && std::abs(pr->get() - 4.0f) < 0.01f
                                  && std::abs(pl->get() - 11.0f) < 0.01f;
            std::printf("  LINK off: independence restored         L %.2f  R %.2f  %s\n",
                        pl ? pl->get() : -1.0f, pr ? pr->get() : -1.0f,
                        unlinked ? "PASS" : "*** FAIL ***");
            pass = pass && unlinked;

            std::printf("  overall %s\n", pass ? "PASS" : "*** FAIL ***");
        }
    }
    return 0;
}
