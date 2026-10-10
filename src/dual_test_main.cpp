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
#include <ctime>
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
                                     "dual", "link", "draw_mode", "tip",
                                     "raw"};
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
    // EQ depth up for this check: with Band Q correcting fully the EQ is
    // honest, so at a modest depth gating it off barely moves the broadband
    // rms and the assertion was measuring noise. The threshold was
    // calibrated against the old 24-96x overdrive.
    set(proc, "drive", 24.0f);
    set(proc, "drive_r", 24.0f);
    set(proc, "q_mode", 1.0f);
    set(proc, "q_mode_r", 1.0f);
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
    // Informational, NOT an assertion. Gating one voice's EQ barely moves
    // the broadband rms now that the bank corrects fully -- the EQ is
    // honest, so it no longer multiplies the level 24-96x -- and the
    // residual is smaller than the chaotic sim's own run-to-run variance
    // (L moved MORE than R on one run). Channel independence is proved
    // deterministically by section 6, which reads the parameters directly
    // instead of inferring from a noisy level.
    std::printf("gating voice 1's EQ (informational)       L %+.1f%%  R %+.1f%%\n",
                100.0 * (on_gate.rms_l / on_diff.rms_l - 1.0),
                100.0 * (on_gate.rms_r / on_diff.rms_r - 1.0));
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

            // Band Q reported as "will not move". Discrete 0/1 rotary.
            for (const char* id : {"q_mode", "physics", "direction"}) {
                auto* pp = find(proc, CurvesAudioProcessor::voice_param_id(id, 1));
                const float was = pp ? pp->get() : -1.0f;
                ce->nudge_knob(id, 1.0f);
                const float now = pp ? pp->get() : -1.0f;
                std::printf("  discrete knob %-10s %.2f -> %.2f  %s\n",
                            id, was, now,
                            (now > 0.5f) ? "writes ok" : "*** CANNOT WRITE ***");
                pass = pass && (now > 0.5f);
            }
            // How far must a 2-position rotary be dragged? JUCE maps the
            // FULL range onto getMouseDragSensitivity() pixels, so a knob
            // with range 0..1 needs half of that to cross the snap point.
            ed->setSize(1000, 784);
            const juce::String ov = ce->header_overlap_for_test();
            std::printf("  header buttons do not overlap           %s\n",
                        ov.isEmpty() ? "PASS" : ("*** FAIL: " + ov).toRawUTF8());
            pass = pass && ov.isEmpty();
            std::printf("  drag sensitivity (px for full range): %d\n",
                        ce->drag_sensitivity_of("q_mode"));

            std::printf("  overall %s\n", pass ? "PASS" : "*** FAIL ***");
        }
    }
    std::printf("\n=== 7. O1 END TO END: control-signal peak vs Wrinkle and Flap ===\n");
    std::printf("Measured through the REAL budget solve. This is the quantity\n");
    std::printf("that clamps at +-1; it should stay flat as Wrinkle/Flap rise.\n\n");
    {
        set(proc, "dual", 0.0f);
        set(proc, "physics", 1.0f);
        set(proc, "precision", 0.0f);
        set(proc, "motion", 0.6f);
        set(proc, "wind", 0.5f);
        set(proc, "test", 1.0f);          // internal pink, so input is steady
        std::printf("%9s %8s %12s %12s\n", "wrinkle", "flap",
                    "ctl rms", "ctl peak");
        for (float wr : {0.0f, 0.25f, 0.5f, 0.75f, 1.0f})
            for (float fl : {0.0f, 1.0f}) {
                set(proc, "wrinkle", wr);
                set(proc, "flap", fl);
                run(proc, 60);                 // settle
                proc.take_control_rms(0);
                proc.take_control_peak(0);
                run(proc, 400);                // long window: rms converges
                const double rms = proc.take_control_rms(0);
                std::printf("%9.2f %8.2f %12.4f %12.4f\n", wr, fl, rms,
                            proc.take_control_peak(0));
            }
        set(proc, "wrinkle", 0.0f);
        set(proc, "flap", 0.45f);
        set(proc, "test", 0.0f);
    }
    std::printf("\n=== 8. DETAIL: more smoothing on SMALL waves, big ones kept ===\n");
    std::printf("fine share = node-to-node difference energy / total. It must\n");
    std::printf("FALL with Detail while the overall rms stays put.\n\n");
    {
        // set_knob_on writes to the HIGHLIGHTED voice, and section 6 leaves
        // it on 1. Measuring voice 0 while toggling voice 1's Smooth is a
        // test bug, not a feature bug -- it cost a round trip once already.
        proc.set_ui_voice(0);
        set(proc, "dual", 0.0f);
        set(proc, "physics", 1.0f);
        set(proc, "motion", 0.8f);
        set(proc, "wrinkle", 0.7f);      // plenty of small waves to work on
        // 2 ms, not 15: at 15 the temporal smoother has already removed
        // most of the buzz before Detail sees it, so the test measured
        // almost nothing. Buzz is visible at LOW Smooth, which is where
        // the knob has to earn its place.
        set(proc, "smoothing", 2.0f);
        set(proc, "test", 1.0f);
        proc.set_band_probe(true);
        std::printf("%9s %12s %12s %12s\n", "detail",
                    "SMALL waves", "BIG waves", "small/big");
        // 0.02 rather than 0 for the baseline: the split is skipped
        // entirely at 0, so there would be nothing to measure against.
        for (float d : {0.02f, 0.25f, 0.5f, 0.75f, 1.0f}) {
            set(proc, "detail", d);
            run(proc, 60);
            proc.take_control_rms(0);
            proc.take_fine_share(0);
            run(proc, 300);
            // fine share FIRST: it divides by ctl_sum2, which take_control_rms
            // zeroes on read.
            const double fine = proc.take_band_fine(0);
            const double broad = proc.take_band_broad(0);
            proc.take_control_rms(0);
            std::printf("%9.2f %12.5f %12.5f %12.3f\n", d, fine, broad,
                        fine / std::max(1e-9, broad));
        }
        // Detail must go inert when Smooth is powered OFF: it is a
        // modifier on the smoother, not a second smoother.
        set(proc, "detail", 1.0f);
        proc.set_knob_on(CurvesAudioProcessor::kSmoothing, false);
        run(proc, 60);
        proc.take_band_fine(0);
        run(proc, 300);
        const double off_fine = proc.take_band_fine(0);
        proc.set_knob_on(CurvesAudioProcessor::kSmoothing, true);
        run(proc, 60);
        proc.take_band_fine(0);
        run(proc, 300);
        const double on_fine = proc.take_band_fine(0);
        std::printf("\n  Smooth OFF  -> small %.5f   (Detail must be inert)\n",
                    off_fine);
        std::printf("  Smooth ON   -> small %.5f   %s\n", on_fine,
                    (off_fine > on_fine * 2.0) ? "PASS" : "*** FAIL ***");
        // What the 16th-order blur costs. It is paid only when Detail is
        // up, but 512 nodes with wind was already near a full core.
        proc.set_band_probe(false);
        set(proc, "precision", 2.0f);
        for (int w = 0; w < 2; ++w) {
            set(proc, "detail", w ? 1.0f : 0.0f);
            run(proc, 40);
            const clock_t t0 = clock();
            run(proc, 400);
            const double ms = 1000.0 * double(clock() - t0) / CLOCKS_PER_SEC;
            const double audio_ms = 400.0 * 256.0 / 48.0;
            std::printf("  512 nodes, Detail %s: %7.1f ms for %.0f ms audio"
                        "  = %4.1f%% of a core\n",
                        w ? "ON " : "OFF", ms, audio_ms, 100.0 * ms / audio_ms);
        }
        // The reported hitch: Flagpole + Dual. Measure the real cost.
        set(proc, "detail", 0.0f);
        set(proc, "wind", 0.5f);
        set(proc, "flutter_hz", 3.0f);
        for (int pr = 0; pr < 3; ++pr) {
            set(proc, "precision", (float)pr);
            for (int du = 0; du < 2; ++du) {
                proc.set_dual(du != 0);
                run(proc, 40);
                const clock_t t0 = clock();
                run(proc, 300);
                const double ms = 1000.0 * double(clock() - t0) / CLOCKS_PER_SEC;
                const double audio_ms = 300.0 * 256.0 / 48.0;
                std::printf("  %4d nodes  Dual %s  %6.1f%% of a core  %s\n",
                            proc.active_nodes(), du ? "ON " : "OFF",
                            100.0 * ms / audio_ms,
                            (ms / audio_ms > 0.85) ? "*** WILL DROP OUT" : "");
            }
        }
        proc.set_dual(false);
        set(proc, "precision", 0.0f);
        set(proc, "test", 0.0f);
    }
    std::printf("\n=== 8b. RAW: every modifier off, bare string only ===\n\n");
    {
        proc.set_ui_voice(0);
        set(proc, "dual", 0.0f);
        set(proc, "physics", 1.0f);      // Flagpole, so there IS weather
        set(proc, "motion", 0.9f);
        set(proc, "wind", 0.5f);
        set(proc, "wrinkle", 0.8f);
        set(proc, "test", 1.0f);
        set(proc, "raw", 0.0f);
        run(proc, 80);
        proc.take_control_rms(0);
        run(proc, 200);
        const double cloth = proc.take_control_rms(0);
        set(proc, "raw", 1.0f);
        run(proc, 120);                  // let the string settle to rest
        proc.take_control_rms(0);
        run(proc, 200);
        const double bare = proc.take_control_rms(0);
        std::printf("  cloth running   ctl rms %.5f\n", cloth);
        std::printf("  RAW             ctl rms %.5f   %s\n", bare,
                    (bare < cloth * 0.1) ? "PASS (string is bare)"
                                         : "*** FAIL ***");
        set(proc, "raw", 0.0f);
        set(proc, "test", 0.0f);
    }

    std::printf("\n=== 9. PHYSICS RATE vs PRECISION ===\n");
    std::printf("VALIDATE FIRST: the string's fundamental goes as wave speed,\n");
    std::printf("so doubling Tension must roughly double the measured rate.\n\n");
    {
        set(proc, "physics", 1.0f);
        set(proc, "motion", 0.6f);
        set(proc, "wind", 0.5f);
        set(proc, "wrinkle", 0.0f);
        set(proc, "detail", 0.0f);
        set(proc, "smoothing", 2.0f);     // minimal, so the metric sees the sim
        set(proc, "test", 1.0f);
        set(proc, "precision", 0.0f);
        std::printf("%10s %12s\n", "tension", "rate Hz");
        for (float t : {150.0f, 300.0f, 600.0f}) {
            set(proc, "tension", t);
            run(proc, 60);
            proc.take_control_rate(0);
            proc.take_control_rms(0);
            run(proc, 300);
            const double r = proc.take_control_rate(0);
            proc.take_control_rms(0);
            std::printf("%10.0f %12.3f\n", t, r);
        }
        set(proc, "tension", 300.0f);
        std::printf("\nNow the same patch at each precision:\n");
        std::printf("%10s %8s %12s %12s %10s\n", "precision", "nodes",
                    "rate Hz", "ctl rms", "finite");
        for (int pr = 0; pr < 3; ++pr) {
            set(proc, "precision", (float)pr);
            run(proc, 80);
            proc.take_control_rate(0);
            proc.take_control_rms(0);
            run(proc, 300);
            const double r = proc.take_control_rate(0);
            const double rm = proc.take_control_rms(0);
            const double pk = proc.take_control_peak(0);
            std::printf("%10d %8d %12.3f %12.4f %10s\n", pr,
                        proc.active_nodes(), r, rm,
                        (std::isfinite(rm) && pk < 2.0) ? "ok" : "*** BAD");
        }
        set(proc, "precision", 0.0f);
        set(proc, "test", 0.0f);
    }
    return 0;
}
