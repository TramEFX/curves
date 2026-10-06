# Curves — working notes for Claude

A JUCE/CLAP plugin. **A physics sandbox for spectral control**: a
physics-driven string is simulated continuously and its shape is read off as
a parameter across the frequency axis. The string pushes the parameters.

It generalises two ways, and both are first-class — keep them in mind before
assuming where a change belongs:

- **What drives the string** is a selectable *physics mode*. Cloth-in-wind
  ("Flagpole") is deep, Free works, Gust and Elastic are thin. The cloth is
  **not the product**, it is the first mode built in depth.
- **What the string drives** is a *subscriber*. EQ gain and the spatial
  pair (width + placement) are built, on a mid/side bank; per-node Q and
  cutoff modulation are planned on the same control signal.

`StringSim` publishes nodes and knows nothing about filters. A new mode
replaces the physics without touching the bank; a new subscriber attaches
without touching the physics. The **editor** does not yet honour this — the
CLOTH & WIND panel is Flagpole-shaped and hard-coded, so a new mode is UI
work too. See `ORCHESTRATION.md` §1.

## This is a SHARED machine — the owner is usually using it

**The owner is often doing other things on this PC while work is in
progress.** That has two consequences, and both have already caused a wrong
diagnosis.

**Synthetic input is intrusive and unreliable here.** Driving the UI
(`mouse_event`, `SetForegroundWindow`, `AttachThreadInput`) **steals focus
and moves the physical cursor** — it will interrupt whatever the owner is
doing, and a stray click of theirs lands in the test and corrupts it.

- **Default to headless verification.** The test binaries under
  `build/Release/` cover almost everything: `scenarios`, `gain_test`,
  `display_test`, `wind_test`, `spatial_test`, `mid_test`, `waves_test`,
  `patch_test`, `tension_test`, `lines_test`, `rnd_test`, `hash_probe`,
  `stereo_test`, `overlap_probe`, `display_src`, `bench`. Prefer these over
  driving the GUI, always.
- **Screenshots are safe; clicks are not.** `PrintWindow(hwnd, dc,
  PW_RENDERFULLCONTENT)` grabs the window without taking focus or touching
  the cursor. Use it freely. Synthetic *input* should be announced first and
  kept brief.
- **Never trust a negative result from a UI harness** without first checking
  it against a control with an unmistakable effect (the theme button). A
  harness that silently delivers nothing looks exactly like a broken
  feature. This happened: the walls were reported dead when in fact no click
  was landing.
- `mouse_event` absolute coordinates are **primary-monitor-relative** unless
  `MOUSEEVENTF_VIRTUALDESK` is set *and* they are normalised against the
  virtual desktop. Its origin here is **x = −1920** (a monitor left of
  primary), so getting this wrong sends every click to the wrong screen.
- If the owner says they are at the keyboard, **do not drive the UI at all**
  — ask them to perform the interaction, or verify it headlessly.

## Build

`cmake` is **not on PATH**. Use:

```
"/c/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe" --build build --config Release
```

- **Reaper and the standalone lock their binaries.** With Reaper open the
  VST3 deploy fails on `moduleinfo.json`; with the standalone running the
  link fails with LNK1104. Kill the standalone *before* building, and ask
  the owner to close Reaper rather than killing it.
- VST3 auto-deploys to `C:\Program Files\Common Files\VST3`. **CLAP does
  not** — build tree only, no copy step (known debt).
- Reaper caches the parameter list. After adding or removing a parameter,
  tell the owner to rescan.

## Before starting anything: read SPEC-0.0.2 §5

It is the blocked / pending / open status. One item is **blocked on an owner
decision** (band overlap, B1) and it gates the spatial subscriber's
placement control — do not try to "fix" placement without reading it.

## The one check that must pass

```
python prototype/run_scenarios.py
build/Release/scenarios.exe prototype
```

then cross-diff the **`value` column parsed as floats**. Currently
**12 scenarios, 119,936 cells, 0 mismatches**.

> A naive byte diff of those CSVs reports ~156,000 mismatches and means
> nothing: Python writes `0.0` where C++ writes `0.000000`, and the scenario
> column is tagged `cpp_pluck` vs `pluck`. Do not panic at it.

New physics goes in **both** `src/string_sim.cpp` and
`prototype/string_sim.py` with identical arithmetic order, default-off so the
frozen baselines stay byte-identical — and then **a scenario must exercise
the new path in both**, or validation passes while covering a configuration
the plugin never uses.

## Invariants (see ORCHESTRATION.md for the full statements)

1. **WYSIWYG** — the bright curve is what the audio goes through. Any new
   stage added *after* the bank must be folded into the display too; this
   has been violated twice (Steady, then Mix).
2. **Bit-exact cross-validation** — above.
3. **Image provenance** — the stereo image moves only when told to. Every
   per-band coefficient (mid gain, side gain, mid→side cross-feed) must be a
   deterministic function of the control surface and the knobs, and
   therefore **drawable**. Forbidden: any detector that measures the
   channels separately and lets the result change the balance between them.
   Permitted: deliberate stereo transforms driven by the control surface.
   Content-driven gain reduction stays channel-linked — the Leveler's
   control signal is `max(|L|,|R|)` and Steady's detector is the mid, both
   for this reason. (Renamed from "channel coherence" 2026-10-06; the old
   name described a mechanism, not the principle.)

## Habits that have paid off

- **Measure before fixing.** Several confident hypotheses have been wrong
  here; measurement caught all of them.
- **Validate a metric on a known signal before trusting it.** Two metrics
  (wave direction, then motion frequency) gave confidently wrong answers.
- **When a knob "does nothing" or "needs turning way up", it is usually
  fighting something structural, not short of range.** Four cases so far.
  Check the *level* across its sweep first.
- **Measure any generator's level before shipping it** — one was 4× full
  scale.

## Documentation workflow — local first, then Notion

**The repository is the source of truth.** There is a summary page in Notion:

> Curves — a physics sandbox for spectral control
> `https://app.notion.com/p/3ee0c6e138f2818ca142e6f2b33e7c2b`

It is written **from** the repo docs, never the other way round. The order is
always: change the code → update `ORCHESTRATION.md` / `SPEC-0.0.2.md` /
`SESSION-LOG.md` → **then** push the summary to Notion. If the two disagree,
the repo wins and the Notion page is the bug.

Notion holds the *product-level summary* — what this is, the two axes, status
at a glance. It does not duplicate the control reference or the measurements;
those stay in the repo, which is where they are verifiable.

Remote: `github.com/TramEFX/curves` (**private**).

## Documentation map

| file | role |
|---|---|
| `docs/SESSION-LOG.md` | running log: what was measured, what was decided, what is open |
| `docs/SPEC-0.0.2.md` | current control reference, all 28 parameters; **§5 is the blocked/pending/open status** |
| `docs/SPEC-0.0.1.md` | superseded, kept |
| `ORCHESTRATION.md` | architecture and decision record |
