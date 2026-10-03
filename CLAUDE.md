# Curves — working notes for Claude

A JUCE/CLAP plugin: a 1D damped wave-equation string drives a multiband
filter bank. The string pushes the parameters.

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
  `patch_test`, `bench`. Prefer these over driving the GUI, always.
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

## The one check that must pass

```
python prototype/run_scenarios.py
build/Release/scenarios.exe prototype
```

then cross-diff the **`value` column parsed as floats**. Currently
**11 scenarios, 104,576 cells, 0 mismatches**.

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
3. **Channel coherence** — one instance is one string; nothing inside a
   single instance may treat L and R differently. Stereo comes from running
   separate instances.

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

## Documentation map

| file | role |
|---|---|
| `docs/SESSION-LOG.md` | running log: what was measured, what was decided, what is open |
| `docs/SPEC-0.0.2.md` | current control reference, all 22 parameters |
| `docs/SPEC-0.0.1.md` | superseded, kept |
| `ORCHESTRATION.md` | architecture and decision record |
