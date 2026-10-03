# Curves — Orchestration Document

> **Documentation map.** `docs/SPEC-0.0.2.md` is the current control
> reference (every knob → parameter → function). `docs/SESSION-LOG.md` is the
> running work log: measurements, root causes, dead ends and open threads.
> `docs/SPEC-0.0.1.md` is superseded, kept for history. This file remains the
> architecture and decision record.
>
> **Living document.** This file is the source of truth for the current state of the Curves project. Update it whenever a decision is made, changed, or superseded. Section [Change Log](#change-log) must be appended on every material edit.

---

## 1. What Curves Is

**Curves** is an **EQ-based effect sandbox**. A parametric EQ where the user draws shapes directly on the curve, and those shapes are *alive* — governed by physics, able to move, travel, and respond to audio.

Core identity:

- **Simple in front, deep underneath.** Straightforward, tank-like, sounds good *because* of rigorous engineering. All advanced parameters exist under the hood from day one, exposed only in an advanced submenu. Vital-style philosophy.
- **The string pushes the parameters.** This is the central architectural inversion (see §3). Not "parameters render a curve" — the curve *is* the parameter source.
- **Target audience:** the nerds and sound designers who will find sounds nobody else can. But it must remain playable and simple by default.
- Early use cases: a chorus-like effect that doesn't rely on detuning (spectral motion instead of pitch motion); a creative toolbox for EDM sound design.

**Name:** "Curves" — confirmed 2026-09-29 (formerly "Curiosity").

---

## 2. Product Vision (Feature Set)

From the original brief, in priority order:

1. **Parametric EQ with direct drawing** — shapes drawn on the curve are heard immediately.
2. **Line/shape/curve generator** — generate shapes procedurally (see §7).
3. **Multiple instances running in parallel paths** — semantic decision pending, leaning toward visual lanes over one control string (see §4, Open Decisions).
4. **Unique physics attached to the line** — e.g., the "Flagpole" preset: highest frequency is an anchor; wind blows leftward down the visual EQ line and the drawn curve behaves as if blown. Other preset families fall out: viscosity, elasticity/spring-back, magnetic wells, tidal drift.
5. **Slow variable wave function — shapes travel** across the frequency axis.
6. **"Wave imprint" (later)** — a lightweight IR-like capture. Leaning toward: capture the *string state/motion* as the imprint. Freeze-at-capture semantics first.
7. **Audio → string (the door, §8)** — audio creates a string; later, audio plays the string (sympathetic resonance).

---

## 3. Core Architecture

### 3.1 The Central Inversion

> "Normally parameters push the string. Here, the string pushes the parameters."

The string is a **first-class object in the dataflow** — a live, shared medium:

- The string is a fixed-resolution **node array** (log-spaced across frequency). Drawing simply writes displacement values into nodes.
- A **physics engine owns the nodes.** Nothing else writes to them except through defined interfaces (drawing = displacement, forces = forces, audio = forces, generator = initial conditions).
- The **filter bank reads the nodes** every block, forever. The engine never asks "what did the user draw" — only "what does the string look like right now."
- Drawing, presets, wind forces, traveling shapes, audio analysis — all are just *writers* to the same nodes.

### 3.2 The String Is a Real String

The physics is an actual **1D damped wave equation** on the nodes, not a springy drawing surface:

- Drawing = displacing the string; release lets it oscillate and settle.
- Wind = a persistent force field along the frequency axis. Flagpole = wind + anchored end.
- Traveling shapes = genuine wave propagation (reflects, interferes). The "slow variable wave function" becomes physics for free.
- The detune-free chorus emerges naturally: a slowly vibrating string makes the whole spectrum sway — spectral motion instead of pitch motion.
- Cost is trivial: finite-difference integration over N nodes, a few hundred multiply-adds per block at 256 nodes.

### 3.3 The String as a Universal Parameter Bus

Node values are stored **normalized** (−1..+1), not as dB. Any parameter can subscribe to the string:

- Node f → gain of filter at f (the EQ — default mapping)
- Node f → Q of filter at f
- Future subscribers: pan, drive, delay time, etc. The EQ is just the first subscriber.

This is what makes it a *sandbox* and not just an EQ.

**Selection criterion for subscribers (user framing, 2026-10-01):** the
parameter must *itself be a spectrum* — it has one meaningful, independent
value at every frequency. That is what makes the string's node array the
natural control surface for it, and it is the filter that decides what gets
built. "Stereo placement" was raised as a strong candidate; the broader
shortlist, ordered by fit:

| Candidate | Values per node | Fit | Notes |
|---|---|---|---|
| **Gain** | 1 | shipping | the first subscriber |
| **Stereo width** (M/S ratio per band) | 1 | **excellent** | mono bass / wide highs is already a classic move; width is a *ratio*, so it needs no band isolation |
| **Q** | 1 | decided (§5) | per-node Q |
| **Stereo placement** (pan per band) | 1 | good | differential boost/cut between L and R; a *spectral* pan rather than a hard one |
| **Drive / saturation** | 1 | good | per-band nonlinearity depth |
| **Gate / expansion threshold** | 1 | good | natural extension of Steady, which is already a per-band follower |
| **Delay time** | 1 | expensive | needs a delay line per band; this is spectral smear / dispersion |
| **Phase / allpass rotation** | 1 | expensive | overlaps the "phase wrecking" open decision |

**Topology consequence for the stereo pair — important, and it splits the
two cleanly.** The bank is currently `out = x + Σ bandpass·(G−1)`: a dry
path plus summed peaking bands. That can *boost or cut* a band, but it
cannot **move** one, because the dry path always carries the full-bandwidth
signal at its original position. So:

- **Width works with the topology as it stands.** Width is a ratio, not a
  position: split to M/S, run a bank instance on each, and let the string
  drive the per-band S-vs-M gain. Cost is ~2× the bank, no topology change,
  and the string supplies exactly the one scalar per node it needs.
- **Pan needs more care.** As a differential L/R boost/cut it works and is
  musically useful, but it can never be a hard pan while a dry path exists.
  A true per-band pan wants a complementary/crossover bank that *partitions*
  the spectrum instead of summing onto dry — which is a bigger change and
  interacts with the parallel-path Open Decision.

**Watch item:** the ear is far more sensitive to time-varying inter-channel
differences than to gain wobble, and a *moving* per-band pan or width is
exactly that. Any stereo subscriber needs smoothing at least as strict as
gain, probably stricter, and a mono-compatibility check — a wide, moving
spectral image can collapse badly when summed.

### 3.4 Visual String ≠ Control String

Decoupled from day one:

- **Visual string:** rendered at screen resolution for display and input.
- **Control string:** fixed node count, deterministic, sample-rate-stable.

Drawing deposits values into control nodes by sampling the drawn path *at node frequencies*. Visual resolution never changes what a drawing *means*, only its fidelity.

---

## 4. Resolution ("Precision Modes")

256 nodes was deemed **not enough**. Node count is user-selectable via precision modes. Three costs scale independently and are exposed as a tuple (sim resolution / bank resolution / sim tick rate):

| Mode (illustrative) | Sim nodes | Bank bands | Sim tick |
|---|---|---|---|
| Fine | 256 | 256 | per-sample |
| Ultra | 512 | 512 | every 2–4 samples |
| Analysis | 2048 | 512 | every 2–4 samples |

- Physics sim scales linearly with nodes; running the sim every N samples with interpolation keeps high resolutions cheap. Nobody hears the difference on multi-millisecond string dynamics.
- The **filter bank is the real CPU budget.** Every subscribed node is a live filter. At high band counts, use SVF/TPT-style filters (stable and cheap); naive biquad stacking produces gain errors between adjacent bands.
- The bank always reads from the **control string**, never from the visual string.

---

## 5. Q, Line Shaping, and the Sim Boundary

- **Per-node Q:** each node is a band — displacement = gain, Q = node Q. Editing scheme (proposed): vertical drag = gain, modifier-drag = Q.
- **Line shaping (hard boundary, decided):** interpolation between nodes (linear, spline, power curve, sine-arc, etc.) is a **visual + deposit-layer concern only**. Drawing is sampled at node frequencies; the sim owns the nodes afterward. Physics never operates on shaped segments — sim state must never depend on rendering decisions.
- Shaped lines (N control points + a curve law) double as a **shape generator primitive**: they degrade gracefully to node values.

---

## 6. Smoothing (The Sound of the Plugin)

Airtight and boring by design — one mechanism:

1. The string sim already produces smooth motion (that's what damping is). Don't sample it sparsely; avoid discontinuities into the parameter path.
2. Every subscriber gets **one one-pole smoother** toward the current sim value, with a **per-subscriber time constant**. That is the entire smoothing system. Per-sample, fixed math, no state to corrupt.
3. Advanced menu exposes: sim tick rate, per-subscriber smoothing time, optional **velocity coupling** (subscribers react to string velocity, not just position — toggles "sways gently" vs. "plucked aggressively").

No crossfades, no lookup tables, no exception paths. One continuous, band-limited path from physics to filter coefficients with one deliberate filter on it.

Rationale: a vibrating string means filter parameters are always in motion — that is frequency modulation of the filters, which produces sidebands. Whether that sounds lush or cheap is almost entirely the smoothing timescale. This is the single most important under-the-hood knob.

---

## 7. Shape Generator & Shape Library

All shape origins are the **same object**: a node array + metadata (Q profile, line law, source tag).

Origins:

1. **Drawn** by the user.
2. **Physics frames** — the generator runs a physics preset with chosen parameters and captures a frame; presented as a static shape. Pure procedural code, zero inference.
3. **Analyzed audio** (see §8).
4. **Line-law primitives** — N control points + a curve law (§5).

**Tiny model stance (decided):** the valuable "tiny model" is an **inverse map** ("given a target spectrum, produce string + physics parameters approximating it") — *later*, if at all. If trained: tiny (small MLP), run **offline only** (shape factory in a menu, never the audio path). Real-time guarantees and ML in the audio path do not mix.

**Shape file format:** one open, versioned, human-readable text format (JSON) defined **on day one**. The physics-preset→shape bridge is just an exporter. This is the community flywheel — people trade small text files.

---

## 8. Audio ↔ String (The Door)

Two directions, explicitly ranked by the project owner:

1. **Audio creates a string — PRIORITIZED.** "Listen, then draw": FFT analysis over a user-selected window → band energies at node frequencies → displacement per node (dB offset, match-EQ style) → frozen as a shape the user keeps, edits, or uses as physics initial conditions. Deterministic, testable, zero ML. **Build this in v1 scope.**
2. **Audio plays the string — the door.** Per-band envelope followers apply force to string nodes (sympathetic resonance; a kick drum plucks the low end). Introduces audio→string→audio feedback: needs a force limiter and a one-way mode. **Build the interface now, keep the door closed** until the sim tick rate + subscriber topology decision (§Open Decisions) confirms it can be added without a feedback redesign.

---

## 9. Technology Stack

| Component | Choice | Notes |
|---|---|---|
| Framework | **JUCE** (v8, AGPLv3 for open source) | Batteries included. Alternative considered: iPlug2 (more permissive, fewer batteries). |
| Plugin formats | **CLAP first, VST3 second** | CLAP is MIT, modern, has per-voice/per-note modulators for later. VST3 SDK GPLv3 — fine for open source. |
| GUI | JUCE, budget generously | Full-curve redraw at 60fps with input handling is a bigger chunk than it looks. Feel matters. |
| Math | Eigen (or hand-rolled) | Don't over-engineer. |
| Open-source credits | chowdsp_utils (filter/modulation infrastructure), Vital (reference elegance), Surge XT (engineering discipline), FFTConvolver (HiFi-LoFi, for the wave imprint) | Credit where due; project is open source. |

### 9.1 Borrowable Sources (licensing map)

**Licensing baseline:** the project is AGPLv3 (driven by the JUCE choice). Consequence: GPLv3 sources are cleanly borrowable (they merge into AGPL without conflict); permissive sources (BSD/MIT) are borrowable with attribution; LGPL sources require care (dynamic linking or strict separation) — prefer to avoid in plugin code.

**Important correction on record:** ReaEQ (REAPER) is **not** open source — closed Cockos code. The legitimate path into the Cockos world is the **WDL library** (open-sourced REAPER infrastructure) and REAPER's plaintext JSFX effects as reading material only.

| Source | License | Take |
|---|---|---|
| **chowdsp_utils** (J. Chowdhury) | Permissive, BSD-style (verify per repo) | **Primary dependency from M1.** TPT/SVF filter classes, per-sample parameter smoothing, JUCE-native wrappers. This *is* our filter-bank + smoothing layer; we write only string + physics. |
| **Vital** | GPLv3 | Study, not dependency. Closest existing relative to "string pushes parameters" — learn its modulator→parameter routing at audio rate. Pasteable if ever needed (license-compatible). |
| **Surge XT** | GPLv3 | Study, not dependency. Filter-bank design and clean handling of hundreds of modulated parameters; GUI discipline. |
| **WDL** (Cockos) | Historically LGPL — verify current terms | FFT, resampling primitives. Avoid static linking into plugin code unless terms check out. |
| **HiFi-LoFi/FFTConvolver** | Permissive | Wave imprint engine (M6), as already planned. |
| **iir1** (B. Porr) | Permissive | Simple biquad/shelf IIR — for **offline tools** (analysis, shape generation) where modulation-grade filters aren't needed. |
| **LSP / Calf / x42-fil4** | GPLv3 | Production parametric-EQ reference: adjacent-band interaction handling, spectrum overlays. Ideas over code (GUIs are GTK-land). |
| **Zavalishin, *The Art of VA Filter Design*** | Free book, not code | Canonical TPT/SVF theory. The "why" behind our filter choice. |
| **KissFFT** | BSD | Alternative FFT primitive for the M4 analysis path if JUCE's isn't preferred. |

**Adoption policy:**

1. chowdsp_utils is the only real dependency adopted at M1; everything else is study material or arrives at the milestone that needs it.
2. No whole-EQ adoption — assemble from filter primitives. The bank is ours; the filters are chowdsp's.
3. FFT analysis path (M4) is written in-house (~50 lines of band mapping on top of an FFT primitive).
4. Every borrowed piece goes in the credits file at adoption time, not at release time.

---

## 10. Decisions Made

- ✅ String is the parameter source; physics engine owns nodes; filter bank reads them (§3.1).
- ✅ String has real 1D damped wave-equation dynamics (§3.2).
- ✅ Nodes normalized (−1..+1); string is a universal parameter bus; EQ is the first subscriber (§3.3).
- ✅ Visual string and control string are separate (§3.4).
- ✅ Resolution modes / precision modes; 256 is a floor, not a ceiling (§4).
- ✅ Per-node Q (§5).
- ✅ Line shaping is visual/deposit-layer only; never touches the sim (§5).
- ✅ One-pole per-subscriber smoothing, per-sample, with per-subscriber time constant (§6).
- ✅ Audio-creates-string is prioritized over audio-plays-string; both will exist (§8).
- ✅ Tiny model = offline inverse mapping, later; procedural physics-frame generator first (§7).
- ✅ Open text shape format on day one (§7).
- ✅ JUCE + CLAP-first + VST3 (§9).
- ✅ **The string is DC-free** (added 2026-09-29): mean position and mean velocity are removed every tick. Rationale: the wave equation is blind to uniform displacement (tension can never act on it), so a drawn shape's mean would ring forever at √stiffness when damping is low — the "stuck slow waves" bug reported by the project owner. Semantic consequence: a globally-shifted curve is not a string state; global level belongs to the Drive knob. Force presets must express their look within the zero-mean constraint (wind amplitudes were retuned accordingly).
- ✅ **The string is TILT-free as well as DC-free** (added 2026-10-01): the least-squares linear trend is removed every tick, for exactly the reason the mean is. Tension is blind to a linear ramp (the Laplacian of a ramp is zero in the interior), so mode 1 is restored only by the artificial `stiffness` spring and the two Neumann end nodes — it rings at ~0.6–0.9 Hz, barely responds to the Stiffness knob, outlives every real wave, and under wind is re-excited indefinitely. Semantic consequence, mirroring the DC decision: a tilted curve is not a string state. Spectral tilt, if wanted, belongs in the gain mapping as a static control where it cannot ring. `remove_tilt` defaults true; the frozen validation scenarios opt out to stay byte-identical.
- ✅ **HARD ANCHOR — WYSIWYG is inviolable** (user, 2026-10-01): "shape of string on screen should ALWAYS be perfectly representative of the eq being applied." This outranks every other consideration and constrains all future work. Mechanism: `FilterBank::gains()` is the declared truth for the display; `publish_display()` derives the bright curve from the bank's actual smoothed + Steady-modulated gains, never from the sim, (this was *claimed* from 2026-09-29 but only became **true** on 2026-10-02 — Steady's duck was applied to a local and never written back, hiding up to 4.52 dB of real gain reduction from the curve) and normalizes with the SAME ±1 clamp the audio path uses. The faint reference curve is the raw sim and is labelled as such. Consequence for design: any amplitude mechanism must keep the string inside ±1 in normal use, because beyond that the EQ flat-tops — the display stays truthful, but the product looks and sounds clipped.
- ✅ **ONE INSTANCE IS ONE STRING — channel coherence inside, stereo by instancing** (user, 2026-10-02): "the goal for this is to run multiple instances within a sandbox that you can run in parallel left and right channels independently for truly interesting stereo effects … it still needs to work on a single instance for stereo throughput, but running one left and right is really where the sandbox aspect gets powerful."

  Consequence, and it is a general rule rather than a note: **no process inside a single instance may treat L and R differently.** One string drives one coherent spectral process, applied identically to whatever it is fed. Anything that diverges per channel inside one instance is a dynamic stereo-image mover and therefore a defect.

  This settled a real question (whether the display should draw Steady's max or mean duck) by dissolving it: with a mono detector there is exactly one true curve, so the hard anchor is satisfied exactly rather than approximated. `OutputLeveler` already followed this rule — its control signal is `max(|L|,|R|)` — and `FilterBank`'s Steady now does too.

  Deliberately NOT covered by this rule: driving a *different spectrum* with the string — stereo width alone, or alongside EQ frequency. That is a wanted feature, to be built on purpose with its own control surface, not something that should fall out of per-channel followers by accident.

- ✅ **Smoothing lives in the log domain, on the control signal** (2026-10-02): the one-pole belongs on `disp` — the string's own coordinate — not on the band's linear gain. The x axis is log frequency and `disp` maps linearly to dB, so the whole chain is linear physics on a log-log graph with Range dB as a single memoryless scale at the end. Putting the smoother downstream in `exp(disp)` put Range *inside* a recursive filter, which is not scale-invariant: measured, the drawn curve moved up to 0.298 of full scale from a Range change alone, and boosts ran 2.21x faster than cuts at Range 18. §6 specified the correct form from the start; the implementation had drifted. The bank now holds no time state of its own and walks to each block target geometrically, which is exactly linear in dB for one multiply per band per frame.

- ✅ Name: **Curves** (renamed from "Curiosity", 2026-09-29).

## 11. Open Decisions

- ⬜ **Parallel-path semantics:** parallel lanes that sum (phase cancellation becomes a feature: "phase wrecking" mode) vs. one control string with multiple visual layers/modifiers. *Lean: build lanes-as-visual-layers first, true parallel summing as an advanced toggle later. Must be settled before the filter-bank topology is locked.*
- ⬜ **Sim tick rate + subscriber topology** — gates whether "audio plays the string" (§8.2) needs a feedback redesign. *Next technical discussion queued.*
- ⬜ **Exact transfer spec** (subscriber mapping: offset, scale, bipolar mode, smoothing) — where sound character is finalized. *Queued; expected shorter than it looks once smoothing is pinned.*
- ⬜ **Wave imprint semantics:** freeze-at-capture (simple, predictable) vs. continuously updating (cool, expensive). *Lean: freeze first.*
- ⬜ Frequency axis: log-spaced assumed; exact band mapping (e.g., log vs. mel vs. custom warp) not finalized.

---

## 12. Milestones

### M0 — Foundations (repo, skeleton) — ✅ COMPLETE 2026-09-29
- Repository, license decision (AGPLv3 assumed), build system, plugin shell in CLAP + standalone.
- Node-array data structure, open JSON shape format v0 with round-trip tests. *(shape format v0 still pending — moved into M1)*
- **Exit:** shell builds and loads in a host; shape files save/load. *(build + standalone smoke test: done; shape format deferred to M1)*

### M1 — The String (vertical slice, the whole product in miniature) — ✅ **GATE PASSED** 2026-09-29 ("sounds GREAT" — project owner ear test in Reaper)
- One lane. Drawn curve → node array. 1D damped wave-equation sim at fixed node count (~128–256), per-sample or interleaved tick.
- Filter bank reads nodes (one filter per node; SVF/TPT). One-pole per-subscriber smoothing.
- **Implemented:** `src/filter_bank.h/.cpp` (128 TPT peaking bands, log-spaced 30 Hz–18 kHz, per-band one-pole gain smoothing; cutoff modulation deferred to M2); `PluginProcessor` (sim ticks every 4 samples, node positions → ±max_gain_db targets, spinlock mailboxes both directions, 5 params: Damping/Stiffness/Tension/Smoothing/Drive); `PluginEditor` = drawable canvas (left-drag deposits held shape, release rings, right-click flattens, 30 Hz display of live sim).
- **GUI-verified:** stroke deposit renders (orange preview while held); after release the live string visibly rings and propagates (zoomed capture shows traveling ripples). Audio output path runs uncrashed; **the subjective "feels amazing" gate requires human ears** — user must listen (standalone input is muted by design; test via DAW with the VST3/CLAP, or configure standalone audio settings).

### M2 — Physics personality — CODE COMPLETE 2026-09-29, ear/feel test pending
- Force system: wind fields, anchors, spring-back, damping controls. Flagpole preset working.
- Velocity coupling toggle. Sim tick-rate option. Precision modes (sim/bank/tick tuples).
- **Implemented:** `Physics` parameter (0–3: Free / Flagpole / Gust / Elastic) — anchors + force fields applied engine-side when the mode changes (force amplitudes scale with node count); selecting a preset also sets Damping/Stiffness/Tension knobs (editor-side). `Precision` parameter (0–2: Fine 128 nodes/÷4 tick, Ultra 256/÷2, Analysis 512/÷2) — sim+bank rebuilt on change (allocation on audio thread accepted for M2, flagged for message-thread rebuild). `Velocity` coupling parameter (0–1): gain targets = position + velocity/tension — tension-independent by normalization. Editor: 11 knobs total, anchor pin marker drawn in Flagpole mode, status line shows active preset.
- **Exit (pending user feel-test):** Flagpole preset feels physical, not animated. Zipper-free at all settings.

### M3 — Shape factory & library
- Line-law primitives (control points + curve law). Physics-preset→frame exporter. Shape library browser reading the JSON format.
- **Exit:** a user can save, load, and share shapes as text files.

### M4 — Audio creates a string
- FFT analysis window → node displacements → frozen shape / initial condition. Match-EQ-style reference handling.
- **Exit:** "listen, then draw" round-trips: analyze audio, apply resulting shape, hear the imprint.

### M5 — Advanced submenu & tank-polish
- Full parameter exposure under the hood (smoothing times, tick rate, Q editing, oversampling, phase/summing modes). Default UI stays curated and simple.
- Determinism, sample-rate, edge-case, and host-hardening passes. Preset system.
- **Exit:** beta quality — "tank" standard.

### M6 — The door opens
- Audio-plays-string: per-band envelope followers → forces, with force limiter and one-way mode.
- Wave imprint (freeze-at-capture) via FFTConvolver-style engine.
- **Exit:** sympathetic resonance demo; imprint round-trips.

### M7 — Sandbox expansion (post-1.0)
- Parallel-path semantics per Open Decision. Non-gain subscribers (Q, pan, drive, delay). True parallel summing / phase-wrecking mode. Optional offline inverse-map model.

---

## 13. Risks & Watchlist

- **Zipper noise** is the #1 technical killer — every time-varying parameter path must be smoothed per-sample. M1 exists partly to prove this out.
- **Adjacent-band interference** at high bank resolutions (biquad stacking) → SVF/TPT filters; verify at 512+ bands early.
- **The drawing must feel good** — GUI/input polish is a large, easily underestimated work item.
- **Scope gravity:** every feature after M1 is optional expansion. Protect the vertical slice.
- **AGPL implications** for JUCE — fine for open source; revisit only if licensing strategy changes.

---

## 14. Prototype — String Sim (current work)

Location: `prototype/` (Python spec) and `src/` + `CMakeLists.txt` (validated C++ port).

**M0 status (2026-09-29):**

- ✅ Toolchain installed: MSVC Build Tools 2022 (v14.44) + bundled CMake, via winget. Git for Windows pre-existing. No standalone Python compiler needed for the plugin path.
- ✅ Repo initialized (`main` branch): `src/` (sim + plugin), `prototype/` (Python spec), `docs/`, `presets/`, `resources/`; `.gitignore`, `README.md`, full AGPLv3 `LICENSE`.
- ✅ **C++ port validated bit-for-bit against the Python spec**: all 12,160 CSV cells across pluck/flagpole/travel match exactly (max diff 0.0 at 6-decimal precision). `curves::StringSim` (`src/string_sim.h/.cpp`) is now the canonical implementation; the Python module remains the human-readable spec.
- 🔄 Plugin shell in progress: CMake FetchContent of JUCE 8.0.8 + clap-juce-extensions (`main`); minimal `CurvesAudioProcessor` (transparent passthrough) + placeholder editor; formats Standalone + VST3 native, CLAP via clap-juce-extensions (JUCE has no native CLAP — earlier CMake draft was wrong on this, corrected).
- ✅ **M0 EXIT ACHIEVED (2026-09-29):** all three formats built and the standalone was smoke-tested (launched, ran, clean kill): `VST3/Curves.vst3`, `CLAP/Curves.clap`, `Standalone/Curves.exe` under `build/Curves_artefacts/Release/`. Gotcha for the record: JUCE CMake format names are mixed-case — `Standalone`, not `STANDALONE` (invalid names are silently ignored).
- Note: python harness CSV writer had mislabeled tick/time columns; fixed and regenerated before the cross-validation.

**Build commands (Windows, Git Bash):**

```
CMAKE="/c/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe"
"$CMAKE" -S . -B build -G "Visual Studio 17 2022" -A x64 -DCURVES_BUILD_PLUGIN=ON
"$CMAKE" --build build --config Release
./build/Release/scenarios.exe prototype   # sim validation runner
```

**Validated (2026-09-29):**

- 1D damped wave equation over N=128 nodes, velocity-Verlet integration, unit node spacing, CFL guard (`wave_speed * dt <= 1`).
- Interfaces match the architecture exactly: `displace` / `pluck` / `set_force_field` are the *writer* interfaces; node positions `u` are what the future filter bank reads; velocities `v` kept for the velocity-coupling option.
- **Pluck:** pulse splits into two counter-propagating waves, reflects off anchored ends with inversion, rings down and settles (energy decay matches e^(−damping·t) exactly). This is the M1 go/no-go behavior — it works.
- **Flagpole:** pin the high node, wind force ramping toward the free (low) end → curve sags like a flag; wind release → springs back. Confirmed visually.
- **Travel:** localized bump propagates and reflects genuinely — the "slow variable wave function" is physics, not a crossfade, as designed.

**Learnings on record:**

- Integrator: velocity Verlet required. Semi-implicit Euler is marginally unstable for oscillatory modes at these parameters (|λ| ≈ 1); Verlet is bounded. Keep the CFL assertion in the C++ port.
- Energy metric gotcha: `sum(u² + v²)` is *not* the energy of a wave equation — most energy lives in the stretch term `c²·Σ(du/dx)²`. Use `Σv² + c²·Σ(du/dx)² + k·Σu²` for any sanity check or limiter work (relevant later for the audio-plays-string force limiter).
- Static sag under constant force is tension-limited (a Helmholtz balance, roughly `u_ss ≈ (f/k)·(1 − sech(√k·L/c))`), not simply f/k. Preset design must account for tension when aiming at a target sag depth.

---

## Change Log

- **2026-09-29** — Initial version. Captures full brainstorm state: architecture (string-as-parameter-bus, physics-owned nodes, filter-bank readout), resolution modes, Q/line-shaping boundary, smoothing design, shape generator/library stance, audio↔string direction, tech stack, decisions, open questions, and milestones M0–M7.
- **2026-09-29** — Renamed project from "Curiosity" to **Curves**. Folder moved to `ProjectsAI/Curves`; name removed from Open Decisions and recorded in Decisions Made.
- **2026-09-29** — Added §9.1 Borrowable Sources (licensing map). Key points: project is AGPLv3; ReaEQ is closed source (corrected on record); **chowdsp_utils adopted as the only real dependency at M1**; Vital/Surge XT are study material; no whole-EQ adoption; credits logged at adoption time.
- **2026-09-29** — **String sim prototype built and validated** (`Curves/prototype/`, Python): pluck/flagpole/travel scenarios all behave correctly; velocity-Verlet integrator chosen; energy-metric and tension-sag learnings recorded; see §14. Decision: prototype-first path chosen over M0 skeleton (user-confirmed). Next: M0 toolchain + repo, then C++ port of the sim in M1.
- **2026-09-29** — **M0 COMPLETE.** MSVC Build Tools 2022 + CMake installed; repo initialized (git `main`, AGPLv3 LICENSE, README, .gitignore); string sim ported to C++ (`src/string_sim.h/.cpp`) and **validated bit-for-bit against the Python spec** (12,160/12,160 cells, zero diff); JUCE 8.0.8 + clap-juce-extensions fetched via FetchContent; **plugin shell built in all three formats (Standalone, VST3, CLAP) and the standalone smoke-tested successfully.** Gotchas recorded: JUCE CMake format names are mixed-case (`Standalone`); CLAP requires clap-juce-extensions; shape format v0 deferred into M1.
- **2026-09-29** — **M1 vertical slice CODE COMPLETE.** FilterBank (128 TPT peaking bands + per-band one-pole smoothing) wired to StringSim in processBlock (sim ticks every 4 samples; positions → ±18 dB targets); drawable canvas editor with draw/release/flatten; spinlock mailboxes both directions; 5 physics params. GUI-verified end-to-end with automated strokes: deposit renders, released string visibly rings and propagates. Pending: human ear test (the go/no-go). Build artifacts: Standalone/VST3/CLAP all green.
- **2026-09-29** — **M1 GATE PASSED** — project owner ear test in Reaper: "sounds GREAT... very cheap to run so far."
- **2026-09-29** — **Volume management added (M2 scope item pulled forward).** Two new layers: (1) **Steady** — per-band dynamic EQ: envelope follower per band on the bandpass signal (5 ms attack / 150 ms release) squeezes each band's own contribution above a 0.25 linear threshold; costs ~2 ops/band/sample since v1 is already computed. (2) **Leveler** — output stage: slow RMS auto-trim toward −18 dBFS (0.5 s follower, 1 s trim, ±6 dB clamp) + fast peak safety limiter (1 ms attack, −1 dBFS ceiling); stereo control from max(|L|,|R|). Params: Steady 0–1 (default 0.35), Leveler 0–1 (default 0.5). Deployment note: Reaper locks a loaded VST3 — auto-copy (now enabled via COPY_PLUGIN_AFTER_BUILD) fails while the FX is loaded; deploy on next Reaper close + rescan.
- **2026-09-29** — **Editor control strip added.** User couldn't see the new knobs because the custom editor only drew the canvas — the 7 parameters had no visible controls. Added a bottom strip of rotary knobs (Damping, Stiffness, Tension, Smoothing, Drive dB, Steady, Leveler) bound to the raw AudioParameterFloats via begin/setValueNotifyingHost/endChangeGesture (no APVTS yet — noted as possible refactor). Window enlarged to 760×560; plot area reserved above the strip.
- **2026-09-29** — **Reaper crash root-caused and fixed.** Exception 0xc0000094 (integer divide by zero) in Curves.vst3: `resized()` computed `strip.getWidth() / knobs_.size()` while `setSize()` fired it during the constructor — before any knobs existed. Guarded with an early return; standalone repro'd the crash, then verified the fix live (app runs, all knobs render and bind correctly). **Test noise source added** ("Test noise" knob): Paul Kellet pink noise replaces the input when on, so the standalone demos the string with zero input wiring. Lesson on record: repro plugin crashes in the standalone first — same binary, faster loop, and the Windows event log gives the exception code.
- **2026-09-29** — **"Stuck slow waves" fixed via string DC blocker** (user report: low tension + near-zero damping left the whole string slowly modulating even after tension was restored). Root cause: the wave equation is blind to uniform displacement — tension cannot act on a shape's mean, which rings at √stiffness until damped. Fix: mean position and mean velocity removed every tick in both the C++ engine and the Python spec; validated bit-for-bit across 13,184 CSV cells including a new `recover` regression scenario (late |mean(u)| ≤ 1e-6 with tension restored). Flagpole wind force retuned (−4 → −12) because the zero-mean constraint re-centers the sag. Both suites now cover 4 scenarios.
- **2026-09-29** — **M2 CODE COMPLETE.** Physics presets (Free/Flagpole/Gust/Elastic — anchors + force fields, knob presets applied on selection), Precision modes (128/256/512 nodes with sim/bank/tick tuples), Velocity coupling (tension-normalized). All formats rebuilt, VST3 auto-deployed (09:04), 11 knobs verified rendering in the standalone; JUCE standalone's plugin-state persistence confirmed as a side effect (user's knob values survive relaunch). Note: one regression caught by the linker — the M2 processor rewrite had dropped the createEditor definition; restored.
- **2026-09-29** — **Total-silence bug found and fixed (M2 deploy regression).** The M2 restructure left `sim_` null in `prepareToPlay` with creation deferred to `processBlock` — but the null-guard early-returned *before* the rebuild code, so the sim was never created and every block hit the "no sim → clear output" path. Fix: precision rebuild moved above the null check. Lesson on record: when deferring initialization to the audio callback, the guard that protects against the uninitialized state must not precede the code that initializes. VST3 redeployed 09:13.
- **2026-09-29** — **Cloth wind (user-directed redesign of Flagpole wind).** Old wind was a static ramp — no cloth behavior. New: (1) **traveling forcing wave** — two incommensurate sinusoid components (never visibly repeats) whose phase travels from the anchored end toward the free end, with a distance-growing envelope (small injection at the pole); (2) **density taper** (octaves, CFL-clamped) — string is heavy at the anchor and light toward the free tip, so wave speed and acceleration grow toward the tip: the physical whip mechanism (energy flux into lighter mass). Physics decision on record: damping is a rate and stays uniform; only force terms (tension, stiffness, external) divide by node mass. Metrics: pole→tip u-ratio 2.3×, v-ratio 2.4× (tip velocity 8.4 nodes/s — the whip). New `cloth` scenario in both suites; cross-diff 18,304 cells bit-for-bit. Plugin: **Wind** knob (0–1, scales strength, default 0.6); Flagpole preset = cloth wind + taper 2.0 octaves.
- **2026-09-29** — **Wind/Force semantics split (user decision): "Wind should equal how fast waves propagate the length of the string, not magnitude of modulation."** Wind knob now maps to the gust pattern's **phase velocity** (10–500 nodes/s, default ≈192); the new **Force** knob (0–1, default 0.6) is the modulation magnitude. Damping ceiling raised 10 → **40 /s** (more total damping available, per user). Engine API renamed accordingly (`set_flagpole_wind(phase_velocity, wavelength, exponent)`, live-settable `wind_phase_speed`). Suite still cross-validated bit-for-bit (18,304 cells) — the scenario's 192 nodes/s × 128-node wavelength reproduces the old 1.5 Hz forcing exactly. Gotcha on record: the C++ cloth scenario initially passed the old frequency semantics into the renamed API and silently produced near-static wind — caught by the Python↔C++ cross-diff, which exists precisely for this.
- **2026-09-29** — **Force redefined as a flap limiter (user decision): "Force should be an amplitude limiter that controls how far the cloth is allowed to flap. Wind is how fast it flaps."** Rationale: with Force as a *drive amplitude*, its perceived effect was entangled with damping and wind through resonance — the "sticky knobs" report. As a constraint, amplitude is independent of drive and resonance by construction. Engine: soft tanh displacement clamp per tick (`flap_limit`), velocity scaled with the clamp so the constraint **dissipates** energy instead of pumping it; drive amplitude is now an internal constant. Cloth metrics with the limiter at 0.6: tip displacement governed by the ceiling, whip v-ratio improved to **4.2×**. All five scenarios still bit-for-bit across both implementations.
- **2026-09-29** — **Cloth model v3 (user direction): distributed wind, second anchor, reversibility, bigger Force cap.**
  - **Force cap raised**: flap-limit range 0.05–1.0 → **0.05–3.0** (flaps may exceed the display range; the old cap rarely engaged, which is why Force felt ineffective). **Raised again to 0.05–8.0** after feel-testing ("force supplies globally, which is good, but we need to raise that limit quite a bit"). Audio contribution still clamps at ±1.0 × Max gain dB inside the bank, so extreme flaps stay sonically safe; the visual motion is what scales.
  - **Distributed wind**: the single traveling pattern is replaced by **three gust sites** at 25/50/75% of the span with incommensurate phase multipliers (1.0 / φ / 0.755) — "wind doesn't merely initiate a modulation from the start point; it generates along the length."
  - **Second anchor (left)**: Flagpole now pins both ends — the left anchor denotes the end of the physics domain. Envelope became a **tent** (distance to nearest anchor, raised to exponent 3.0): waves run smaller toward the anchor points, biggest mid-span. Both pin markers drawn in the editor.
  - **Direction control** (new 0/1 knob): the wind pattern can run **low→high** as well as high→low (travel coordinate measured from the chosen origin anchor). `cloth_rev` scenario added; reversed run is symmetric, no blowup.
  - Drive amplitude raised 100 → 400 (node-count-scaled) so flaps reach the Force ceiling in normal use — Force is now the audible amplitude control.
  - Validation: six scenarios, **23,424 cells bit-for-bit** Python↔C++ (cloth: ends/mid u-ratio 1.4×, v-ratio 2.0×; reversal symmetric).
  - Queued idea (not implemented): literal node-resolution refinement near the anchors ("higher resolution at the end") via non-uniform node spacing — needs a non-uniform finite-difference grid; revisit if the tent envelope isn't enough.
- **2026-09-29** — **Cloth v4: the termination is a LENGTH, not an anchor (user decision: "the end animation node does not need to be an anchor point... we lose the whip effect").** The left end is free again and whips. New **Cloth length** parameter (0.2–1.0, measured from the pole): the wind envelope ramps 0→max from the pole to the termination node and is zero beyond; the tail beyond the termination is near-rigid (m_inv = 0.02) with 25× damping so the animation genuinely ends there; the density taper compresses into the cloth span (light end at the termination = whip). Engine refactor: `recompute_cloth()` builds envelope/taper/damping-multiplier arrays from (`cloth_length`, exponent, taper); damping is now per-node (`damping_mult`). Verified: length 1.0 → whip restored (pole→tip v-ratio 4.3×); length 0.8 → tail damped to 0.09. Cross-diff 23,424 cells bit-for-bit.
  - Debug lessons on record: (1) the cloth-length boundary was first computed backwards (termination measured from the wrong end → whole string became "tail" at length 1.0); (2) the envelope branch was inverted (cloth got zero wind); (3) `std::max(0.0, NaN)` silently returns 0.0 — a NaN'd sim printed as perfectly zero and hid the bug; the bit-for-bit Python↔C++ diff is what made the divergence undeniable.
- **2026-09-29** — **Wave-fidelity pass (user report: limiter flattened wave peaks; waves smeared in travel).**
  - **Flap limiter redesigned: per-node tanh waveshaper → shape-preserving peak limiter.** One smoothed gain (fast attack 5 ms, slow release 750 ms) scales the whole string so the true simulated peak touches the Force ceiling. Wave shape is untouched — round peaks stay round; the limit "describes the rounded wave peak" instead of capping it flat.
  - **Numerical dispersion fixed: 4th-order Laplacian** in the interior (2nd-order kept at the two edge-adjacent nodes and ends with Neumann ghosts). Short waves no longer lag long waves, so traveling shapes arrive intact and smooth. CFL headroom verified (scenario 0.30, plugin worst case ≈0.10 vs the 4th-order limit of 0.866).
  - **Wind resolution upgraded: 3 → 5 gust sites**, incommensurate temporal multipliers plus per-site spatial scales — finer, richer detail traveling smoothly down the cloth.
  - Result: pole→tip displacement growth improved 1.8× → **4.0×** (waves now genuinely grow toward the whip end), whip v-ratio 3.5×. All six scenarios bit-for-bit (23,424 cells) after the changes landed identically in both implementations.
- **2026-09-29** — **Force dial re-tuned + Direction mirrored (user reports).**
  - **Force dial dead-zone fix:** with a constant drive and a high ceiling, only the bottom ~25% of the dial engaged. Force now scales the wind drive (1400·(0.15+0.85f), node-count-scaled) *and* the flap ceiling rises exponentially (0.05·60^f → 0.05–3.0). Measured response: peak tracks the ceiling monotonically across the entire dial (0.075 → 2.99) with no dead zone. Semantic note on record: Force is now "limiter + proportional drive" rather than limiter alone — the limiter still guarantees the maximum, and the peak was verified to track the ceiling at every dial position.
  - **Direction now mirrors the cloth geometry (user decision): "switching direction... needs to be wind blowing from the other direction, so whips on the highs instead of lows."** `wind_origin` (0/1) moves the anchored pole to the high or low end; envelope, taper, termination, tail damping, and gust sites all rebuild from that origin. Verified: reversed mode whips at the HIGHS with the tail damped at the lows. Pure reverse-playback of the traveling pattern is a separate queued option (advanced menu).
  - Validation: 23,424 cells bit-for-bit across all six scenarios.
- **2026-09-29** — **Draw-flatline root-caused + Flagpole near-pole small waves (user reports + screenshot).**
  - **Drawing flatlined because Force was at 0.1**: the flap-ceiling formula `0.05·60^f` put the ceiling at 0.07 — the shape-preserving limiter then crushed every fresh deposit (and the whole string) to 7% amplitude. Confirmed via GUI repro (deposit preview drew fine; the ceiling flattened it on release). Fix: ceiling range remapped **0.5 → 3.0** (`0.5·6^f`) — the dial bottom stays playable (deposits at ≥50% size), the top still clamps hard. Lesson on record: a limiter whose floor is near zero makes low settings indistinguishable from "broken".
  - **Flagpole first-third flatness**: envelope exponent 3.0 made near-pole forcing ~0 (0.3³ ≈ 0.03). Lowered to **1.5** — small waves now exist at the pole and grow toward the termination (measured: pole_u 0.14 vs tip_u 0.59; whip v-ratio 5.9×, the best yet).
  - **UI directive recorded (user): adding knobs is fine; the control strip will be grouped into colored category boxes, with an advanced-settings split decided later.** Control redesign stays queued until the feature set stabilizes.
  - Validation: 23,424 cells bit-for-bit; VST3/CLAP redeployed 22:49.
- **2026-09-29** — **"Rubberband broken at damping zero, stuck after" root-caused: the flap limiter was scaling the STRING STATE.** With damping near zero, continuous wind drove the sim's unscaled amplitude unboundedly high, so the shape-preserving limiter's gain (limit/peak) collapsed toward zero — a persistent state that then crushed every fresh user deposit to invisibility, and survived damping changes. **Redesign (fabric metaphor made literal): the limiter now starves the WIND instead of scaling the string.** When the peak is at the ceiling, the forcing is scaled toward zero (fast attack 5 ms / slow release 750 ms, applied at use-time to `f_`, never compounding into the stored field); the string state is untouched — waves always true to the sim, deposits never crushed, no stuck gain state. Consequence on record: the ceiling is now *soft* — string momentum can overshoot it (e.g., 0.82 vs 0.6 limit) rather than being flattened, which is the trade the user explicitly chose ("waves stay round and true to the simulation"). Validation: 23,424 cells bit-for-bit after the redesign; standalone relinked and running.
- **2026-09-29** — **Dial calibration from feel-testing (user): "most of the meaningful dampening happens by 45%; most of the meaningful force is applied by 25%."** Dial ranges now map to the *useful* span of each control: **Damping** max 40 → **18 /s** (the old 45% point); **Force** compressed ×0.25 — ceiling 0.5–**0.88** and drive 210–507 across the full dial (the old 25% point is now f=1.0). Calibration philosophy on record: dial ranges must be derived from where the meaningful action actually occurs, verified by feel-testing — extreme values beyond that are advanced-screen material, not dial headroom. Saved states with out-of-range values clamp on load. VST3 redeployed 23:26.
- **2026-09-29** — **Force drive made exponential (follow-up to "force doesn't seem changed"):** the EQ response in dB is logarithmic in drive, so a linear drive dial perceptually compressed at the top. Drive is now `210·2.4143^f` (210–507), giving a linear dB sweep across the dial. Also flagged: the JUCE standalone restores the previous session's knob values — a low Force from earlier testing persists across relaunches and can masquerade as "the fix didn't work." VST3 redeployed 23:32.
- **2026-09-29** — **Canvas amplitude scales + Drive presented as a ±range (user: "the string is hitting hard caps... set different density levels or amplitude levels for the curve to draw on"; "Drive will be... a slider that goes from +-3 all the way to +-24").**
  - The "hard caps" were the canvas display range (±1 mapped to the plot height) — a display limitation masquerading as a physics limit. Added a **Scale button** (top-right, cycles ±0.5 / ±1 / ±2 / ±4, scope-style): the string gets visual headroom at larger scales, and drawing resolution tightens at smaller ones (y->value mapping scales with it). Physics untouched — display-layer only.
  - **Drive** relabeled **"Range ±dB"** (3–24 unchanged — the user's ±3-to-±24 slider is the existing range, now presented as the EQ's boost/cut span). VST3 redeployed 23:36.
- **2026-09-29** — **Force dial v5 (user: "still incredibly strong, doesn't go down far enough... keep most of the scale and make the linear curve of the control lean 65% to the upper half").** Floor dropped 0.5 → **0.15**, top kept at 0.88, and the control curve applies a **0.66 power lean**: 65% of the dial travel covers the upper half of the force range (value midpoint sits at dial 35%). Drive follows the same leaned curve (60–507 effective). Note: the previous build in this sequence briefly shipped the old mapping due to a patch-script mismatch — v5 is the one deployed (23:46).
- **2026-09-29** — **"Rubberband physics still broken" root-caused: Damping was maxed at 40/s (the top of the range added earlier).** At 40/s a drawn shape decays with a ~25 ms time constant — it vanishes within 2–3 video frames, which reads exactly like "drawing flatlines." Confirmed empirically: a GUI-driven stroke at damping 40 was invisible 2 s later, while the identical deposit rings in the validation suite. The wind preset masks this because it continuously re-excites the string; a deposit gets no such energy. Fix applied on the running standalone: damping set to 1.5 via the UI — rubberband behavior is intact (the M1-style ring-and-settle is provably working in the engine). Design note for the advanced screen: max damping is a legit "instant return" setting but should be labeled as such; consider whether 40/s is worth keeping at the top of the dial.
- **2026-09-29** — **WYSIWYG display (user report: "very slow modulation in the EQ that outlives what is happening on screen").** Root cause of the mismatch: the canvas drew the raw string, while the EQ applies smoothed + Steady-modulated gains derived from it — plus two invisible slow modulators (Steady's 150 ms per-band followers; the Leveler's ~1 s output trim). Fix: the filter bank now exposes its **actual smoothed per-band gains**, and the canvas draws **that** as the main curve (normalized to ±1 at Max gain dB) with the sim string as a faint reference underneath. Screen = EQ behavior exactly; the only lag is the Smoothing knob (user-accepted). Steady's ducking is now visible on the curve. Note: the Leveler is output-wide gain, not part of the EQ curve — it can't appear on the display by definition; set it to 0 for a strictly string-only signal path.
- **2026-09-29** — **Resolution pass (user request: smaller waves, especially propagating toward the end).**
  - **Precision modes raised:** Fine 128 → **256** nodes (new default), Ultra 512, Analysis **1024**. Mailbox max raised to 1024. Finer grid = smaller wavelengths resolvable; the 4th-order stencil is what keeps those small waves intact in transit. CPU note: Analysis (1024 bands × 2 channels) is the opt-in heavy mode.
  - **Wind resolution: 5 → 8 gust sites**, positions spread along the full cloth (8–96%), adding high-spatial-frequency sites (k up to 2.6× base) that inject small ripples — the fine detail the finer grid can now carry, especially near the termination where the taper accelerates them.
  - Measured: whip v-ratio improved 3.5× → **5.4×** (fine ripples ride the big wave toward the termination). All six scenarios bit-for-bit (23,424 cells).
  - Force-dial tuning from the previous pass retained (drive + exponential ceiling, monotonic across the dial).
- **2026-09-29** — **Waves knob (user request: "a knob for a higher concentration of waves").** Scales the spatial density of the wind pattern (`wave_density`, 0.5× stretched → 3× packed) multiplicatively across all 8 gust sites — big waves stay big, smaller waves get packed in tighter. Live-settable, no recompute needed (pure forcing-side). Panel is now 16 knobs; a layout/UX redesign of the control strip is queued before release. Validation held: 23,424 cells bit-for-bit.
- **2026-10-01** — **Repo under version control and pushed to GitHub (private).** Git had never been configured on this machine — zero commits existed, so the entire project (design doc, validated engine, working plugin) lived only on local disk. Identity set, initial commit `3da1368` (33 files, 53,782 insertions; `build/` correctly excluded by `.gitignore`), pushed to `github.com/TramEFX/curves` (private, `main` tracking `origin/main`). `gh` CLI installed via winget; auth via browser passkey → OAuth token in the Windows keyring, so pushes need no further interaction. Validation re-run at commit time: **25,216 cells, 0 mismatches, max diff 0** across all six scenarios.
  - **Housekeeping nits on record (not yet actioned):** (1) the cell count **23,424** appears in 8 earlier changelog entries and is stale — the suite has since grown to 25,216 (precision/scenario changes); entries are left as written, treat 25,216 as current. (2) `prototype/cpp_*.csv` are committed and are regenerated by every `scenarios.exe` run, so they show as modified after each validation pass — useful as a regression tripwire, noisy otherwise; `.gitignore`-ing them is a one-line change if the noise outweighs the signal.
- **2026-10-01** — **Aerodynamic drag: the wind force is now closed-loop (`kClothAero`).** Until now the wind was a *prescribed* force field — `f_[i] = wind_strength * wind_env_[i] * gust` — purely a function of position and time, with no node velocity anywhere except uniform viscous damping. The string could be pushed by the wind but could never push back. That open-loop forcing is the root cause of the entire flap-limiter saga (four separate bug/feel reports: tanh flattening peaks, the dead-zone dial, deposits crushed to invisibility, and the latched gain that survived damping changes) — an open-loop drive has no natural amplitude bound, so an artificial one was bolted on and kept having wrong semantics.
  - **Force law** (per node, in `compute_accel` so it sees the Verlet half-step velocity, not the stored one): `v_rel = W_i - v_i`, `F_i = aero_rho_cd * |v_rel| * v_rel / (1 + s_i^2)` where `W` is a transverse wind **velocity** field (nodes/s, same units as `v_`) built from the existing 8 gust sites, and `s_i = du/dx` is the local slope. The `1/(1+s^2)` angle-of-attack factor is what breaks rigid-body motion into travelling waves and nodes: a steeply sloped element lies along the flow and catches less of it.
  - **Added as a NEW force model, not a modification.** `kFlagpoleWind` is untouched, so all six original scenarios stayed **byte-identical** (verified against the previous commit) and the regression tripwire survived a change that makes the system nonlinear. This was deliberate: a linear system cross-validates trivially, a nonlinear one may not.
  - **Measured findings (these corrected an expectation on record):** drag **compresses** amplitude, it does *not* hard-saturate it — peak grows ~`wind_speed^1.4` with no ceiling. The saturation lives in `aero_rho_cd`: 33× more drag only doubles amplitude (asymptote ≈1.25), because the string gets dragged along with the wind and stops moving through it. Two consequences that directly address long-standing knob complaints: **damping is now nearly decoupled** (100× damping change moves peak only 0.84 → 0.50, where previously damping-at-zero let the forcing run away — the "rubberband broken at damping zero" report), and **tension became the dominant factor** (4.7× spread), so it is normalized out.
  - **Calibration** (`wind_speed = 37 * force * (tension/300)^0.6 * (128/n)^0.25`, `aero_rho_cd = 0.6`): dial is **monotonic with no dead zone** — 0.04 at 15% through 0.995 at full, peak landing just under the ±1 clamp so the EQ curve stays round (the WYSIWYG hard anchor). Tension normalization cuts dial spread 4.7× → 1.4×; the node term holds peak within 0.73–0.995 across all precision modes (128/256/512/1024). **Tick-rate invariant** (0.964 / 0.962 / 0.963 at 1/4/12 kHz), so the 1 kHz scenario calibration transfers exactly to the plugin's 12–24 kHz, and the stability cap never engages.
  - **Stability guard:** explicit integration of quadratic drag is stable only while the drag's effective damping rate stays under 1/dt, so `|v_rel|` is capped at `0.5/(aero_rho_cd * m_inv_max * dt)`, recomputed per tick because both inputs are live. Physically a stall cap. Measured headroom: worst observed `|v_rel|` 73 vs a cap of 250 — it is a true safety net, never an operating point.
  - **Flap limiter retired from the plugin path** (`flap_limit = 1e9`). Kept in the engine so `kFlagpoleWind` and the original scenarios keep exact behavior.
  - **Bug fixed: the v5 Force calibration was dead code.** `PluginProcessor.cpp` assigned `flap_limit` twice — the v5 mapping on one line and the superseded `0.05 + 7.95*force` on the next, which silently won. The changelog entry claiming "v5 is the one deployed (23:46)" was wrong; the old mapping was running, which is why Force still felt wrong after that pass. Both assignments are now gone.
  - **Wind knob made precision-independent.** `wind_phase_speed` is in nodes/s, so at a fixed setting the gust pattern traversed a 1024-node string 8× slower than a 128-node one — "Wind" meant different things in different precision modes. Now scaled by `n/128` (and the same scaling is baked into the Flagpole preset's `set_cloth_aero` call), which also removed a resonance artefact that made node-count calibration non-monotonic.
  - **Known interaction, not fixed:** Waves at minimum (stretched waves) + Force at maximum reaches peak ≈1.71 and therefore clamps and flat-tops. Stretched waves push more coherently along the string. Normalizing by `wave_density` was tested and rejected — it fixes the stretched case but creates overshoot at high density (three of four test points over 1.0 instead of one). The principled bound is a geometric-stiffening / Duffing restoring term (real cloth stiffens as it stretches, which is why a flag does not flap infinitely far); that is a new physics term and is **queued for a decision**, not assumed.
  - **Validation: seven scenarios, 30,336 cells, 0 mismatches, max diff 0** — including the nonlinear `aero` scenario, where the predicted libm-divergence risk did not materialize at a 4 s run length. **Tolerance policy on record:** the six linear scenarios must stay zero-diff forever (tripwire); for nonlinear scenarios, expect eventual legitimate divergence at longer run lengths and compare short windows at 1e-6 plus long-run statistics, rather than treating a late per-cell mismatch as a bug. Any future stochastic gusting must use an exactly-specified integer PRNG implemented identically in both languages — `np.random` and `std::mt19937` will not agree.
  - Aero scenario metrics: whip intact (pole→tip u-ratio 3.6×, v-ratio 4.2×), late peak 0.723, finite throughout. All formats rebuilt, VST3 auto-deployed, standalone smoke-tested clean (no fault events).
- **2026-10-01** — **Rocking sway root-caused: it is the TILT mode, not the Leveler.** User report: after the string is excited, and also while it is moving, a slow sway from low to high continues indefinitely — suspected the Leveler.
  - **The Leveler was ruled out by inspection before testing:** it is an output-wide gain, so it moves all frequencies together and cannot produce a spectral tilt. Its RMS follower also reads the *pre-trim* signal, so it is feedforward with no loop to oscillate in.
  - **Actual cause — mode 1 has the same pathology the DC blocker was built to fix.** Measured on an asymmetric deposit: the tilt component starts at **38x the amplitude of all other modes combined**, `|mean(u)|` stays a perfect 0.000000 (the DC blocker works), every other mode is gone in 2–3 s, and the tilt is still ringing past 10 s. Frequency measured 0.61–0.87 Hz and only weakly dependent on stiffness (restored by the `stiffness` spring *plus* the two Neumann end nodes, not by tension) — which is why the Stiffness knob never tamed it. At low damping it is still visible at 14 s; at very high damping it becomes overdamped and creeps back slowly instead, so neither end of the dial escapes it.
  - **Under wind it was consuming most of the usable range:** sustained tilt displacement mean 0.246 / max **0.947** of the ±1 span — the cloth's fine detail was riding on a giant see-saw. Removing it dropped overall string peak from max 1.831 → 1.414, i.e. materially less clamping, which directly serves the WYSIWYG anchor.
  - **Fix: tilt blocker** — remove the least-squares linear trend from position and velocity every tick, in the centered basis (orthogonal to the mean, so it composes cleanly with the DC removal). Result: tilt displacement **0.000000**, late peak 0.096 — only genuine waves remain. Under wind, 0.947 → 0.000.
  - **Side effect: the Waves/Force overshoot flagged in the previous entry is resolved.** Waves-at-minimum + Force-at-max went 1.708 → 0.774. The geometric-stiffening / Duffing term is therefore **no longer needed** and comes off the queue.
  - **Force recalibrated: 37 → 25.** The previous calibration measured peak over a 3.5 s window, but the gusts are quasi-periodic and keep finding higher peaks — the 30 s peak at base 37 was **1.41**, not the 0.995 that was recorded. Lesson on record: calibrate amplitude against a long observation window, never a short one. At base 25 every precision mode and knob extreme tested stays under the ±1 clamp (n=256 default 0.83).
- **2026-10-01** — **Leveler reassessed (user: the knob doesn't seem to do much of anything). Three real defects found.**
  - **The target level was wrong by 6 dB.** `const double target = 0.0625; // (-18 dBFS)^2` — but 0.0625 is 0.25², and 0.25 is **−12 dBFS**. The mean square of −18 dBFS is 0.0158489. Measured consequence: fed a signal at exactly −18 dBFS, the stage applied **+5.9 dB** instead of nothing. It had been levelling 6 dB hotter than both the code comment and the design notes claimed, which also made the limiter engage sooner than intended. Fixed; verified −18 in → 0.03 dB applied, and −24 / −12 both now land on −18.
  - **Range too narrow to matter: ±6 dB → ±12 dB.** With Range at ±18 dB the EQ alone swings the output further than 6 dB, so the clamp pinned and the depth knob had almost nothing left to scale. At amount 0.5 it now applies exactly half the dB correction (measured 3.00 of 6.01).
  - **`std::pow` was being called per sample per channel pair.** `g` tracks the live RMS so it cannot be hoisted, but it is now `exp(amount*log(g))` with a short-circuit at amount = 1.
  - **Documentation corrected: `amount = 0` is NOT a bypass.** The peak limiter is unconditional — a speaker guard, not part of the levelling effect — so at 0 the path is still peak-limited if it runs hot (measured: a −3 dBFS sine reads −3.73 out at amount 0). The earlier note claiming amount 0 gives a strictly string-only signal path was wrong. A true bypass would be a separate decision.
  - Note: the Leveler's ~1 s global trim can still add a slow *level* pump on varying material. That is distinct from the spectral sway above and is inherent to any auto-trim; the remedy if it bothers is a shorter follower or amount = 0.
  - **Validation: eight scenarios, 35,456 cells, 0 mismatches, max diff 0.** New `tilt` scenario exercises the blocker; all seven pre-existing baselines **byte-identical**, having been explicitly opted out of the blocker so they remain the frozen regression tripwire while the engine default stays correct.
  - **Deploy note:** Reaper was open holding the VST3, so the auto-copy to Common Files failed with "Permission denied" and aborted the build before the Standalone target. CLAP and VST3 built locally; Standalone rebuilt separately and smoke-tested clean. **The deployed VST3 in Common Files is still the previous build — close Reaper and rebuild to deploy.** CLAP is not auto-deployed at all (no copy step configured); it exists only under `build/Curves_artefacts/Release/CLAP/`.
- **2026-10-01** — **Subscriber shortlist recorded (user: stereo placement, and more broadly "things that are spectrums").** §3.3 now carries the selection criterion — a subscriber must *itself be a spectrum*, one meaningful independent value per frequency — plus a ranked candidate table and the topology consequence that splits the stereo pair: **width** fits the current `dry + Σ peaking bands` topology (it is a ratio, so M/S with a bank instance on each needs no isolation, ~2× bank cost), while **pan** cannot be a hard pan while a dry path carries the full-bandwidth signal — a true per-band pan wants a complementary/partitioning bank, which interacts with the parallel-path Open Decision. Watch item logged: the ear is far more sensitive to time-varying inter-channel differences than to gain wobble, so any stereo subscriber needs smoothing at least as strict as gain plus a mono-compatibility check.
- **2026-10-01** — **Step 2 — vortex shedding ("Flutter"): the string is now ALIVE rather than driven.** A van der Pol **wake oscillator per node**, after Facchinetti-style vortex-induced-vibration models: `q'' = -eps*w_i*(q^2-1)*q' - w_i^2*q + coupling*v_i`, with `F_i = shed_gain * q_i * wind_env_i`. The `(q^2-1)` term is negative damping below `|q|=1` and positive above, so `q` is a genuine **limit cycle**.
  - **The acceptance test was motion with the gust wind switched off entirely**, because anything that needs gusting is just more forcing. Measured: self-sustained flapping at `wind_speed = 0`, peak 0.23–0.35 depending on amount, finite and stable across a 4× gain sweep. This is also what finally lets the Damping knob behave on a *deposit* — previously only the wind re-excited the string, so a drawn shape had no energy source and damping read as "drawing flatlines".
  - **Vortex cells, not one global breath.** `w_i` carries a gradient along the cloth (`shed_spread`, thinner/lighter elements shed faster), which detunes neighbours. Measured wake coherence **0.02–0.03** (0 = fully decorrelated cells, 1 = everything in phase). Without the gradient the whole string sheds together and reads as a single LFO.
  - **Lock-in** comes from the `coupling*v_i` term: structure velocity pulls the wake's phase, so shedding near a structural resonance synchronizes to it.
  - **Rate is an audible modulation rate, so it is set in Hz and tuned by ear**, never derived from a fictional Reynolds number (the x-axis is log-frequency, not space — there is no real Strouhal number here). Measured monotonic control: nominal 0.7/1.5/3/6/12 Hz → dominant band-modulation 0.50/1.06/2.06/4.19/8.38 Hz. **The observed/nominal ratio is configuration-dependent** (≈0.70 at n=256 with an FFT-dominant metric, ≈1.30 at n=128 with a zero-crossing metric), so NO calibration fudge was applied — `shed_freq_hz` is documented as the nominal wake frequency and the per-band rate varies around it by the gradient, which is exactly what makes it read as cloth rather than an LFO. Faking a single Hz number would have been dishonest.
  - **Useful range is ~0.3–4 Hz, and the rate knob is implicitly also a depth knob:** the string's mass and tension lowpass the forcing, so peak falls from 0.35 at 3 Hz to 0.03 at 12 Hz. Left physical rather than gain-compensated; the knob tops out at 8 Hz.
  - **It makes the amplitude situation BETTER, not worse.** Combined with gust wind at Force = 1.0, raising shed gain 0 → 120 *lowered* peak 0.826 → 0.722 while raising the mean 0.176 → 0.233: the wake is incoherent with the gusts, so it trades occasional big coherent lurches for continuous motion. No new clamping risk against the WYSIWYG anchor.
  - **Debug lesson on record:** the first prototype produced exactly zero string motion despite the wake oscillator visibly running at its limit cycle (`q` = 1.27). Two causes, both instructive. (1) In `kClothAero` the wind update *owns* `f_` and zeroes it, so an externally injected shedding force was wiped every tick — shedding has to be computed inside the wind update. (2) `q = qd = 0` is an **unstable fixed point** of the van der Pol oscillator: with both exactly zero and the string at rest it never departs. It needs a deterministic nonzero seed (`kShedSeed = 0.01`); the working prototype had one and the spec port did not. The seed is now preserved across cloth-geometry changes so moving Length or Direction does not stall the flutter mid-flight.
  - **Plugin: two new parameters, 16 → 18.** `Flutter` (0–1 → shed_gain 0–120, **default 0** so existing feel is unchanged until dialled in) and `Flutter Hz` (0.3–8.0, default 3.0). Also fixed a stale comment: `force_` was still documented as "flap ceiling" when it has been wind speed since the aero pass.
  - **Validation: nine scenarios, 43,136 cells, 0 mismatches, max diff 0.** New `shed` scenario asserts self-sustaining behaviour with zero gust wind. All eight pre-existing baselines **byte-identical** — `shed_gain` defaults to 0 and the stage is skipped entirely at 0, so it costs nothing and perturbs nothing.
  - All formats rebuilt, VST3 deployed and verified byte-identical to the build, standalone smoke-tested clean.
- **2026-10-01** — **Wave travel direction fixed (user: "the oscillation applied to the end of the string is propagating waves inwards towards the anchor point — counterintuitive to wind blowing from the anchor point").** The user kept the string physics and asked only that waves propagate *away* from the anchor, with the wind.
  - **Measurement discipline first — and it mattered twice.** The initial check used `argmax(|cross-correlation|)` between two probes, which lets ANTI-correlation masquerade as a delay; it reported the gusts as backwards and the new flutter as correct, i.e. exactly inverted. A 2-D spatio-temporal FFT metric was then built and **validated against synthetic waves of known direction** before being trusted (outward 1.000 / inward 0.000 / standing 0.500 — and the first attempt had the k-sign convention backwards, which the synthetic check caught). Verdict with the validated metric: **gusts 0.621 outward (correct), flutter 0.203 INWARD** — the stage added in the previous commit was the culprit. Lesson on record: validate a directional metric on a known signal before believing it about your own system.
  - **Root cause: a FREQUENCY gradient cannot set a travel direction you want.** The wake bank was decorrelated by making thinner elements shed faster, but a frequency gradient produces a phase ramp that *steepens linearly with time* (`dphi/dp = (dw/dp)*t`), so the direction is locked by the sign of `dw/dp` and the apparent wavelength shrinks without bound. Convective (one-sided, upstream) coupling was tried and **could not beat it** — 0.203 → 0.218 across a 0–8 sweep — because the ramp keeps steepening. Removing the gradient and relying on convective coupling alone worked but was fragile: weak coupling fully synchronized the bank (phase lag ~0, no travel, coherence 0.999), strong coupling spatially aliased and flipped the direction back (outward 0.662 at convect 25, 0.190 at 40), and it also dragged the collective frequency off the knob (3.0 → 4.5 Hz).
  - **Fix — exploit that van der Pol is NEUTRAL IN PHASE.** The oscillators are autonomous and identical, so an amplitude attractor with no phase preference keeps whatever phase relationship it starts with. The bank is therefore *seeded* with an outward travelling ramp, `q_i(t) = r*cos(w*t - 2*pi*p_i*waves_per_cloth/span)` with `p_i` the distance from the pole. Direction is then correct **by construction**, the wavelength is explicit rather than an emergent function of a coupling gain, there is no stiffness or stability concern, and the collective frequency is no longer pulled off the knob. Verified the ramp **survives a 40 s run** (phases persist as predicted).
  - **Results:** travel direction is now positive (outward) in **every** configuration tested — flutter alone, gusts alone, all four precision modes, cloth length 0.6, and with Direction flipped (the ramp re-derives from the active pole, so flipping the knob flips the travel). Outward fraction 0.70–0.88 at full cloth length, versus 0.203 inward before. Wake coherence 0.21 — ordered rather than random, but still far from 1.0, so the vortex cells survive without the gradient.
  - **Bonus: the Flutter Hz label became honest.** With the gradient gone the measured probe modulation is **2.992 Hz against a nominal 3.0** (it read 3.898 at the same settings before). The previous entry's note that the observed/nominal ratio was configuration-dependent no longer applies to the shipping design.
  - **Wavelength: 1.5 per cloth span.** Measured across lengths 1.0/0.8/0.6/0.4, no single value wins everywhere — short cloths are reflection-limited because the near-rigid tail is a reflecting boundary, which is physics, not a bug. 1.0 is best at full length (0.751) but drops to 0.381 at 0.6; **1.5 is the most consistent** (0.53–0.70 across all lengths).
  - **Re-phasing preserves amplitude.** `seed_wake_phase(preserve_amplitude)` keeps each oscillator's current magnitude and only rotates its phase, so dragging Length or Direction re-aims the travel without restarting the flutter from silence.
  - **Honest limitation on record — Force and Flutter now SHARE the amplitude budget, and it does not fully fit.** The travelling ramp makes the wake coherent, so it *adds* to the gusts where the old incoherent version partly cancelled them (which is why the previous entry could report that flutter *lowered* the peak). Measured long-run peak at Force 1.0, n = 256/512/1024: gain 0 → 0.826/1.011/0.906; gain 5 → 0.841/0.931/0.943; gain 8 → 0.853/0.971/0.992; gain 30 → 1.104 (n=256), 1.585 (n=512). Only ~5 would keep every mode under the clamp, which would make Flutter too weak to be worth having, so **the range is kept useful at 30 and both knobs near maximum will reach the ±1 clamp and flat-top.** That stays honest against the WYSIWYG anchor (the display draws the clamped curve, so it is visible) and matches the standing decision to favour waves true to the simulation over an enforced ceiling. Flutter defaults to 0, so nothing clamps unless both are dialled up. **Also noted: n = 512 reads 1.011 at Flutter 0, so the Force base-25 calibration is itself marginal in that one precision mode** — worth a look if 512 is a mode the project cares about.
  - **Validation: nine scenarios, 43,136 cells, 0 mismatches, max diff 0.** The `shed` scenario now also asserts travel direction, using the wave momentum flux `-sum(u_t * u_p)` accumulated online — `u_t` IS the velocity array, so no time differencing is needed, and the metric is cheap, deterministic and identical in both languages (validated +1.000 / -1.000 / 0.000 on synthetic waves). `shed` reports **+0.372188** in both engines. Only the `shed` baseline changed (its physics did); the other eight are byte-identical.
  - All formats rebuilt, VST3 deployed and verified byte-identical, standalone smoke-tested clean.
- **2026-10-02** — **High-precision dropouts root-caused and fixed: the audio thread was over 100% of ONE core.** User report: lags, skips and artifacts in high precision, only with the wind running, at ~6.5% total CPU — "I should have enough headroom".
  - **The 6.5% was the misleading part.** The machine is a Ryzen 7 5700X3D: 8 cores, **16 logical processors**. 6.5% of 16 threads is **1.04 of a single core**, and an audio callback runs on one thread. The headroom was real but unreachable — total-CPU readings are meaningless for real-time audio, only per-thread cost matters.
  - **Measured rather than guessed:** a new `bench` target (`src/bench_main.cpp`, CMake target `bench`) times each stage and reports it as a **percentage of one core at 48 kHz**. Before the fix, at 512 nodes: sim tick wind OFF 13.9%, wind ON **86.1%**, wind+flutter 89.9%, bank 11.5% — **total 101.4%, i.e. over budget**, which is exactly a guaranteed dropout. Wind-off totalled 25%, which is why it only glitched with the wind.
  - **Cause: the gust field evaluated 8 `sin()` per node per tick.** At 512 nodes and a 24 kHz tick that is 4,096 sins per tick, **98 million per second**, measured at 7.35 ns each (~22 cycles) — the wind alone cost 6.2× the entire rest of the sim.
  - **Fix: tabulate the time-invariant spatial part.** The sum is `sum_s sin(psi_s(t) - k_s*p_i)`; expanding `sin(a-b)` splits it into a per-tick scalar part and a per-node spatial part that never changes with time. `cos(k_s*p_i)` and `sin(k_s*p_i)` are precomputed into tables laid out `[node*8 + site]` (one node's eight sites contiguous — 16 separate arrays would thrash the cache). Per-tick cost becomes **8 sin + 8 cos TOTAL, independent of node count**, plus multiply-adds per node. Tables rebuild only when geometry or `wave_density` changes, never per tick.
  - **Result at 512 nodes: wind-on 86.1% → 25.9% of a core; total with flutter and bank 101.4% → 43.0%.** Full table after the fix (sim with wind+flutter / bank / total): 128 → 3.8/3.1/**6.9%**; 256 → 15.9/5.9/**21.7%**; 512 → 30.8/12.2/**43.0%**; 1024 → 57.2/22.2/**79.4%**.
  - **Numerically free: all nine CSV baselines came out BYTE-IDENTICAL.** The trig-identity rewrite is indistinguishable at 6 decimals, so the whole frozen regression set survived a change to the hottest code path. Cross-validation still 43,136 cells, 0 mismatches.
  - **Second, independent glitch source fixed: priority inversion on the display mailbox.** `publish_display()` took a **blocking** `SpinLock` that the editor also holds while copying the snapshot out. A spinlock owned by a lower-priority GUI thread means the audio thread burns its deadline spinning — intermittent dropouts uncorrelated with load. Now `tryEnter()` and bail: dropping a display frame is invisible at 30 Hz, a dropout is not.
  - **Display publishing throttled to ~60 Hz.** It ran every block (187/s at 256 samples) while the editor repaints at 30 Hz — ~6× the lock traffic and 6× the per-node `log10` for no visible gain.
  - **`pow` → `exp2` in the gain mapping.** `10^(db/20) == 2^(db/6.0206)`, once per node per block (512 calls at high precision). Minor but free.
  - **Doc/code mismatch on record:** an earlier entry claims "Precision modes raised: Fine 256, Ultra 512, Analysis 1024", but `PluginProcessor.h` ships `kPrecisionNodes = {128, 256, 512}` with divisors `{4, 2, 2}`. **The code is the truth; that entry overstates.** With the wind now 3.3× cheaper, raising to 256/512/1024 is newly plausible — 1024 measures 79.4% of a core, which is viable but leaves little margin for the host and other plugins, so 512 at 43% remains the sensible top. Left as a decision, not changed unilaterally.
  - Lesson on record: for real-time audio, measure **per-thread cost as a fraction of the deadline**. A total-CPU percentage on a many-core machine will hide a saturated audio thread completely.
  - All formats rebuilt, VST3 deployed and verified byte-identical, standalone smoke-tested clean.
- **2026-10-02** — **Sample rate doubles the cost; `bench` now takes an fs argument.** The sim ticks at `fs/divisor` and the bank runs per sample, so BOTH scale linearly with sample rate. Measured at n=512: **48 kHz → 42.8% of one core** (sim+flutter 30.9 + bank 11.9), **96 kHz → 85.4%** (62.1 + 23.3). So high precision at 96 kHz is back at the edge of the budget even after the gust-table optimization, and the glitch report would reproduce there. Guidance on record: treat the headroom table as per-sample-rate, and prefer 256 nodes at 96 kHz (which lands near 43%, the same place 512 sits at 48 kHz).
  - **Host threading note (user question):** a Reaper FX chain is **serial within a track** — plugin outputs feed the next plugin's input — so two Curves instances on the same track ADD on one thread (43% + 43% = 86%) and will glitch, while instances on **different tracks** are distributed across worker threads and run genuinely in parallel. Take/item FX belong to their track's chain and serialize with it. Two consequences worth remembering: (1) the 16 logical processors are 8 physical with SMT, and SMT buys perhaps 1.2–1.3× on FP-heavy DSP rather than 2×, so effective capacity is ~10 core-equivalents, not 16; (2) Reaper's anticipative FX processing gives a track's chain far more slack than the per-buffer deadline, but it is bypassed on record-armed / input-monitored tracks — which is exactly when a borderline plugin starts glitching.
- **2026-10-02** — **Tip "Wrinkle" control added, and both Flutter ranges widened (user request).** "Adjust the wrinkle density of the end of the wind — how the last third resolves or flaps is where we are deriving much of the cool musical quality. The beginning is good because of Flutter."
  - **Two cheaper designs failed first, and the reason is quantitative.** (1) Warping the gust coordinate so the wavenumber grows toward the tip (`dPsi/dp = 1 + wrinkle*(p/span)^3`) just made the end *quieter*: tip/pole density ratio moved only 1.11 → 1.33 while peak collapsed 1.008 → 0.183. (2) Extra high-wavenumber components travelling at the WIND's phase velocity did almost nothing either. **Cause:** at these wavenumbers the tension restoring term `c^2*k^2` is ~1944 against a stiffness of ~2, so short waves are suppressed roughly 1000x. Off-resonance forcing cannot excite them at any sane amplitude.
  - **What works: tip-RESONANT components.** Four extra gust components with high spatial wavenumbers `{12, 17, 23, 31}`, windowed by a `d^4` envelope so they live in the last third, and — the actual trick — with their **phase velocity locked to the local wave speed at the tip**, `c*sqrt(2^taper)`. That puts them on the short-wave resonance, whose width is only a few percent, so "approximately right" was never going to work (the earlier attempt was 27% off at 764 vs 600 nodes/s). Measured tip/pole detail ratio **1.11 (off) → 1.45 at wrinkle 1.5 → 1.82 at 3.0, with the pole end unchanged** (0.83 → 0.87) — the effect stays where it was asked for. The scenario at 512 nodes reports a **4.27x** tip/pole ratio.
  - **Measured: the wavenumber set matters and FINER is worse.** `{12,17,23,31}` gives ratio 1.68; `{20,28,38,50}` gives 1.56; `{32,45,60,80}` gives 1.35 — past a point the modes are too stiff even on resonance.
  - **Free at runtime.** Because the gust spatial part is already tabulated (the dropout fix), the `d^4` envelope and the wrinkle wavenumbers bake straight into tables. Per-tick cost is 8 extra scalar transcendentals plus 4 multiply-adds per node; measured sim cost at n=512 is unchanged within noise (27.5% of a core with wind on).
  - **Anti-alias clamp added.** The finest wrinkle is held to at least 8 nodes per cycle, scaling the set together. Without it, at 128 nodes the wavelengths land at 10.7–4.1 nodes — at or past what the grid carries — and the tip measured **less** detailed than the pole (ratio 0.85, and 0.71 after clamping coarsened it below the base detail). **Usage note: Wrinkle is a high-precision control.** Effective at 512, marginal at 256 (finest wrinkle ~8 nodes), not useful at 128. The `wrinkle` scenario therefore runs at 512 nodes, not the harness default.
  - **Flutter ranges widened as asked.** Gain 30 → **90** (measured peak at Force 1.0: 1.585 / 1.977 / 2.233 at gain 30 / 60 / 90; alone: 0.585 / 0.876 / 1.096), and rate 0.3–8.0 → **0.1–20 Hz**.
  - **Honest warning on the wider rate range, left uncompensated:** amplitude swings ~**50x** across it. It peaks near **0.45 Hz at 4.35x** the amplitude of 3 Hz, because the shedding there resonantly drives the string's fundamental (`omega = c*2*pi/span` lands about there), and **damping barely touches it** (4.37 at damping 0.5 vs 3.48 at damping 12 — so it is not a damping problem). Above ~12 Hz the string's own mass and tension lowpass it away (0.08 at 12 Hz, 0.03 at 20 Hz). Not gain-compensated, because the project prefers true-to-simulation behaviour and the big slow flapping may well be wanted — but compensating the rate is a one-line change if the dial starts behaving like a volume knob.
  - **Debug lesson on record: the bit-exactness tripwire fired on a REPORTING bug, not a physics one.** The wrinkle scenario's C++ and Python reports disagreed (peak 0.4224 vs 0.4401) while the CSV cross-diff showed 0 mismatches — the sims were identical. Two causes, both in the metric: the C++ loop ran `i = 1..n-2` so its peak excluded the free whip end (usually the largest node), and the accumulated density sums differ by summation order (`np.sum` is pairwise, the C++ loop sequential). Fixed the peak range; the density figure is a diagnostic, not a bit-exact assertion. Corollary worth keeping: **trust the CSV cross-diff, not a printed summary statistic.** Also fixed a genuine parity hazard found while looking: `d**4` in NumPy and `d*d*d*d` in C++ round differently, so both now use the identical `d2*d2` grouping.
  - **Plugin: 19 parameters** (`Wrinkle` added, default 0 so the stage costs nothing and changes nothing until dialled in).
  - **Validation: ten scenarios, 73,856 cells, 0 mismatches, max diff 0.** All nine pre-existing baselines byte-identical. All formats rebuilt, VST3 deployed and verified byte-identical, standalone smoke-tested clean.
- **2026-10-02** — **UI reworked: grouped control panels, five colour themes, glow rendering (user request).** "Fully rework the visuals... a bit more flashy... more neon aqua-electric blue... five themes of colour... all the knobs are getting a bit excessive, group them into categories, global knobs in one obvious spot and the knobs that affect each physics setting grouped together."
  - **Controls grouped by SCOPE, not listed in one strip.** Three panels: **GLOBAL** (Physics, Precision, Range dB, Smoothing, Steady, Leveler, Test Noise) on its own full-width row — the "one obvious spot"; **STRING** (Damping, Stiffness, Tension, Velocity) — the physics that always applies; and **CLOTH & WIND** (Force, Wind, Waves, Wrinkle, Direction, Length, Flutter, Flutter Hz) — Flagpole-only, and it **dims and disables itself outside Flagpole**, which is the real answer to "19 knobs is excessive": eight of them are inert most of the time and now look it.
  - **Five themes** (AQUA default — neon aqua / electric blue — then MAGENTA, AMBER, LIME, VIOLET), cycled by a header button. Each is a full 15-colour palette covering background, plot gradient, panels, grid, curve, glow, fill, the sim reference and text. The choice rides in the state tree (`uiTheme`), so a session reopens looking the way it was left; it is deliberately NOT an automatable parameter since it is pure appearance.
  - **Flashier rendering:** vertical plot gradient, rounded panels with accent bars, labelled log-frequency grid (50/100/500/1k/5k/10k), a gradient area fill under the curve, a **four-pass curve** (two glow passes, a saturated core, a white hairline), a glowing anchor pin, and a custom rotary LookAndFeel drawing a glowing value arc instead of the stock JUCE knob.
  - **The hard anchor is untouched and re-documented in the editor header:** the bright curve still comes from `FilterBank::gains()` with the same ±1 clamp the audio path uses, the sim is still only the faint reference line, and the deposit preview is drawn in a contrasting accent so it can never be mistaken for the applied EQ.
  - **Bug found and fixed by screenshotting the result rather than trusting the build — and it was invisible by construction.** `CurvesTheme` has 15 colour fields; every theme literal supplied **14**. C++ aggregate initialization silently shifted every field after the omitted one and **zero-initialized the last**, and a zero `juce::Colour` is **fully transparent, not black**. The symptom was that every piece of text drawn in `text_dim` — all 19 knob labels, all three panel titles, the hint line and the frequency marks — rendered invisibly while the geometry was correct the whole time. It compiled without a warning. Fixed by writing all 15 fields explicitly with a field-order comment, plus a `jassert` in `curves_theme()` that the tail colours are non-zero so the same mistake announces itself next time. Verified all five literals now carry exactly 15 fields.
  - **Method note worth keeping: a visual change needs looking at.** The build was clean and the layout maths was right, yet the UI was substantially broken. Screenshots were captured with `PrintWindow(hwnd, dc, PW_RENDERFULLCONTENT)` rather than a screen grab, because a screen grab caught whatever window happened to be in front; `PrintWindow` captures the window's own surface regardless of occlusion. The theme cycling was verified by synthesising a click on the header button and re-capturing (AQUA and MAGENTA both confirmed).
  - **Window 760x560 → 1000x700** to fit the three panels without crowding.
  - Engine, spec and all ten CSV baselines **untouched** — verified by diff, this was a pure UI change. All formats rebuilt, VST3 deployed and verified byte-identical.
- **2026-10-02** — **Documentation restructured ahead of a context compaction.** `docs/SPEC-0.0.1.md` (the first spec) is superseded and kept; `docs/SPEC-0.0.2.md` is the current exhaustive knob → parameter → function reference, audited against source at `08416ab`; `docs/SESSION-LOG.md` is new and backfills the whole first session — twenty commits from `3da1368` to `08416ab` — with the measurements worth not re-deriving (cost table, Motion budget constants, Flutter Hz response table, Waves centroid table, Velocity scale), the root causes behind each reported bug, the three hypotheses that measurement rejected, the Windows UI-automation lessons, and twelve open threads. Purpose is to survive context loss: anything expensive to measure is written down once.
- **2026-10-02** — **Gain path reworked.** Smoothing moved off the band's linear gain onto the string's control signal at the sim tick rate, so Range dB no longer sits inside a recursive filter: the drawn curve is now exactly Range-invariant (0.298 → 4.95e-14 of full scale across a 16× change) and boosts/cuts are symmetric in dB (2.21× → 1.00×). The bank holds no time state and walks each block target geometrically. Fixed the gain advancing once per CHANNEL (stereo smoothed at half the set time). Steady's follower made mono per the new channel-coherence rule, and `gains()` is finally Steady-inclusive — up to 4.52 dB of real gain reduction had been hidden from the curve, a live hard-anchor violation. Canvas gained a labelled dB axis driven by the effective Range. Cost measured neutral (control smoother +1.57% of a core at 512 nodes, bank unchanged); an earlier 33% reduction projected from flop counting was wrong. New `gain_test` target guards all three properties. Validation unchanged: 73,856 cells, 0 mismatches.
- **2026-10-02** — **Tension amplitude compensation made per-precision.** The flat `(c/300)^0.6` on wind speed under-corrected below 512 nodes, so raising Tension audibly squashed the string (peak 300→600: 0.56× at 128 nodes, 0.61× at 256). The exponent is derivable — drag balancing tension gives `u ∝ W²/c²`, so `W ∝ c` — but peak vs Tension is a **hump** centred on a fixed `f0 = c/span`, so it must be per-precision: `{0.85, 0.80, 0.60}`. 512 unchanged (0.94×, not broken). Whole-dial spread at 128 nodes 3.15× → 1.46×; the Motion calibration at Tension 300 is untouched since `(c/300)^p = 1` there. New self-validating `tension_test` harness. Flagged for later: the Motion budget under-delivers ~37% at the default operating point (likely stale since the Waves range extension), and the shedding path has no Tension compensation at all (80.7× across the dial at Character 1).
- **2026-10-02** — **Wind re-ranged, Smoothing default restored.** Wind read as doing nothing because it was mis-ranged, not inert: what the eye sees depends on whether the string can FOLLOW the gust, i.e. on f_gust/f0, and the old mapping held f_gust constant across precision while f0 = c/span falls as 1/N — so the default sat at ratio 0.64 at 128 nodes but 2.54 at 512, with the dial top at 6.67 where the string cannot respond. Wind is now a multiple of the wave speed (0.12·25^Wind, spanning 0.12–3.0 of the fundamental), precision- and Tension-independent by construction, mirroring the f0 normalization Flutter Hz already uses. Measured peak across the dial at 512: 0.21/0.45/0.28/0.24/0.18 → 0.14/0.40/0.44/0.51/0.32. Damping ruled out as a lever by measurement (10× damping → 4% frequency change). Smoothing default 30 → 15 ms, because fixing the once-per-channel double-step doubled the real smoothing — measured sim-vs-drawn divergence 0.019/0.050/0.119 at 2/10/30 ms, confirming the display path is exact at the minimum and that the lag is the only thing separating the two lines.
- **2026-10-02** — **Mid-band ripples added, folded into Wind.** Measured spatial spectra showed a structural hole: gust sites cover 1.7–6 waves across the cloth, wrinkle 25–64, nothing between, and the 1–2 band carries 5–15× everything above it. Tuning cannot fix it (raising Waves shrinks every band rather than moving energy up) and wrinkle is not targeted (lifts every band proportionally, 1–2 included, by 2.9×). Four new components locked to the MID-cloth wave speed c·2^(taper/4) with a raised-cosine bump envelope centred where that lock is resonant — the same resonance requirement wrinkle hit, since c²k² dwarfs stiffness at these wavenumbers. No new knob (owner's choice): Wind sets the SIZE 6→20 waves, amount follows Motion. Shipped at 1.5: 5.2× the mid-band content for 1.06× peak, against wrinkle's 1.57×. Cost +4.05% of a core at 512 nodes. Mirrored in Python with a new `mid` scenario exercising the path in both engines (the frozen baselines cannot — mid_ripple defaults to 0); validation now 11 scenarios, 104,576 cells, 0 mismatches.
- **2026-10-02** — **Mid-band reach + Wind-driven travel; Mix knob; flat string.** The mid ripples faded over the last fifth (envelope peak and resonance lock coincide by construction, both were mid-cloth); lock moved to 0.65, envelope widened, tip curvature 6870→11120. Wind now also sets their TRAVEL speed over 0.55–1.25 — measured, the resonance is broad enough to detune (R 0.286…1.000…0.114, asymmetric because a slow forcing still pushes quasi-statically) — with the response compensated out as Flutter Hz does. The comp clamp is tight for a reason worth remembering: the table measures DETAIL, but a slow forcing makes large DISPLACEMENT, so full compensation drove peak to 1.98× the baseline at Wind 0; [0.80, 1.25] holds it ≤ 1.28×. Plot glow and area fill removed (one bright line = applied EQ, one faint = string). Mix knob added (21 parameters), dry/wet before the Leveler. CLOTH & WIND reordered. Python mirrored and the `mid` scenario repointed at the shipped shape — the new formula reduces to the old at the defaults, so validation would otherwise have covered a config the plugin never uses.
- **2026-10-02** — **Flap knob added (22 parameters).** The mid ripples shrinking toward the free end was not damping: the bump envelope returns to EXACTLY ZERO at d = 1, so they were never forced there — the owner's own read. Flap adds flap·d²·(1−bump), filling the tip in without touching the middle (measured: f2 10454→11139 while the tip goes 3649→6523 across the range; flap = 0 reproduces the old envelope exactly). Default 0.45. Two limits recorded: the tip is off-resonance (2.00c local against a 1.57c lock, 1.27× detune) so it couples less per unit forcing however hard it is driven, and Flap is UNBUDGETED like Wrinkle (1.45× peak across its range, on top of the Motion budget). Mirrored in Python with the `mid` scenario running flap = 0.45 so validation covers it.
- **2026-10-02** — **Gain latency decoupled from the host buffer.** Targets were aimed once per block from the control signal at the block's START, with the bank arriving at the END — about a block and a half of lag on top of Smoothing, growing with the buffer (measured at Smoothing 2 ms: 0.0164 at a 32-sample buffer rising to 0.0771 at 512, so most of what could be heard with the knob off was the pipeline, and the plugin sounded different at different buffer sizes). The ramp is now re-aimed every 32 samples: constant 0.0164 at every buffer size, for 1.49% of one core at 512 nodes. Total ~44.6%.
- **2026-10-02** — **Waves amplitude-compensated and extended; the blooms identified.** The big slow curves that swing both directions seemingly at random are the eight gust sites BEATING — incommensurate temporal multipliers give pairwise beat periods of 11.6–88.9 s, and the signed low mode measured −0.108…+0.069 over 150 s without repeating. Waves phases them out because it moves the two lowest-wavenumber sites up out of the 1–2 band. Waves itself felt weak not from range but because finer gust patterns couple less efficiently: rms fell 2.3x from the default to the old top, and the 6–12 band only looked flat because the mid band was holding it up. Compensated via the wind dial: level now flat to 1.14x across the sweep (was 4.3x) and the 6–12 band triples. Range 5.0→8.0 (measured dead past ~7), default knob 0.40→0.24 so the shipped density stays 2.3.
- **2026-10-02** — **Five test textures; Mix power-only; wall grab made reachable.** Test Src is now a six-position switch (off/pink/white/brown/tone/pulse) — tone is a 24-harmonic stack so the EQ reads as timbre movement, pulse is sparse clicks so it exposes ringing and smoothing that steady noise hides. Levels measured: brown at the first-written gain 3.2 was rms 0.806 / peak 3.968, four times full scale; correct gain is 0.70. Mix drops its magnitude LED (has_magnitude() is per knob now) since half/double travel is meaningless on a 0…100% blend. The left wall was never broken — wall_at() expanded plot_bounds only vertically, so a parked wall had half its grab zone outside the hit test and the new dB labels sat on that spot; now expanded horizontally with reach 18 px. Also fixed the synthetic-input harness, which had been sending every click to the wrong monitor (mouse_event absolute coords need MOUSEEVENTF_VIRTUALDESK and a virtual-desktop origin of x = -1920 here) and reporting success while nothing was clicked — it nearly produced a wrong diagnosis.
- **2026-10-03** — **Seeded randomisers on Wind and Flutter (24 parameters).** Small dials on the top-right of each: drag for amount, click to re-roll the seed. Both mechanisms are built from fixed constant sets, so a seed perturbs those constants — different weather, not a different amount; at spread 0 every value equals its constant and the baselines are untouched. Flutter perturbs AMPLITUDE and a STATIC PHASE OFFSET, never frequency: frequency spread inverted the outward-travel invariant (+0.269 → −0.27 at any spread, peak collapsing 0.60 → 0.16), and spatial smoothing did not help — this is the 2026-10-01 finding that a frequency gradient cannot set a travel direction, reintroduced by accident. Shipped version holds direction at +0.28 across the whole range. Seeds live in the state tree, not as AudioParameters. hash_pm1 verified identical across engines; new `random` scenario exercises both spreads. Validation 12 scenarios, 119,936 cells, 0 mismatches.
