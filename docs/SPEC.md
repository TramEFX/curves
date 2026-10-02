# Curves — Technical Specification

Reference for what the plugin does and exactly what every control does, as
built on 2026-10-02. Written as the working document for a knob-by-knob
tuning pass, so each control lists its real mapping, its measured behaviour,
and the interactions that will bite during tuning.

Companion to `ORCHESTRATION.md` (architecture, decisions, change log). Where
the two disagree, the **code** is the truth — `ORCHESTRATION.md` is a
historical log and some older entries have been superseded.

---

## Part 1 — What Curves Is

### 1.1 The inversion

A normal EQ works *parameters → curve*: you set a gain, the display draws
it. Curves works the other way round. A **physical string simulation owns an
array of node values, and the filter bank reads those nodes every block.**
The string is the parameter source. Drawing, wind, flutter and presets are
all just different *writers* to the same nodes; the EQ is the first
*subscriber* to them.

The consequence that matters musically: the EQ is never static. The string
has mass, tension and damping, so what you draw rings, travels, reflects and
settles on its own.

### 1.2 The control string

- A fixed array of **N nodes**, N set by the Precision control (128 / 256 / 512).
- Node values are **normalized displacement, nominally −1 … +1**. They are
  not dB and not Hz — the mapping to dB happens later, at the subscriber.
- Nodes are mapped to frequency **logarithmically**:

  ```
  f(i) = 30 Hz · (18000/30)^( i / (N−1) )      i = 0 … N−1
  ```

  so node 0 = 30 Hz, node N−1 = 18 kHz, spanning **9.23 octaves**. Band
  centres are additionally clamped to 0.45·fs so the top band never crosses
  Nyquist.

  Resolution per node: **87 cents** at N=128, **43 cents** at N=256,
  **21.7 cents** at N=512. This is why fine spatial detail (Wrinkle) only
  becomes meaningful at high Precision.

  A useful property of the log axis: a disturbance that is *n nodes wide* is
  a constant number of **octaves** wide wherever it sits. Wind events are
  octave-shaped, not Hz-shaped.

### 1.3 The physics

A **1-D damped wave equation** on the nodes, integrated with velocity Verlet
at `fs / divisor` ticks per second:

```
a_i = c²·∇²u_i          wave propagation (tension)
    − k·u_i             restoring force toward flat (stiffness)
    − d·v_i             viscous damping
    + f_i               external force (wind, flutter)
```

- `∇²` is a **4th-order Laplacian** in the interior (2nd-order at the two
  edge-adjacent nodes, free ends using Neumann ghosts). The 4th-order
  stencil is what keeps travelling shapes from smearing as they move.
- Stability (CFL): `c·dt ≤ 1` at unit node spacing, asserted and clamped.

**The string is DC-free and tilt-free.** Every tick, the mean *and* the
least-squares linear trend are removed from both position and velocity.
Reason: tension is blind to both a uniform offset and a linear ramp (the
Laplacian of each is zero in the interior), so those two modes are restored
only by the artificial `stiffness` spring and would ring for tens of seconds
as a slow global sway. Semantic consequence: **a globally shifted or tilted
curve is not a string state.** Overall level belongs to Range dB; spectral
tilt, if ever wanted, belongs in the gain mapping as a static control.

### 1.4 How the string becomes EQ

Per block, for each node:

```
disp_i = u_i + Velocity · (v_i / Tension)
dB_i   = clamp(disp_i, −1, +1) · RangeDB
G_i    = 10^(dB_i / 20)
```

`G_i` is the **target** gain for band *i*. The bank then moves its actual
gain toward that target with **one one-pole smoother per band**, time
constant set by Smoothing. That single smoother is the entire smoothing
system — there are no crossfades or exception paths anywhere in the
parameter path.

Each band is a **TPT/SVF peaking filter** in the configuration

```
out = x + bandpass(x) · (G − 1)
```

All N bands run in parallel on the same input and sum onto the dry path.
Two consequences worth knowing: the topology can **boost or cut** a band but
cannot **move** one (a true per-band pan would need a partitioning bank),
and SVF/TPT is used rather than stacked biquads because naive biquads
produce gain errors between adjacent bands at these densities.

### 1.5 Signal flow

```
input ─┬─────────────────────────────────── dry ───┐
       └─ N bandpass taps ─ × (G_i − 1) ─ Steady ─ ┴─ sum ─ Leveler ─ output
                                   ▲
                      per-band one-pole smoothing
                                   ▲
         gain targets  ◀─ string nodes u, v ◀─ StringSim (ticks every `divisor` samples)
                                                    ▲
                            drawing │ wind │ flutter │ wrinkle │ presets
```

### 1.6 The hard anchor — WYSIWYG

**The shape on screen is always exactly the EQ being applied.** This
outranks every other consideration in the project.

Mechanically: the bright curve is drawn from the **filter bank's smoothed,
Steady-modulated gains**, normalized with the *same* ±1 clamp the audio path
uses — never from the sim. The raw sim is shown separately as the faint
reference line and labelled as such. The Leveler is an output-wide gain and
by definition cannot appear on a per-band curve.

The practical obligation this creates: any amplitude mechanism must keep the
string inside ±1 in normal use. Past that the EQ flat-tops, and while the
display stays truthful, the product looks and sounds clipped.

### 1.7 Cost

Measured as **percentage of one CPU core at 48 kHz** (the only number that
matters for audio — an audio callback runs on one thread, so total-CPU
readings on a many-core machine will hide a saturated audio thread
completely):

| Nodes | Sim (wind + flutter) | Filter bank (2 ch) | Total |
|---|---|---|---|
| 128 | 3.8 % | 3.1 % | **6.9 %** |
| 256 | 15.9 % | 5.9 % | **21.7 %** |
| 512 | 30.8 % | 12.2 % | **43.0 %** |

Both stages scale **linearly with sample rate**: at 96 kHz every figure
roughly doubles (512 nodes → ~85 % of a core).

Run `build/Release/bench.exe [sample_rate]` to re-measure.

---

## Part 2 — Control Reference

19 controls in three groups. Format for each: **range · default**, what it
maps to internally, what that does physically, measured behaviour, and the
interactions that matter when tuning.

> **Excitation budget — resolved 2026-10-02.** Force and Flutter used to be
> two independent *amount* controls feeding the same string, so their
> contributions added and neither dial meant anything on its own (turning one
> up changed the other's effect by **2.46×**). They are now **Motion**
> (amount) and **Character** (balance), with the individual source strengths
> solved for rather than set directly. Motion means the same thing at every
> Character setting and every Precision — measured spread **1.06× / 1.11× /
> 1.38×** at 128 / 256 / 512 nodes, with nothing clamping anywhere.
>
> **Wrinkle is still outside this budget** and remains additive: at Motion 1
> it multiplies peak by **1.57×** at Character 0 and 1.12× at Character 0.5,
> so Wrinkle near maximum can still push the EQ into the clamp. Folding it
> into the budget is the remaining piece.

---

### GLOBAL

#### Physics — 0…3, step 1 · default 0 (Free)

Selects the force model, and is the only control that rewrites other knobs.

| Value | Mode | What it configures |
|---|---|---|
| 0 | **Free** | No anchors, no forces. The string only does what you draw. |
| 1 | **Flagpole** | The cloth-aero model. Anchors the high end; sets density taper 2.0 octaves, aero coefficient 0.6, gust wavelength = N, gust phase speed base 192·(N/128). Also sets Damping 1.2, Stiffness 2.0, Tension 300. |
| 2 | **Gust** | No anchors. A single static half-sine force field across the spectrum, amplitude 8·(N/128). Sets Damping 1.5, Stiffness 4.0, Tension 300. |
| 3 | **Elastic** | No anchors, no forces. Sets Damping 2.0, Stiffness 12.0, Tension 300 — tight spring-back. |

**All eight CLOTH & WIND controls are inert outside Flagpole**, which is why
that panel dims and disables itself in the other modes.

**Tuning note:** selecting a preset *overwrites* Damping/Stiffness/Tension,
so it will stomp on manual edits. Modes 2 and 3 differ from Free only by
those three knob values plus (for Gust) a static force — they are thin.

#### Precision — 0…2, step 1 · default 0

Node count and sim tick divisor together.

| Value | Nodes | Divisor | Tick rate @48 k | Resolution |
|---|---|---|---|---|
| 0 | 128 | ÷4 | 12 kHz | 87 cents/node |
| 1 | 256 | ÷2 | 24 kHz | 43 cents/node |
| 2 | 512 | ÷2 | 24 kHz | 21.7 cents/node |

Changing this **rebuilds the sim and the bank**, which allocates on the
audio thread — accepted for now, flagged for a message-thread rebuild.

**Tuning notes:** the knob labels in some older notes say "Fine / Ultra /
Analysis = 256 / 512 / 1024"; the code ships 128 / 256 / 512. Several
features (Wrinkle especially) are resolution-dependent and effectively need
value 2.

#### Range dB — 3…24 dB · default 18

The ± span the normalized string maps onto. A node at +1 becomes
`+RangeDB` of gain; −1 becomes `−RangeDB`.

```
dB_i = clamp(disp_i, −1, +1) · RangeDB
```

This is also the **display normalization**, so changing it rescales the
curve and the audio identically — it cannot break WYSIWYG.

**Tuning note:** this is the master depth control for the whole effect, and
it interacts with everything. At 18 dB a full-scale string swing is a
36 dB peak-to-trough EQ, which is extreme; much of the "too strong" feel in
earlier sessions traces here rather than to the physics knobs.

#### Smoothing — 2…100 ms · default 30

Time constant of the one-pole that moves each band's actual gain toward its
target: `α = 1 − exp(−1 / (τ·fs))`.

This is **the sound of the plugin**. A vibrating string means the filter
gains are always moving, which is frequency modulation of the filters and
produces sidebands. Whether that reads as lush or as cheap is almost
entirely this timescale. Short = fast and edgy, with more sideband content;
long = smeared and smooth, and the display visibly lags the string.

**Tuning note:** this is the only lag between the string and the displayed
curve, and that lag is accepted by design. It is the highest-leverage
under-the-hood control in the plugin.

#### Steady — 0…1 · default 0.35

Per-band dynamic EQ. Each band runs an envelope follower on its own
bandpass magnitude (**5 ms attack, 150 ms release**) and, above a fixed
linear threshold of **0.25**, squeezes that band's own contribution:

```
if env > 0.25:
    over  = env/0.25 − 1
    (G−1) ← (G−1) / (1 + Steady · over)
```

It is **feedforward** — the follower reads the input-side bandpass, not the
output — so it cannot oscillate. Costs ~2 extra ops per band per sample
because the bandpass is already computed.

Its ducking **is** visible on the displayed curve (the bank's gains are what
the display draws).

#### Leveler — 0…1 · default 0.5

Output stage, two layers:

1. **Slow RMS auto-trim** toward **−18 dBFS** (mean-square target
   0.0158489), RMS follower ~0.5 s, trim smoothing ~1 s, correction clamped
   to **±12 dB**. The knob is a *depth* control: the applied correction is
   `g^Leveler`, so 0.5 gives exactly half the dB correction.
2. **Peak limiter** at −1 dBFS, 1 ms attack / 100 ms release.

**The limiter is always active, independent of the knob.** `Leveler = 0`
therefore means "no RMS trim", **not** "bypass" — a hot signal is still
peak-limited. (An older note claiming 0 gives a strictly string-only path
was wrong.)

Control signal is `max(|L|,|R|)` so both channels get identical gain and the
stereo image cannot shift.

**Tuning note:** this is a global gain, so it can produce a slow level pump
on varying material. That is inherent to any auto-trim; shorten the follower
or use 0 if it bothers.

#### Test Noise — 0…1 · default 0

Above 0.5, replaces the input with internally generated pink noise
(Paul Kellet filter). Lets the standalone demonstrate the string with no
audio wiring. A switch, not a continuous control.

---

### STRING

These four apply in every Physics mode.

#### Damping — 0.05…18 /s · default 1.5

The `−d·v` term: viscous damping, a **rate**, applied per node. Higher =
energy leaves faster, so a drawn shape settles sooner.

Damping is *uniform* by design — only force terms divide by node mass. The
one exception is the cloth tail beyond the termination, which carries a
damping multiplier up to 25× so the animation genuinely ends there.

**Tuning notes:**
- The top of the range used to be 40 /s, where a drawn shape decays with a
  ~25 ms time constant — it vanishes within 2–3 video frames and reads as
  "drawing is broken". Range was cut to 18; the top is still fast.
- Since the aero model landed, **damping is largely decoupled from flap
  amplitude** under wind: a 100× damping change moves peak displacement only
  0.84 → 0.50, because aerodynamic drag now dominates the dissipation. It
  still governs how a *deposit* settles.

#### Stiffness — 0…20 /s² · default 4

The `−k·u` term: a spring pulling every node toward flat, independent of its
neighbours. Where tension resists *curvature*, stiffness resists
*displacement itself*, so it is what makes the whole string sag back to zero
when forces stop.

**Tuning note:** its most audible former role is gone. Stiffness used to set
the frequency of the slow global sway (the DC and tilt modes), because those
modes are invisible to tension and stiffness was their only restoring force.
Both are now removed every tick, so Stiffness is a subtler control than it
used to be and will feel weaker than its range suggests.

#### Tension — 50…600 nodes/s · default 300

Wave speed `c`. Sets how fast disturbances travel along the spectrum and how
strongly the string resists curvature (`c²·∇²u`). Clamped internally to the
CFL limit `1/dt`.

Higher tension = faster travel, shallower response, shorter settling.

**Tuning notes:**
- Tension is the **dominant** influence on wind flap amplitude — a 6× change
  gave a 4.7× amplitude spread — which is why Force is normalized against it
  (see Force).
- It also sets the local wave speed that Wrinkle's resonance is tuned to, so
  moving Tension moves the wrinkle sweet spot with it (this is handled: the
  wrinkle phase velocity is recomputed per tick from live `c`).

#### Velocity — 0…1 · default 0

Mixes string *velocity* into the gain mapping alongside position:

```
disp_i = u_i + Velocity · (v_i / Tension)
```

Division by Tension makes the coupling tension-independent. At 0 the EQ
follows the string's shape; turned up it reacts to how fast the string is
moving, which reads as a sharper, more "plucked" response versus a gentle
sway.

**Tuning note:** this is one of the few genuinely under-explored controls —
it has been left at 0 for most of the project's testing.

---

### CLOTH & WIND  *(Flagpole only)*

The wind model: a **transverse wind velocity field** `W` built from 8
travelling gust components, from which the force on each node comes from
**relative** velocity:

```
v_rel = W_i − v_i
F_i   = ρCd · |v_rel| · v_rel / (1 + s_i²)        ρCd = 0.6, s_i = du/dx
```

The `1/(1+s²)` term is an angle-of-attack factor — a steeply sloped element
lies along the flow and catches less of it. Because the force depends on
relative velocity, the string can fight the wind: drag rises quadratically
with how fast a node moves through the air, so amplitude is self-limiting
and no flap limiter is needed.

Cloth geometry: the high end is the **pole** (anchored); the far end is a
free termination that whips. A density taper of 2 octaves makes the string
heavy at the pole and light at the tip — the whip mechanism — and a forcing
envelope `d^1.5` grows from pole to termination.

#### Motion — 0…1 · default 0.45

**The single amount control for the whole cloth.** Sets a target peak string
displacement, linearly:

```
target_peak = 0.85 · Motion · mix_boost(Character, Precision)
```

The two source dials are then *solved for* rather than set directly, by
inverting each source's measured peak-vs-dial curve:

```
wind: peak = 0.8720 · dial^1.679      (measured, 3.7% error)
shed: peak = 1.2756 · dial^0.707      (measured, 1.8% error)

wind_dial = ((1 − Character) · target_peak / 0.8720)^(1/1.679)
shed_dial = (     Character  · target_peak / 1.2756)^(1/0.707)
```

0.85 is the target at Motion = 1. The binding limit is Character = 0, where
wind alone tops out at 0.872, so 0.85 sits just under it and well under the
±1 clamp.

Two measured corrections sit on top, both indexed by Precision because the
sources do not scale with resolution the same way:

- **Per-precision source gains** (`kNodeWindGain`, `kNodeShedGain`). Without
  them Motion = 1 gave peak ~0.37 at 128 nodes and clamped at 512.
- **Mix correction** (`kMixBoost`, a 3×4 table). Two *incoherent* sources do
  not add their peaks exactly — the peaks rarely coincide — so a mix
  under-delivers. The deficit is asymmetric because wind's exponent (1.679)
  is far steeper than shedding's (0.707), and its size varies with
  resolution.

**Measured result:** peak tracks Motion monotonically and near-proportionally
(ratio 0.86 → 1.08 across the dial), with spread across Character of 1.06× /
1.11× / 1.38× at 128 / 256 / 512 nodes. Nothing clamps at any setting.

**Tuning note:** this is now the knob to set first. Everything else in the
panel is character, rate, scale or geometry.

#### Character — 0…1 · default 0.0

**Where the motion comes from**, not how much of it there is.

- **0** — all **gust wind**: a travelling transverse velocity field driving
  the string through aerodynamic drag. Needs the wind to keep blowing.
- **1** — all **vortex shedding**: a van der Pol wake oscillator per node,
  a genuine limit cycle, so the string flaps with no gust wind at all.

Intermediate values split the budget between them, so the two mechanisms
blend without the total changing.

At Character = 1 the air is still but the drag term is still active, so the
string experiences aerodynamic damping from its own motion through it —
correct physics, and it is baked into the calibration.

**Tuning note:** default 0 reproduces the previous wind-only behaviour
exactly. Raising it is a tuning decision and one of the least-explored axes
in the plugin — the shedding mechanism is what makes a *drawn* deposit keep
moving, since gusts are not the only energy source any more.

#### Wind — 0…1 · default 0.37

Gust **phase velocity** — how fast the flapping pattern travels along the
cloth, *not* how hard it blows:

```
wind_phase_speed = (10 + 490 · Wind) · (N/128)   nodes/s
```

The `N/128` factor keeps a given setting meaning the same thing in every
Precision mode; without it the pattern crossed a 512-node string 4× slower
than a 128-node one.

#### Waves — 0…1 · default 0.4

Spatial **density** of the gust pattern — how many waves fit along the
cloth. Scales the wavenumber of all 8 gust components together:

```
wave_density = 0.5 + 2.5 · Waves          (0.5 = stretched, 3.0 = packed)
```

Also scales Wrinkle's wavenumbers, so the two stay coherent.

**Tuning note — measured 2026-10-02, needs no compensation.** Amplitude
across the whole dial varies only **1.50×**, and non-monotonically (peak at
density 1.0):

| Waves | 0.00 | 0.20 | 0.40 | 0.60 | 0.80 | 1.00 |
|---|---|---|---|---|---|---|
| peak \|u\| | 0.803 | 1.011 | 0.854 | 0.881 | 0.675 | 0.704 |

This was previously listed as a compensation target on the strength of an
older 1.7 overshoot at the stretched end, but the tilt blocker already
resolved that. A 1.5× hump is smaller than the gap between Force 0.75 and
1.0, and is arguably characterful — denser waves genuinely carry less energy
per wave. **Left alone deliberately.**

#### Wrinkle — 0…1 · default 0

Fine detail concentrated in the **last third** — "how the cloth resolves".
Adds 4 extra gust components with high spatial wavenumbers {12, 17, 23, 31},
windowed by a `d⁴` envelope so they live at the tip, and — the essential
part — with their **phase velocity locked to the local wave speed at the
tip**, `c·√(2^taper)`.

```
wrinkle = 3 · Wrinkle
```

The resonance lock is not optional. At those wavenumbers the tension
restoring term `c²k²` is ~1944 against a stiffness of ~2, so short waves are
suppressed roughly 1000× and **can only be excited on resonance** — and that
resonance is only a few percent wide. Two cheaper designs (warping the gust
coordinate; extra components at the wind's phase velocity) both did
essentially nothing for exactly this reason.

**Measured:** tip/pole detail ratio 1.11 (off) → 1.45 at 1.5 → 1.82 at 3.0,
with the pole end unchanged — the effect stays where it is aimed.

**Tuning notes:**
- **Still outside the excitation budget.** Unlike Motion and Character,
  Wrinkle adds on top: at Motion 1 it multiplies peak by **1.57×** at
  Character 0 and 1.12× at Character 0.5, so near maximum it can still push
  the EQ into the clamp. Folding it into the budget is outstanding work.
- **Resolution-dependent.** Effective at 512 nodes, marginal at 256, useless
  at 128 (the wavelengths fall below what the grid can carry; an anti-alias
  clamp holds the finest to ≥8 nodes/cycle so it degrades rather than
  aliases). Treat it as a high-Precision control.
- Counterintuitively, *finer* wavenumber sets are **less** effective — past
  a point the modes are too stiff even on resonance.

#### Direction — 0/1 · default 0

Which end the wind blows **from**. 0 = pole at the high end (whip at the
lows); 1 = pole at the low end (whip at the highs). Mirrors the entire cloth
geometry — envelope, taper, termination, tail damping, gust sites — and
re-aims the flutter phase ramp so travel direction flips with it.

A two-position switch presented as a rotary.

#### Length — 0.2…1.0 · default 1.0

Where the cloth animation **terminates**, as a fraction of the array
measured from the pole. Beyond the termination the tail is near-rigid
(inverse mass 0.02) with up to 25× damping, so the motion genuinely ends
there rather than fading out.

The termination node is **not** anchored — it moves freely and whips.
Pinning it kills the whip.

**Tuning note:** shorter cloths are reflection-limited — the near-rigid tail
acts as a reflecting boundary, so outward wave travel is weaker at 0.6 than
at 1.0. That is physics, not a defect.

#### Flutter Hz — 0.1…20 Hz · default 3.0

Nominal wake shedding frequency, set directly in Hz because in Curves this
**is** the audible rate at which the EQ bands wobble. Tuned by ear, not
derived from a Strouhal number — the x-axis is log-frequency, not space, so
no real Strouhal number exists here.

**Rate–amplitude compensation (added 2026-10-02).** Raw, this dial swung
amplitude ~130× across its range: the shedding drives the string's
fundamental resonantly near `f0 = c/span`, and above that the string's own
mass and tension lowpass it away. It behaved as much like a volume control
as a rate control.

The response shape is approximately **universal in `f/f0`** — measured
across tension 150/300/600 and 256/512 nodes, three of four configurations
agree within 0.19 once peak-normalized, and the outlier deviates mostly
below the reachable dial range. So a single normalized table, evaluated at
`f/f0`, adapts automatically to Tension, node count and cloth length:

```
f0   = Tension / span,   span = (N−1) · Length
comp = clamp( kRef / R(f/f0), 0.25, 8.0 )
shed_gain = 90 · Flutter · comp
```

`R` is the peak-normalized response table, log-interpolated; `kRef` is `R`
at the default operating point so **the default setting is left untouched**
(measured 0.877 → 0.876, i.e. no change to any tuning already done by ear).
The 8× boost ceiling is deliberate — fully flattening the top would need
~100×, which is forcing the string cannot use.

Lives in the **plugin mapping layer**, not the engine, so the validated
physics and every CSV baseline are untouched by it.

**Measured, before → after:**

| Rate | before | after | comp |
|---|---|---|---|
| 0.10 Hz | 0.96 | 0.78 | 0.83 |
| 0.45 Hz | 3.98 | 0.99 | 0.25 |
| 0.70 Hz | 3.76 | 1.36 | 0.25 |
| 3.00 Hz | 0.88 | 0.88 | 1.00 |
| 8.00 Hz | 0.17 | 0.71 | 6.38 |
| 20.0 Hz | 0.03 | 0.20 | 8.00 |

Full-range swing **133× → 6.8×**; across the working band 0.3–8 Hz,
**23× → 2.03×**. The residual at the extremes is the boost clamp binding,
which is honest: very fast flutter genuinely is subtle.

---

---

## Part 3 — Known Tuning Targets

Collected from measurements made while building, as the agenda for the
knob-by-knob pass:

1. **Shared amplitude budget** — **mostly done 2026-10-02.** Force and
   Flutter became Motion (amount) and Character (balance); spread across
   Character is now 1.06×/1.11×/1.38× by Precision and nothing clamps.
   **Remaining:** Wrinkle is still additive (1.57× at Character 0).
2. ~~**Flutter Hz behaves like a volume knob**~~ — **done 2026-10-02.**
   Rate–amplitude compensation added in the plugin mapping; swing across the
   working band 23× → 2.03×, default setting unchanged. See its entry above.
3. **Range dB at 18** gives a 36 dB peak-to-trough span; likely too hot a
   default.
4. **Stiffness feels weaker than its range** now that DC and tilt are
   blocked.
5. **Velocity is unexplored** — left at 0 through nearly all testing.
6. **Force is marginal at Precision 2** (long-run peak ~1.01 at full).
7. **Physics presets 2 and 3 are thin** — three knob values and, for Gust, a
   static force field.
8. **Three stepped parameters are rotaries** (Physics, Precision,
   Direction); switches or a selector would read better.
9. **Precision rebuild allocates on the audio thread.**

---

## Appendix — Validation

The C++ engine is cross-validated against the Python spec
(`prototype/string_sim.py`) cell by cell: **10 scenarios, 73,856 cells, 0
mismatches**. Any physics change must land in both implementations with
identical arithmetic order, and the pre-existing scenario CSVs must stay
byte-identical — they are the regression tripwire.

Run:

```
python prototype/run_scenarios.py        # Python spec
build/Release/scenarios.exe prototype    # C++ engine
build/Release/bench.exe 48000            # per-core cost
```
