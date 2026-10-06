# Curves — Session Log

Running record of working sessions: what changed, what was **measured**, what
was decided and why, and what is still open. Backfilled 2026-10-02.

Purpose: survive context loss. Anything expensive to re-derive — calibration
constants, measured response curves, root causes, dead ends — lives here so it
is never measured twice.

**Companions:** `SPEC-0.0.2.md` (current control reference), `SPEC-0.0.1.md`
(superseded), `ORCHESTRATION.md` (architecture and decisions, with a change
log that runs to 2026-10-02).

---

## State at a glance — 2026-10-06

| | |
|---|---|
| What it is | **A physics sandbox for spectral control.** Physics mode and subscriber are both axes; the cloth is one mode, EQ gain is one subscriber |
| Repo | `github.com/TramEFX/curves` (private), branch `main` |
| Head | `24ba49f` |
| Milestones | M0 ✅ · M1 ✅ (owner ear test) · M2 code-complete, **feel gate still pending** |
| Parameters | 28, in 26 spec entries |
| Validation | 12 scenarios, **119,936 cells, 0 mismatches** Python↔C++ |
| Cost @48 kHz | ~7% / ~23% / ~45% of one core at 128 / 256 / 512 nodes |
| Formats | VST3 (auto-deploys to Common Files), CLAP (build tree only), Standalone |

> **Status lives in `SPEC-0.0.2.md` §5** — blocked / pending / open, with
> numbered items (B1, P3, O4 …). One item is **blocked on an owner
> decision**: band overlap (B1), which gates the Place control (B2).

**Three invariants that constrain everything:**

1. **WYSIWYG.** The bright curve is drawn from `FilterBank::gains()` with the
   same ±1 clamp the audio path uses — never from the sim. Any amplitude
   mechanism must keep the string inside ±1 in normal use.
2. **Bit-exact cross-validation.** Every physics change lands in both
   `src/string_sim.cpp` and `prototype/string_sim.py` with identical
   arithmetic order, and the frozen scenario CSVs stay byte-identical.
   New behaviour goes in behind a default-off flag or a new force model so
   the baselines survive.
3. **Channel coherence** (added 2026-10-02). One instance is one string: no
   process inside a single instance may treat L and R differently. Stereo
   interest comes from running separate *instances* per channel.

---

## 2026-10-01 → 10-02 · Session 1

Twenty commits, `3da1368` → `08416ab`.

### Phase 1 — Version control

The project had **zero commits**: 46 KB of design doc, a validated engine and
a working plugin existed only on local disk, because git had never been
configured on the machine. Set identity, committed, pushed to a new private
GitHub repo. `gh` installed via winget; auth via browser passkey.

### Phase 2 — Physics

Prompted by a design discussion the owner had with another assistant. Two of
its suggestions were already done; the valuable ones were relative-velocity
drag and vortex shedding.

**Aerodynamic drag (`kClothAero`).** The wind had been a *prescribed force
field* — a function of position and time with no node velocity anywhere. The
string could be pushed but never push back, which is the root cause of the
entire flap-limiter saga (four separate bug reports). Replaced with a
transverse wind **velocity** field and force from relative velocity:

```
v_rel = W_i − v_i ;  F_i = ρCd · |v_rel|·v_rel / (1 + s_i²)
```

Added as a **new force model** so `kFlagpoleWind` and all six existing
scenarios stayed byte-identical — deliberate, because this made the system
nonlinear and a linear system cross-validates trivially where a nonlinear one
may not. Flap limiter retired.

> **Correction on record:** I claimed drag would be a hard amplitude bound. It
> is not — peak grows ~`wind_speed^1.4` with no ceiling. The saturation lives
> in `aero_rho_cd` (33× more drag only doubles amplitude). The effect is
> *compressive*, not limiting.

Side effect worth keeping: **damping became nearly decoupled** from flap
amplitude — a 100× damping change moves peak only 0.84 → 0.50 — because drag
now dominates dissipation.

**Tilt blocker.** Owner reported a slow low→high sway that outlived the
string. Ruled the Leveler out by inspection (output-wide gain cannot make a
spectral tilt). The cause was **mode 1** — a linear tilt — which has the same
pathology the DC blocker was built for: tension is blind to a linear ramp.
Measured on an asymmetric deposit: tilt starts at **38× the amplitude of all
other modes combined**, every other mode is gone in 2–3 s, tilt still rings
past 10 s at 0.6–0.9 Hz and barely responds to Stiffness. Under wind it was
consuming **0.947 of the ±1 span**. Removing the least-squares linear trend
each tick fixed it, and incidentally resolved a Waves/Force overshoot.

**Leveler.** Found a 6 dB error: `target = 0.0625` was commented as
"(−18 dBFS)²" but 0.0625 is (−12 dBFS)². Fed exactly −18 dBFS the stage
applied **+5.9 dB**. Also widened the range ±6 → ±12 dB and documented that
`amount = 0` is **not** a bypass (the peak limiter is unconditional).

**Vortex shedding ("Flutter").** A van der Pol wake oscillator per node — a
genuine limit cycle, so the acceptance test was motion with the gusts **off
entirely**. Passes. This is also what lets a *drawn deposit* keep moving.

**Wave direction.** Owner reported waves travelling inward. My first
measurement used `argmax(|cross-correlation|)`, which lets anti-correlation
masquerade as a delay and reported the **exact inverse**. Built a 2-D
spatio-temporal FFT metric and **validated it on synthetic waves of known
direction** first — which caught that *it* had the k-sign convention backwards
too. Verdict: gusts 0.621 outward (correct), flutter **0.203 inward**.

Root cause: a **frequency gradient cannot set a travel direction**, because it
makes a phase ramp that steepens with time. Fixed by exploiting that van der
Pol is **neutral in phase** — identical autonomous oscillators keep whatever
phase relationship they start with — so the bank is *seeded* with an outward
ramp. Direction is then correct by construction. 0.70–0.88 outward everywhere.

### Phase 3 — Performance

Owner reported dropouts in high precision with wind, at ~6.5% total CPU.

**The 6.5% was the misleading part.** The machine has 16 logical processors,
so that is **1.04 of a single core**, and an audio callback runs on one
thread. Measured per-stage as a fraction of one core: at 512 nodes the sim
with wind was **86.1%** and the bank 11.5% — **101.4% total**, a guaranteed
dropout. Wind-off totalled 25%, which is why it only glitched with wind.

Cause: the gust field evaluated **8 `sin()` per node per tick** — 98 million
per second. Fixed by expanding `sin(a−b)` and tabulating the time-invariant
spatial part, making the per-tick cost **8 sin + 8 cos total, independent of
node count**. 86.1% → 25.9%. **All CSV baselines stayed byte-identical** — the
rewrite is numerically indistinguishable at 6 decimals.

Also fixed a **priority inversion**: `publish_display()` took a *blocking*
spinlock the editor also holds, so the audio thread could spin on a
lower-priority GUI thread. Now `tryEnter()` and bail.

Added `bench` (`src/bench_main.cpp`, takes a sample rate). **96 kHz roughly
doubles everything** — 512 nodes ≈ 85% of a core.

### Phase 4 — Features and UI

**Wrinkle** — tip-localized fine detail. Two cheaper designs failed first, for
a quantitative reason worth remembering: at those wavenumbers the tension term
`c²k²` is ~1944 against a stiffness of ~2, so short waves are suppressed
~1000× and **can only be excited on resonance**, which is a few percent wide.
Locking the phase velocity to the *tip* wave speed `c·2^(taper/2)` works.
Counterintuitively, **finer wavenumber sets are less effective**.

**UI rework** — grouped panels (GLOBAL / STRING / CLOTH & WIND, the last
dimming outside Flagpole), five colour themes, glow rendering, custom rotary.

> **Bug found only by screenshotting:** `CurvesTheme` has 15 colour fields but
> every literal supplied **14**. C++ aggregate initialization silently shifted
> every later field and zero-initialized the last — and **a zero `juce::Colour`
> is fully transparent, not black**. Every piece of `text_dim` text rendered
> invisibly while the geometry was correct. Compiled without a warning. Guarded
> with a `jassert` now.

### Phase 5 — Knob orthogonalization

Owner asked whether knobs feeding the same thing could be simplified. The
diagnosis: **seven knobs landed on "amount"**.

**Flutter Hz** swung amplitude ~130×. The response shape is approximately
**universal in `f/f0`** (`f0 = c/span`), so one normalized table adapts to
Tension, node count and window automatically. Working band 0.3–8 Hz:
**23× → 2.03×**, with the default setting unchanged.

**Waves** was measured and **deliberately left alone** at the time (1.50×
swing, non-monotonic) — a fix would have been machinery for nothing.

**Motion + Character replaced Force + Flutter.** The two sources add roughly
linearly in peak (10.6% error) and each has a clean power law, so the source
dials are *solved for* rather than set. Spread across Character fell from
**2.46×** to 1.06×/1.11×/1.38× by precision, with nothing clamping.

**Per-knob modifiers** — power and magnitude LEDs, stored as UI state rather
than 38 extra AudioParameters. Plus **RESET**, which flattens the string and
leaves every modifier running.

**Then four reported bugs, all reproduced:**

1. *"Everything off and the string still drifts."* My own blanket rule
   "power off = the knob's minimum" was wrong: Character's minimum is **all
   gust wind**, Wind's is the *slowest* rate, Waves' is the *most stretched*
   pattern. "Everything off" meant "full wind, slowest, largest" — measured at
   0.084 peak, **0.091 Hz, an 11-second period**. Replaced with per-knob power
   modes (`kPowerToMin` / `kPowerGate` / `kPowerNone`).
2. *"Wind off also turns off wrinkle."* True — wrinkle was scaled by
   `wind_speed`. Gave it its own reference.
3. *"Velocity does nothing."* Measured **3.4%** of rms displacement at full;
   the `1/Tension` divisor was ~30× too large. Made it a **blend** rather than
   a sum, because an additive form at the corrected scale would have gone
   straight through the ±1 clamp.
4. *"Length only reduces to ~80%."* Worse than reported — measured, Length
   0.60 still left **99% of the string moving**. The "near-rigid tail" was
   never a wall, because the DC and tilt blockers subtract a **global** mean
   and trend from every node. Replaced with **hard walls** and windowed the
   blockers. Outside the window is now exactly **0.00e+00**.

> **Hypothesis I got wrong and the measurement rejected:** I expected the
> lighter tip (4× via the taper) to damp faster with wind off and look pinned.
> Measured tip/pole decay ratio ≈ 1.0. The real cause was a `last_length_`
> state desync across `apply_physics`.

---

## 2026-10-02 · Session 2 — the gain path

Three commits, `5303f1e` → `9c2d042`. Triggered by the owner's statement of
what Range dB is *for*: "Range shouldn't affect Physics or visuals. Range
should affect how much the visuals are applying EQ."

### What the audit found

Range already satisfied that **when the string is still** — one writer, two
readers, never reaches `sim_`, and the two readers cancel exactly (identical
to six decimals at every Range). But three defects sat in the same path.

**1. The smoother was in the wrong domain.** The one-pole ran on the band's
*linear* gain, so Range sat inside a recursive filter. A one-pole is only
scale-invariant in its own domain:

| string wobble | drawn-curve deviation, Range 3 vs 18 |
|---|---|
| 0.5 Hz | 0.005 |
| 3.0 Hz | 0.162 |
| 8.0 Hz | **0.298** — 30% of canvas height |

and boosts ran **2.21×** faster than cuts at Range 18 (2.83× at 24).

`ORCHESTRATION.md:155` had specified the right form from day one — "one
one-pole smoother **toward the current sim value**". The implementation had
drifted, because the filter consumes a linear gain so the smoother got
written where the gain lived. Not a decision; nobody had ever chosen it.

The owner's framing was the architecture: nodes are evenly spaced in **log
frequency**, `disp` maps **linearly to dB**, the physics is linear in
`disp` — so the chain is already linear physics on a log-log graph, and the
smoother was the one thing outside that coordinate system. Moved onto
`disp` at the **sim tick rate**; Range now enters once, memorylessly.

**2. The gain advanced once per channel.** `process_sample(0,..)` then
`(1,..)` each stepped the shared smoother. **Stereo smoothed at half the set
time** (30 ms behaved as 15 ms) and a mono instance did not match a stereo
one. Nobody had noticed because both sounded plausible.

**3. Steady was invisible on the curve** — a live hard-anchor violation. The
duck multiplied a local `big` and was never written back to `g_`, which is
what the display reads. Hidden gain reduction: **1.98 dB** at the default
Steady 0.35, **4.52 dB** at 1.0. Both the `gains()` comment and the spec
asserted the opposite.

### The decision that resolved it

Steady's follower was **per channel**, so L and R ducked differently and
*no single curve was correct*. Asked whether to draw the max or the mean,
the owner reframed it:

> "the goal for this is to run multiple instances within a sandbox that you
> can run in parallel left and right channels independently … running one
> left and right is really where the sandbox aspect gets powerful."

So: **one instance is one string**, channel-coherent inside, stereo by
*instancing*. The max-vs-mean question dissolves — a mono detector gives
exactly one true curve. `OutputLeveler` already followed this rule. Recorded
as an invariant, with the explicit carve-out that driving a *different*
spectrum (stereo width) is a wanted feature to be built on purpose.

### Verified on the shipped code

`src/gain_test_main.cpp` exercises the real `FilterBank`, not a replica:

| property | before | after |
|---|---|---|
| drawn curve vs 16× Range change | 0.298 | **4.95e-14** |
| boost/cut asymmetry | 2.21× / 2.83× | **1.00×** at every Range |
| Steady hidden from the curve | up to 4.52 dB | **0.00 dB** |

Cross-validation untouched throughout: 73,856 cells, 0 mismatches — the
work never enters `StringSim`.

### Also

- **dB axis on the canvas.** Gridlines at round dB values (step from
  {1,2,3,6,12,24,48}, ≤4 per side), labelled, derived from the **effective**
  Range so High magnitude on an 18 dB knob reads ±36 and says so. Verified on
  screen at Range 3.74 (1 dB steps) and 18 (6 dB steps).
- **Two more modifier desyncs**: Smoothing and Steady both read the raw
  parameter in `processBlock` and silently ignored their own magnitude LEDs.
  Same class as the `last_length_` bug.
- **The flutter compensation is clamped at the default patch.** Its reference
  point was measured at 512 nodes (`f/f0 = 5.11`), but the default Precision
  is 128 — where `comp` = **0.25**, pinned at its min clamp, not the 1.0 the
  spec implied. Latent today because Character defaults to 0, which makes the
  shed branch inert. It bites the moment Character leaves 0.

### Corrections on record

- **I projected a 33% CPU reduction from flop counting. Wrong.** The SVF
  dominates the per-band cost, so removing the one-pole barely moves it.
  Measured at 512 nodes: bank unchanged within noise (12.16 vs 12.06), and
  the control smoother **adds 1.57%** of one core. The redesign is
  **cost-neutral**; it was never a performance change.
- A naive cell-by-cell diff of the scenario CSVs reports **156,679 mismatches
  of 369,330** and means nothing: Python writes `0.0` where C++ writes
  `0.000000`, and the scenario column is tagged `cpp_pluck` vs `pluck`. The
  real comparison is the `value` column parsed as floats — 73,856 cells.
  Noted because the naive check looks catastrophic and is pure formatting.

### Open after this session

- **Smoothing's default is un-tuned.** Stereo is now 2× slower at the same
  setting; 30 ms is inherited, not chosen. Needs ears.
- **Steady is the one remaining Range interaction** with the curve's shape,
  because the duck multiplies the linear `big`. Real behaviour, shown
  exactly — flagged rather than fixed.
- **Character at higher precisions**, the owner's next target. Hypothesis
  (unverified): amplitude is already budgeted flat, but shedding spreads over
  4× the oscillators at 512 nodes so each couples less. Measure spatial and
  temporal spectrum and the self-sustaining fraction before changing
  anything — three hypotheses have been wrong on this project already.

### Tension vs amplitude

Reported: "Tension compresses the waveform physics amplitude-wise a bit more
than I would prefer." Real, and the compensation that was supposed to prevent
it (`tension_norm = (c/300)^0.6`) under-corrected everywhere below 512 nodes.

**The exponent is derivable.** At steady state drag balances tension,
`ρCd·W² ≈ c²k²·u`, so `u` is Tension-free only when wind speed scales
**linearly** with wave speed — exponent **1.0**. Measurement lands just under
it, which the relative-velocity term explains (a faster-moving string sees
less `v_rel`). Swept at 128 nodes: 0.6 → 3.15× spread, **0.9 → 1.56×**,
1.2 → 4.55×, 1.5 → 17.5×.

**It has to be per-precision**, and that was the non-obvious part: peak
against Tension is a **hump**, not a slope. The hump sits near a fixed
`f0 = c/span` — Tension ≈100 at 128 nodes, ≈450 at 512, both giving
`f0 ≈ 0.8 Hz` — so no single exponent flattens the whole dial, and raising it
to 0.9 made **512 nodes worse** (2.60× → 7.85×). Shipped
`kTensionP = {0.85, 0.80, 0.60}`, tuned on the upper half where the complaint
lives; 512 left alone because it was not broken.

`(c/300)^p = 1` at Tension 300 for any `p`, so the Motion calibration at the
default Tension survives untouched — no re-fit needed.

**Two findings this turned up, both deferred:**

- **The Motion budget under-delivers ~37% at the default operating point.**
  At Motion 0.45 / Character 0 / 128 nodes / Tension 300 the budget predicts
  peak **0.371**; measured **0.234**. The mapping replication is identical to
  the plugin's, so the power law `0.8720·dial^1.679` is likely stale — fitted
  before the Waves range extension (density at the default went 1.5 → 2.3,
  and the spec's own table says peak falls with density) and before the walls
  windowed the DC/tilt blockers. Needs a re-fit.
- **The shedding path has no Tension compensation at all.** At Character 1,
  peak falls **80.7×** across the dial. `shed_gain` carries only the Flutter
  rate compensation, which is peak-*normalized* and therefore blind to the
  string stiffening as `c` rises. An impedance term `(c/300)²` only reaches
  2.95×, with an unexplained dip at Tension 300. Belongs with the Character
  work.

> **Method note that paid for itself.** `tension_test` self-validates against
> `scenario_aero`'s recorded `0.722754` before printing anything. The first
> version of that check read **0.1185** — it scored only the last 8 ms and
> caught a quiet moment. Had I not built the check, that 6× error would have
> looked like catastrophic dt-dependence and invalidated every number here.
> Corrected: the sim is dt-converged to **0.3%** from 1 kHz to 24 kHz. Note
> also that the frozen scenarios run at **1 kHz**, not the plugin's 12/24 kHz.

### Wind, and the two drawn lines

Reported: Wind "not really affecting the string anymore — no difference at
high levels or low levels", plus the faint sim line visibly disagreeing with
the bright EQ line even at low Smoothing.

**Wind was mis-ranged, not inert.** What the eye sees depends on whether the
string can *follow* the gust — a question about `f_gust/f0`. The old mapping
held `f_gust` constant across precision while `f0 = c/span` falls as `1/N`, so
the same setting landed in completely different places:

| | 128 nodes | 512 nodes |
|---|---|---|
| `f_gust/f0`, old default | 0.64 | **2.54** |
| `f_gust/f0`, dial top | 1.65 | **6.67** |

At 512 the whole upper dial was past the string's ability to respond; measured
peak across the dial was 0.21 / 0.45 / 0.28 / 0.24 / 0.18 — **both ends land
in the same place**, which is precisely the complaint. Now a multiple of the
wave speed (`0.12*25^Wind`), mirroring the `f0` normalization Flutter Hz
already uses. After: 0.14 / 0.40 / 0.44 / 0.51 / 0.32.

**Damping ruled out by measurement** before any change: 10x damping moves the
observed motion frequency by 4%, because drag dominates dissipation.

**The two lines were the smoothing lag, nothing else.** Max |sim - drawn| over
14 s at 512 nodes: **0.019 / 0.050 / 0.119** at Smoothing 2 / 10 / 30 ms. At
the minimum they coincide (rms 0.0024), which clears the display path. Fixing
the once-per-channel double-step had doubled the real smoothing at any
setting, so the default moved 30 -> 15 ms to restore the prior feel. Both
reported symptoms trace back to that one change of mine.

> **Unvalidated metric, flagged.** The "motion frequency" column is a spectral
> centroid on one probe node. It is **not** validated against a known signal
> and behaves erratically at 512 nodes; a cross-correlation travel-speed metric
> tried alongside it returned negative and wildly varying lags and was
> discarded — the same failure mode as the 2026-10-01 direction metric. The
> peak figures are solid; do not build on the Hz column without validating it.

**Still open on Wind:** amplitude across the dial remains non-monotonic, and at
512 nodes the string rings in its own modes rather than tracking the gust.
Whether the re-range is enough is an ear call.

### Mid-band ripples

Reported: "there's big slow waves, and there's very small ripples when I turn
wind up really high … we need some mid to mid small sized ripples that wind
affects. Not sure if this will end up being a tuning thing or an additive
thing."

**Measurement answered it: additive.** New `spatial_test` bins |FFT across
nodes| of the string shape by waves-across-the-string:

| | 1–2 | 2–6 | 6–12 | 12–25 | 25–64 |
|---|---|---|---|---|---|
| Wind 0.0 | 0.01733 | 0.00631 | 0.00175 | 0.00105 | 0.00076 |
| Wind 1.0 | 0.02665 | 0.01289 | 0.00556 | 0.00322 | 0.00233 |
| Waves 0.25 | 0.05252 | 0.01595 | 0.00825 | 0.00560 | 0.00413 |
| Waves 1.00 | 0.01171 | 0.00518 | 0.00292 | 0.00154 | 0.00111 |

Three facts, each of which kills a cheaper option:

1. The **1–2 wave band carries 5–15×** everything above it, at every setting.
2. **Raising Waves does not move energy up the spectrum** — it makes every band
   smaller. Tuning cannot do this.
3. **Wrinkle is not targeted** — it lifts every band roughly proportionally,
   including 1–2 by 2.9×.

Structurally the gust sites cover 1.7–6 waves and wrinkle 25–64, with nothing
between. And the reason it must be *built* rather than tuned is the same wall
wrinkle hit: at those wavenumbers `c²k²` dwarfs stiffness, so the modes are
**only reachable on resonance**.

Four components, locked to the **mid-cloth** wave speed `c·2^(taper/4)`, with a
raised-cosine bump envelope centred at `d = 0.5` — exactly where that lock is
resonant. Wrinkle at the tip, this in the middle, gust across the lot.

**Owner chose: no new knob.** Wind sets the size (6 → 20 waves), amount follows
Motion.

| amount | 6–12 band | peak vs off |
|---|---|---|
| 0.0 | 0.00400 | 1.00× |
| 1.0 | 0.01297 | 1.01× |
| **1.5** | **0.02086** | **1.06×** |
| 2.0 | 0.02952 | 1.12× |

Shipped at 1.5: **5.2× the mid-band content for 1.06× peak**. Compare wrinkle's
1.57×, a recorded defect — resonance-locking plus mid-localization adds
structure rather than bulk, which is why it is nearly free in the budget.

Cost: **+4.05% of one core** at 512 nodes (sim tick 29.47 → 33.52), +0.26% at
128. Total still under 50% at the highest precision.

**Validation extended.** The frozen baselines could not test this — `mid_ripple`
defaults to 0 so they stay byte-identical by construction, which proves
nothing about the new code. Added a `mid` scenario to *both* engines; they
report mid-third curvature **948.902818** and peak **0.602248** to the digit.
Validation is now **11 scenarios, 104,576 cells, 0 mismatches**.

### Mid-band reach, Wind-driven travel, and three UI items

**Reach.** The mid ripples faded out over the last fifth, because the
envelope peak and the resonance lock sit at the same place by construction
and both were at mid-cloth. Lock moved to `d = 0.65`, envelope exponent
2.0 → 1.4, amount trimmed 1.5 → 1.2 to pay for it. Tip-fifth curvature
**6870 → 11120**.

**Travel.** Asked for Wind to make the ripples run faster down the string.
The honest constraint is that they are *resonant* — that is why they are
excitable at all — so their natural speed is the string's own wave speed.
But the resonance turned out broad enough to detune usefully:

| travel | 0.35 | 0.50 | 0.65 | 0.80 | **1.00** | 1.20 | 1.45 | 1.80 | 2.40 |
|---|---|---|---|---|---|---|---|---|---|
| R | 0.286 | 0.346 | 0.476 | 0.794 | **1.000** | 0.593 | 0.286 | 0.166 | 0.114 |

Asymmetric — a slower forcing still pushes quasi-statically, a faster one
outruns the string. Wind maps to 0.55–1.25 (2.3×) with R compensated out,
the same shape of fix as `flutter_rate_compensation`.

> **The compensation nearly shipped a bug, and the catch is the lesson.**
> That table measures **detail** (curvature), but a slow-travelling forcing
> makes large *displacement* while making little detail. Compensating detail
> fully drove peak to **1.98×** the no-mid baseline at Wind 0 — straight at
> the ±1 clamp. Caught only because the shipped mapping was measured
> end-to-end rather than assumed from the component tables. Clamp `[0.80,
> 1.25]` holds peak ≤ **1.28×** everywhere while still roughly doubling
> low-Wind detail against no compensation. **A compensation table is only
> valid for the quantity it was measured on.**

**Also:** glow and area fill removed from the plot (one bright line = applied
EQ, one faint = the string; knob arcs keep their glow). **Mix** added — 21
parameters now — dry/wet after the bank and *before* the Leveler, so the
Leveler trims what is actually heard. CLOTH & WIND reordered to Motion /
Wind / Character / Flutter Hz. GLOBAL had to go to one row of **8**: at 7 the
new knob wrapped Test Noise out of the panel height and clipped it, found by
screenshotting.

**Validation note.** `mid_lock_d` / `mid_env` / `mid_travel` were mirrored in
Python, and the `mid` scenario in *both* engines was repointed at the
**shipped** shape. The new envelope formula reduces exactly to the old one at
the default values, so the baselines held — which meant validation would
otherwise have passed while covering a configuration the plugin never uses.

### Flap — the tip was never forced

Owner: "the dampening at the end of the wind may not be a dampening issue at
all. It may be the fact that is the end of the simulation." **Correct**, and
it was my envelope, not the physics. The mid-band bump returns to *exactly
zero* at `d = 1`:

| d | 0.65 | 0.80 | 0.90 | 0.95 | 0.99 | 1.00 |
|---|---|---|---|---|---|---|
| envelope | 1.000 | 0.709 | 0.311 | 0.122 | 0.013 | **0.000** |

Flap adds `flap·d²·(1 − bump)` — fills the tip in, leaves the middle alone,
and `flap = 0` reproduces the old envelope exactly so the baselines hold.

| flap | f2 (middle) | tip | tip/f3 | peak |
|---|---|---|---|---|
| 0.00 | 10454 | 3649 | 0.37 | 1.11× |
| **0.45** | 10755 | 4303 | 0.43 | **1.27×** |
| 1.00 | 11139 | 6523 | 0.63 | 1.61× |

`f2` barely moving across the whole range is the design working.

**Two limits recorded rather than hidden:**

- **The tip is off-resonance.** Local wave speed there is `c·2^(taper/2) =
  2.00c` against a lock at `1.57c` — a 1.27× detune — so it couples less per
  unit forcing than the middle *no matter how hard it is driven*. That is
  why even `flap = 1` leaves the tip below the `f3` peak. Shifting the lock
  toward the tip with Flap would buy more whip at the cost of moving the
  band's centre; not done, available if wanted.
- **Unbudgeted, like Wrinkle** — 1.45× peak across its range, on top of the
  Motion budget rather than inside it. Fine at default Motion (peak 0.55
  against the ±1 clamp), will clip near Motion 1.

### "A hell of a lot of smoothing, even when smoothing is off"

Not the knob. The gain targets were aimed **once per host block**, from the
control signal as it stood at the *start* of the block, and the bank only
reached them by the *end* — roughly a block and a half of lag stacked on top
of Smoothing, growing with the buffer. Measured max |sim − drawn| at
Smoothing 2 ms, the knob fully off:

| buffer | 32 | 64 | 128 | 256 | 512 |
|---|---|---|---|---|---|
| before | 0.0164 | 0.0206 | 0.0288 | 0.0451 | **0.0771** |
| after | 0.0164 | 0.0164 | 0.0164 | 0.0164 | **0.0164** |

At a 512-sample buffer **most of the audible smoothing with the knob off was
the pipeline** — 4.7× the residual. It also meant the plugin sounded
different at different buffer sizes, a defect on its own.

Fixed by re-aiming the ramp every **32 samples** rather than once per block:
one `exp2` per node and one `pow` per band per chunk, **1.49% of one core**
at 512 nodes. Latency is now constant and independent of the host.

> **Worth generalising.** This is the third time a control has turned out to
> be fighting something structural rather than being mistuned — after the
> smoother living in the wrong domain and Wind being mis-ranged against
> `f0`. The pattern: when a knob "doesn't do much" or "does too much at every
> setting", measure what *else* is acting on the same quantity before
> touching the knob's range.

### Which line am I hearing?

**The bright one.** It is `FilterBank::gains()` — the gains the audio is
going through. The faint line is raw `u`, a *control signal*, not the EQ.
What separates them, measured at 512 nodes:

| source | max | rms |
|---|---|---|
| Smoothing 2 ms (off) | 0.016 | 0.003 |
| Smoothing 15 ms (default) | 0.081 | 0.013 |
| Smoothing 100 ms (max) | 0.190 | 0.033 |
| Steady 0.35, loud input | 0.036 | 0.006 |
| **Velocity 0.5** | **0.400** | 0.066 |
| **Velocity 1.0** | **0.812** | 0.133 |

Only **Velocity** makes the faint line a different *shape* rather than a
lagged copy — the bright line then follows a position/velocity blend while
the faint stays pure position. That is what "a mutated version of it" looks
like. Steady's share is content-dependent, so it moves with the material.

> **A violation I introduced and did not notice.** Answering this turned up
> that the curve ignored **Mix** entirely — at Mix 0.5 it drew the full EQ
> depth while half was applied. Live for the few hours between Mix landing
> and the question being asked. Folded in exactly: the bank emits
> `x + Σ v1·big` and Mix scales only the added part, so the effective gain
> is `1 + Mix·(g − 1)`. **Adding any new stage after the bank needs the same
> check** — the anchor is only as good as the last thing bolted on.

### Where the big blooms come from, and why Waves felt weak

**The blooms are the eight gust sites beating.** Their temporal multipliers
are deliberately incommensurate, so at the default operating point they run
at 0.265–0.786 Hz and the pairwise differences land at **11.6, 13.4, 13.9,
19.0, 19.6, 25.2, 28.4 and 88.9 s**. Measured over 150 s, the signed
low-mode amplitude swings −0.108 … +0.069 with large excursions every 10–25 s
and never repeats. **Quasi-periodic, not random**, and it swings both ways
because it is a sum of sinusoids, not a rectified envelope. The two
lowest-wavenumber sites (1.66 and 1.91 waves across the cloth) put that beat
into the 1–2 band — which is exactly why raising Waves phases the blooms
out: it moves those sites up out of that band.

**Waves felt weak because it drained the string, not because of its range.**

| density | 0.5 | **2.3** | 5.0 | 9.5 | 18.0 |
|---|---|---|---|---|---|
| rms, before | 0.139 | **0.090** | 0.039 | 0.029 | 0.027 |

2.3x quieter from default to the old top. And the 6–12 band stayed *flat*
across the entire sweep — what held it up was the **mid band**, which is
independent of `wave_density` by design, while the gust contribution died
underneath it. So the knob's apparent effect was mostly "everything gets
quieter".

Compensated (a multiplier on the wind dial, `R^(1/kWindP)`):

| density | 0.5 | **2.3** | 5.0 | 8.0 |
|---|---|---|---|---|
| rms | 0.088 | **0.090** | 0.087 | 0.080 |
| 6–12 band | 0.0118 | 0.0163 | 0.0318 | **0.0373** |

Level flat to 1.14x (was 4.3x) and the 6–12 band **triples**. Range extended
5.0 → 8.0 — measured, every band flattens past ~7, so that is where it
genuinely stops. Default knob 0.40 → 0.24 keeps the shipped density at 2.3.

> **Fourth instance of the same pattern.** After the smoother in the wrong
> domain, Wind mis-ranged against `f0`, and Smoothing competing with a
> buffer-sized pipeline: a knob that "needs turning way up" was not short of
> range, it was fighting an uncompensated side-effect on the same quantity.
> Checking the *level* across a control's sweep is now the first thing to do
> when one feels weak.

### The big slow arc: what owns its rate vs its size

Asked of a specific patch (low Tension, low Waves, Flutter powered off,
Motion/Wind on High magnitude, Velocity well up, Range 3 dB). `patch_test`
reconstructs it and sweeps each suspect:

| change | centroid | period | arc swing |
|---|---|---|---|
| that patch (Ten ~110) | 8.40 | **1.87 s** | 0.507 |
| Tension -> 300 | 8.20 | **0.72 s** | 0.350 |
| Tension -> 600 | 8.13 | **0.37 s** | 0.289 |
| Waves -> 1.0 | **11.98** | 1.76 s | **0.182** |
| Wind -> 0.30 | 7.05 | **6.26 s** | 0.726 |
| Flutter ON | 8.41 | 0.55 s | **1.017** |

**It is the string's own lowest surviving mode** (mode 2 — DC and tilt are
blocked). Theoretical period `span/(2c)`: 2.32 s at Tension 110, 0.85 s at
300, 0.43 s at 600, against 1.87 / 0.72 / 0.37 measured. Consistently ~80%
of theory because the measurement includes higher modes too.

Division of labour, now measured rather than assumed:

- **Tension owns the RATE.** 5x across its range, and it is the only control
  that moves the rate without also moving the size.
- **Waves owns the SIZE.** Centroid 8.4 -> 12.0, and it cuts the arc's swing
  **2.8x** (0.507 -> 0.182) — the same "phases the blooms out" effect, seen
  from the other side.
- **Wind moves both**, and in the same direction: lower Wind is slower *and*
  bigger (6.26 s, swing 0.726).
- **Flutter adds to it rather than breaking it up** — turning shedding on
  *doubles* the low-mode swing (0.507 -> 1.017). Worth knowing: it is not a
  way to add detail without touching the big arc.

Separately: at that patch the two drawn lines differ in *shape*, not just
lag, because **Velocity is well up** — exactly the case isolated in the
"which line am I hearing" measurement.

### Test textures, Mix LED, and the wall that was never broken

**Five test textures** replace the pink-noise on/off: off / pink / white /
brown / tone / pulse, chosen to probe a *moving* EQ rather than just make
noise — tone is a 24-harmonic stack so the EQ reads as timbre movement on a
pitched sound, pulse is ~7 clicks/sec so it exposes ringing and smoothing
that steady noise hides completely.

> **Levels measured, not eyeballed.** Brown was first written at gain 3.2.
> Measured: **rms 0.806, peak 3.968** — four times full scale, and it would
> have shipped clipping. A leaky integrator with `a = 0.996` driven by
> `0.04*uniform` has `sigma = 0.2585`, so 0.70 is correct. Final set, rms /
> peak: white 0.202/0.350, brown 0.180/0.719, tone 0.197/0.394, pulse
> 0.105/0.795. **Any generator written by eye needs its level measured
> before it ships.**

**Mix** loses its magnitude LED — "half travel" is meaningless on a 0…100%
blend and High would put 100% out of reach. `has_magnitude()` is per knob now.

**The left wall was never broken — it was unhittable.** `wall_at()` rejected
anything outside `plot_bounds` expanded only *vertically*, so a wall parked
at an edge had half its 12 px grab zone outside the test, and the dB axis
labels added the same day sit exactly there. Dragging from *inside* the plot
worked the whole time. Expanded horizontally, reach 12 -> 18 px, parked
posts dim to 0.60 rather than 0.35.

> **The harness was lying, and nearly produced a wrong diagnosis.** The
> first synthetic test showed *neither* wall moving, which looked like
> confirmation they were dead. Sanity-checking against the theme button —
> which has an unmistakable effect — showed that too did nothing: no click
> was landing at all. Cause: `mouse_event` absolute coordinates are
> **primary-monitor-relative** unless `MOUSEEVENTF_VIRTUALDESK` is set and
> they are normalised against the virtual desktop, whose origin here is
> **x = -1920**. Every click had been going to the wrong screen while the
> script reported success. **Always validate a UI harness against a control
> with an obvious effect before trusting a negative result from it.**

### Seeded randomisers — and a bug this project had already fixed once

Two small dials on Wind and Flutter Hz: drag for amount, click for a new
seed. Both mechanisms are built from **fixed constant sets**, which is why
they are regular; a seed perturbs those constants, so it is different
*weather* rather than a different amount.

**Wind** re-rolls the gust sites' rate and wavenumber — i.e. the beat
structure behind the big blooms. Peak across seeds at spread 0.5: 0.417 /
0.578 / 0.685.

**Flutter took two attempts, and the first repeated a known bug.**

| spread | 0.00 | 0.25 | 0.50 | 0.75 | 1.00 |
|---|---|---|---|---|---|
| frequency spread — direction | +0.269 | **−0.180** | **−0.273** | **−0.267** | **−0.251** |
| frequency spread — peak | 0.602 | 0.156 | 0.204 | 0.252 | 0.280 |
| **shipped** (amplitude + static phase) | +0.269 | **+0.281** | **+0.288** | **+0.289** | **+0.286** |
| **shipped** — peak | 0.602 | 0.579 | 0.555 | 0.529 | 0.501 |

Per-node frequency spread **inverted the outward travel** and collapsed the
flutter. Making it spatially smooth did not help (−0.43) — the gradient is
the problem, not the roughness. This is verbatim the 2026-10-01 finding:
*a frequency gradient cannot set a travel direction, because it makes a
phase ramp that steepens with time.* The fix then was identical
oscillators; I reintroduced exactly what that fix removed. Shipped version
perturbs **amplitude and a static phase offset only** — a static offset
persists instead of accumulating, so direction degrades gracefully and
never flips.

> **The metric check earned its keep twice in one sitting.** `rnd_test`
> reproduces `scenario_shed` and refuses to be believed unless it returns
> its recorded **+0.372**. First run: a hardcoded `dpdi = +1` where the
> convention is `-1` at `wind_origin = 0` — every direction reading
> inverted. Second: the validation block itself did not match the scenario
> (it added `anchor_high` and used gain 30 instead of 60) and read
> **−0.511**, so the check flagged SUSPECT. Only after it printed +0.3722
> were any of the numbers above trustworthy. Without that gate I would have
> reported the *opposite* conclusion — which is the third time a direction
> metric has tried to do that on this project.

Seeds ride in the state tree rather than being AudioParameters. Both
engines mirrored; `hash_pm1` (lowbias32) verified identical across 15
probes, and a new `random` scenario runs both spreads non-zero in both —
at spread 0 the frozen baselines say nothing about this path. **12
scenarios, 119,936 cells, 0 mismatches.**

## 2026-10-04 · Framing corrected — the cloth is one mode

Not a code change. Writing a summary for Notion surfaced that the project's
own documents described **cloth-in-wind as if it were the product**. The
owner corrected it:

> "the cloth is just one mode there will be multiple modes with different
> physics applying to play around with like a physics sandbox. The whole
> product is a physics-based string driving the EQ or other Spectrum ratio
> parameters"

So Curves is **a physics sandbox for spectral control**, generalising along
two axes that are both first-class:

| Axis | Today | Open |
|---|---|---|
| **What drives the string** — physics mode | Flagpole deep, Free works, Gust + Elastic thin | Physics is not yet a switch between *distinct simulations*. Candidates: fluid/current flow, granular or particle impacts, magnetic repulsion between nodes, coupled/multi-string |
| **What the string drives** — subscriber | EQ gain only | Stereo width per band, per-node Q, cutoff modulation — all attach to the same control signal |

**Why this is worth recording rather than just rewording.** It changes where
future work belongs. `StringSim` publishes nodes and knows nothing about
filters, so a new mode replaces the physics without touching the bank, and a
new subscriber attaches without touching the physics — the engine already
assumes both axes.

> **A debt the reframing exposed.** The *editor* does not assume them. The
> CLOTH & WIND panel is a Flagpole-shaped control set hard-coded alongside
> the generic ones, so **adding a physics mode is UI work as well as
> physics**. That was not previously written down anywhere, and it is the
> main cost of axis 1.

Updated: `ORCHESTRATION.md` §1 (with the correction stated explicitly rather
than quietly rewritten), `SPEC-0.0.2.md` (scope note, §1 dataflow, the
Physics entry), `CLAUDE.md` (lead paragraph). Also corrected stale figures in
`CLAUDE.md` — it still said 11 scenarios / 104,576 cells and 22 parameters.

## 2026-10-06 · Spatial subscriber, and a pre-existing defect it exposed

### Built

Bank moved to **mid/side** — verified identical to the L/R form at
**3.3e-16** while mid and side share a gain, so the move was free. Three
per-band coefficients: `gm` (EQ), `gs` (width = their difference in dB),
`pan` (placement, `S += pan*v1m`). New **SUBSCRIBERS** panel. Width matches
the EQ's own behaviour to **0.18%**; placement is **mono-safe** — `rms(L+R)`
unchanged to 1.000× across the whole pan range.

### The defect: band overlap, and it is pre-existing

`out = x + Σ bp_b·(g_b − 1)`. With **Q = 1** the bands are ~1.4 octaves wide
while the spacing is `9.23/N` octaves, so **~0.15·N bands overlap** at any
frequency and their contributions **add**:

```
actual = 1 + K·(g − 1),    K ≈ 0.152 · N
```

| bands | nominal | **actual** | fitted K | 0.152·N |
|---|---|---|---|---|
| 8 | +6 dB | +5.56 dB | 0.90 | 1.2 |
| 48 | +6 dB | +17.14 dB | 6.23 | 7.3 |
| 128 | +6 dB | +25.12 dB | 17.11 | 19.5 |
| 256 | +6 dB | +30.98 dB | 34.56 | 38.9 |
| **512** | **+6 dB** | **+36.92 dB** | **69.47** | 77.8 |

Three consequences, none of them previously written down:

1. **Every per-band depth is nominal, not actual.** At 512 nodes the EQ is
   ~70× deeper than the knob says. The owner had already compensated by ear
   — running Range at **3.74 dB** against a default of 18.
2. **Cuts invert.** Once `K·(1−g) > 1` the effective gain goes *negative*:
   measured, a nominal −3 dB at 512 nodes comes back as **+25.85 dB** with
   inverted polarity. A cut is not an attenuation.
3. **Precision cross-couples with depth.** K runs 17 → 69 from 128 to 512
   nodes, so changing resolution changes the EQ depth **~4×**.

**This is the unimplemented per-node Q coming due.** Q = 1 is sensible for a
handful of bands; with one band per node it guarantees the pile-up. Matching
bandwidth to spacing needs `Q ≈ 0.16·N` — about 80 at 512 nodes.

**Why it matters now:** width and the EQ inherit the factor *equally*, so
they stay consistent with each other and the single-curve story holds. But
**placement has an absolute meaning** — pan 1.0 should be hard left — and at
K = 69 it is reached around pan 0.014. Measured at 48 bands, pan 1.0 gives
an L/R ratio of 1.27 (degenerate, both channels huge) instead of the
predicted ∞, and +16 dB of level. **Placement is unusable at its stated
range until this is resolved.**

Two routes, both affecting everything already tuned by ear:

- **Normalise** the band contribution by `1/K` — one scalar, keeps the smooth
  Q = 1 response, makes Range dB mean what it says and removes the Precision
  coupling.
- **Per-node Q** matched to spacing — the architecturally intended fix, but
  high Q will sound resonant and is a much larger change.

Left for the owner to decide; not fixed unilaterally, because either route
changes every sound currently dialled in.

## 2026-10-06 · Spatial subscriber shipped; the curve found twice

Built the stereo subscriber on a **mid/side** bank — verified identical to
L/R at 3.3e-16 while mid and side share a gain, so the move was free. Width
is a depth on the side in the same dB unit as EQ, so the two read one curve
and differ only in how hard they drive it. Placement is `S += pan·v1m`,
measured **mono-safe** (rms(L+R) unchanged to 1.000×). New SUBSCRIBERS panel;
EQ dB moved into it and now gates the EQ entirely. 28 parameters.

**The invariant was renamed and reworded**, not bent: "channel coherence"
described a mechanism we deliberately left behind. It is now **image
provenance** — every per-band coefficient must be a deterministic function of
the control surface and therefore *drawable*. Sharp test: if you cannot draw
it, its provenance is wrong. That still forbids the Steady bug while
permitting string-driven stereo.

> **The curve went missing twice, from the same mistake.** First with the EQ
> gated off (shape was in the side gain, unread); then with Width also at
> zero (shape was in `pan_`, unread). Both times I *enumerated* display
> sources and missed one. It is now an exhaustive ordered fallback
> (SPEC §F8b) verified as a **set** across all five configurations, not case
> by case. The lesson is the method, not the cases: a fallback chain over a
> growing number of subscribers must be tested exhaustively or it will keep
> losing one.
>
> Also worth keeping: the final state, all three subscribers at zero, draws
> a flat curve — which is **correct**, nothing is applied — but looked
> identical to the bug. The axis now says "no subscriber on".

**And a defect the stereo work exposed** — band overlap (B1 in §5). It is
pre-existing, it makes every per-band depth nominal rather than actual, and
it is the unimplemented per-node Q coming due. Blocked on an owner decision
because either fix reshuffles everything tuned by ear.

---

## Measurements worth not re-deriving

**Cost, % of one core @48 kHz** (`bench.exe [rate]`; ×2 at 96 kHz):

| nodes | sim (wind+flutter) | bank | total |
|---|---|---|---|
| 128 | 3.8 | 3.1 | **6.9** |
| 256 | 15.9 | 5.9 | **21.7** |
| 512 | 30.8 | 12.2 | **43.0** |

**Motion budget** — source power laws and calibration:

```
wind: peak = 0.8720 · dial^1.679   (3.7% err)
shed: peak = 1.2756 · dial^0.707   (1.8% err)
kMotionPeak = 0.85 ;  dial ceiling 2.0 (High magnitude over-drives)
kNodeWindGain = {1.590, 1.031, 0.913}   per precision
kNodeShedGain = {2.796, 0.978, 1.099}
kMixBoost     = 3×4 table (incoherent sources under-add)
```

**Flutter Hz** — peak-normalized response vs `f/f0`:

| f/f0 | 0.15 | 0.30 | 0.50 | 0.75 | 1.00 | 1.50 | 2.50 | 4.00 | 7.00 | 12.0 | 25.0 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| R | 0.23 | 0.43 | 0.75 | 0.93 | 1.00 | 0.93 | 0.63 | 0.34 | 0.117 | 0.043 | 0.010 |

**Waves** — spatial centroid vs density (reverses past ~8):

| density | 0.5 | 1.5 | 3.0 | 5.0 | 8.0 | 12.0 |
|---|---|---|---|---|---|---|
| centroid | 1.03 | 1.21 | 1.38 | 1.50 | 1.63 | 1.38 |
| peak | 0.70 | 0.74 | 0.60 | 0.41 | 0.30 | 0.21 |

**Velocity** — `|v|/|u| ≈ 10` at Tension 300. Blend with `kVelScale = 30`
holds rms 0.074–0.107, peak 0.54–0.85, **0% clamping** across the dial.

**Wrinkle** — envelope `d^2.8` (reach 56.1% of the cloth, 1.28× the old
`d^4`); amplitude `3.9·knob`; tip/pole detail ratio 6.20× in-scenario. Still
**outside the excitation budget**: ×1.57 peak at Character 0.

---

## Method notes

- **Measure before fixing.** Three hypotheses were wrong this session (drag as
  a hard limiter; tip-damping as the "anchor"; the Leveler as the sway) and
  measurement caught all three before any code was written for them.
- **Validate a metric on a known signal first.** The direction metric was
  wrong twice — once from `abs()`, once from a sign convention — and synthetic
  waves caught both.
- **Trust the CSV cross-diff, not a printed statistic.** The bit-exact
  tripwire once fired on a *reporting* bug (a loop excluding the end node,
  plus pairwise vs sequential summation) while the sims were identical.
- **A visual change needs looking at.** The build was clean and the layout
  maths right while the UI was substantially broken.
- **Windows UI automation, hard-won:**
  - Screenshots: `PrintWindow(hwnd, dc, PW_RENDERFULLCONTENT)`, *not* a screen
    grab — a screen grab catches whatever window is in front.
  - Synthetic clicks: need `AttachThreadInput` before `SetForegroundWindow`,
    or Windows silently refuses and every click is dropped.
  - Synthetic drags: `SetCursorPos` with a button held does **not** generate
    the move messages JUCE tracks. Use absolute `mouse_event` moves.
  - Scripts live in the session scratchpad (`shot2.ps1`, `click2.ps1`,
    `drag2.ps1`).

---

## Open threads

**Needs the owner's ears** — nothing here can be settled by measurement:

1. **M2 feel gate** — Flagpole "physical, not animated", zipper-free. Still
   the formal milestone exit.
2. **Magnitude multipliers `{0.5, 1.0, 2.0}`** are a guess, not a measurement.
   Medium is calibrated; Low and High are untested by ear.
3. **Wall drag** verified only by click-to-position; drag could not be driven
   synthetically.
4. **Range dB default 18** gives a 36 dB peak-to-trough span. Probably hot;
   set it before tuning physics.

**Known defects / deliberate debts:**

5. **Wrinkle is outside the excitation budget** (×1.57 at Character 0) — the
   last piece of the Motion/Character work.
6. **Whip termination within the window was lost** with the Length knob;
   `cloth_length` is pinned at 1 inside the walls. May want it back as its own
   control.
7. **Stiffness is weaker than its range** now that DC and tilt are blocked.
8. **Physics modes 2 and 3 are thin** — knob values plus, for Gust, a static
   field. Candidates for presets, freeing Physics to be a real mode switch.
9. **Per-node Q is unimplemented** — `Q = 1.0` fixed, despite being a recorded
   architectural decision.
10. **Precision rebuild allocates on the audio thread.**
11. **CLAP is never auto-deployed** — no copy step; it only exists under
    `build/Curves_artefacts/Release/CLAP/`.
12. **Three stepped parameters are rotaries** (Physics, Precision, Direction).

**Next up, per the owner:** a knob-by-knob tuning pass, now that the controls
are orthogonal enough for it to measure something real.

---

## Housekeeping

- `prototype/cpp_*.csv` are regenerated by every `scenarios.exe` run, so they
  show as modified after each validation pass. Useful as a tripwire, noisy
  otherwise.
- Reaper holds the VST3 open; the auto-copy fails with "Permission denied"
  while it is running. Close Reaper, rebuild, rescan.
- `ORCHESTRATION.md` changelog entries from several commits cite a stale
  validation cell count. 73,856 is current.
