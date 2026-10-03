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

## State at a glance — end of 2026-10-02

| | |
|---|---|
| Repo | `github.com/TramEFX/curves` (private), branch `main` |
| Head | `2f3e954` |
| Milestones | M0 ✅ · M1 ✅ (owner ear test) · M2 code-complete, **feel gate still pending** |
| Parameters | 22, across 21 controls |
| Validation | 11 scenarios, **104,576 cells, 0 mismatches** Python↔C++ |
| Cost @48 kHz | ~7% / ~23% / ~45% of one core at 128 / 256 / 512 nodes |
| Formats | VST3 (auto-deploys to Common Files), CLAP (build tree only), Standalone |

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
