# Curves — Technical Specification **v0.0.2**

**Status: current.** Supersedes `SPEC-0.0.1.md`, which is kept for history.

An exhaustive knob-by-knob reference covering all **20 parameters** across
**19 controls** (the two Walls share one entry). For every control it states:

1. **Assigned parameters** — the exact engine variables it writes.
2. **What it does** — the effect, in plain language.
3. **Functions** — the mapping function that produces the value, and the
   consuming function that uses it, both defined.

Consuming functions are defined once in §2 and referenced by tag (**F1**…
**F12**) so each knob entry stays short without being vague.

Verified against source on 2026-10-02 at commit `08416ab`. Where this
document and the code disagree, the code is the truth and this document is a
bug.

**Session history, measurements and open threads:** `SESSION-LOG.md`.

---

## §0 — Notation

| Symbol | Meaning |
|---|---|
| `N` | node count (128 / 256 / 512, set by Precision) |
| `i` | node index, `0 … N−1` |
| `u_i`, `v_i` | node displacement and velocity (normalized units) |
| `f_i` | external force on node `i` |
| `W_i` | transverse wind **velocity** at node `i` |
| `dt` | sim tick period = `divisor / fs` |
| `c` | wave speed (`Tension`), `c² = c2_` |
| `span` | `(N−1) · (wall_hi − wall_lo)` — the active window, in nodes |
| `p_i` | distance of node `i` from the anchored pole, in nodes |
| `d_i` | `p_i / span`, clamped to 0…1 |
| `f0` | string fundamental ≈ `c / span`, in Hz |

**Units.** `u` is dimensionless, nominally −1…+1. `v` and `W` are in
u-units per second. The x-axis is **log frequency**, not space, so
quantities like the drag coefficient are tuning constants with physical
*structure*, not measurable physical values.

---

## §1 — Dataflow

```
knob  ──mapping fn──▶  engine parameter  ──consuming fn──▶  effect
```

Everything a knob does happens in exactly one of five places:

| Stage | Where | Runs at |
|---|---|---|
| **A** String physics | `StringSim` | every sim tick (`fs/divisor`) |
| **B** Node → gain map | `processBlock` | once per node per block |
| **C** Filter bank | `FilterBank` | every sample, per band |
| **D** Output stage | `OutputLeveler` | every sample |
| **E** Structure | `rebuild` / `apply_physics` | on change only |

---

## §2 — Consuming Functions

### F1 · `StringSim::tick()` — one integration step

Order is load-bearing; each step depends on the previous.

1. `update_wind_forces()` (**F3**) — builds `f_` and `W_`.
2. Velocity Verlet using `compute_accel()` (**F2**) twice.
3. **DC blocker** — subtract `mean(u)` and `mean(v)`.
4. **Tilt blocker** — subtract the least-squares linear trend from `u` and `v`.
5. `enforce_anchors()` — zero `u` and `v` at any anchored end.

Steps 3–4 exist because tension is blind to both a uniform offset and a
linear ramp (the Laplacian of each is zero in the interior), so those two
modes are restored only by `stiffness` and would ring for tens of seconds as
a slow global sway.

### F2 · `compute_accel(a, u_now, v_now)` — the force balance

```
a_i  = c² · ∇²u_i                       wave propagation (tension)
     − stiffness · u_i                  restoring force toward flat
     + f_i                              external force field
     + ρCd · |v_rel| · v_rel / (1+s_i²) aerodynamic drag   [kClothAero only]
a_i  = m_inv_i · a_i                    divide force terms by node mass
     − damping · damping_mult_i · v_i   viscous damping (a RATE, not a force)
```

- `∇²` is a **4th-order** Laplacian in the interior; 2nd-order at the two
  edge-adjacent nodes; free ends use Neumann (zero-slope) ghosts.
- `v_rel = W_i − v_now_i`, clamped to `aero_vrel_max_`.
- `s_i = du/dx` (central difference). `1/(1+s²)` is the angle-of-attack
  factor: a steeply sloped element lies along the flow and catches less of it.
- Drag uses **`v_now`**, the velocity passed in, not the stored `v_` —
  Verlet calls this twice per tick with different velocities, and using the
  stored one would lag the feedback by half a step.
- Only **force** terms divide by mass. Damping is a rate and does not.

### F3 · `update_wind_forces()` — builds the forcing

Runs only when `force_model` is a wind model. Three parts:

**(a) Gust field.** Eight travelling components:

```
W_i = wind_speed · wind_env_i · (1/8) Σ_s [ sin(ψ_s)·C_s,i − cos(ψ_s)·S_s,i ]
ψ_s = site_phase_s + k_s · site_pos_s
k_s = wind_k · wave_density · kSiteK_s
site_phase_s += wind_k · wind_phase_speed · kSiteMult_s · dt
```

`C` and `S` are the precomputed spatial tables from **F6**. The expansion of
`sin(a−b)` is what makes the per-tick cost **8 sin + 8 cos total**, rather
than 8 transcendental calls *per node*.

**(b) Wrinkle.** Four more components with high wavenumbers, a `d^2.8` tip
envelope, and phase velocity locked to the tip wave speed:

```
W_i += wind_speed · wrinkle · (1/4) Σ_j [ sin(φ_j)·WC_j,i − cos(φ_j)·WS_j,i ]
φ_j += wrinkle_k_j · c_tip · dt,     c_tip = c · 2^(taper/2)
```

**(c) Shedding.** A van der Pol wake oscillator per node:

```
q̈_i = −ε·ω·(q_i² − 1)·q̇_i − ω²·q_i + coupling·v_i,    ω = 2π·shed_freq_hz
f_i  = shed_gain · q_i · wind_env_i
```

The `(q²−1)` term is negative damping below `|q|=1` and positive above, so
`q` is a **limit cycle** — the string flaps with no gust wind at all.
Skipped entirely when `shed_gain == 0`.

### F4 · `recompute_cloth()` — geometry rebuild

Rebuilds every per-node geometry array from `wind_origin`, `cloth_length`,
`density_taper` and `envelope_exponent`:

| Array | In the cloth (`d ≤ 1`) | In the tail (beyond termination) |
|---|---|---|
| `wind_env_i` | `d^1.5` | `0` |
| `m_inv_i` | `2^(taper·d)` | `0.02` (near-rigid) |
| `damping_mult_i` | `1` | up to `25` |
| `cloth_d_i` | `d` | `1` |

Then calls **F5** and **F6**. The termination node is **not** anchored — it
moves freely and whips; pinning it kills the whip.

### F5 · `seed_wake_phase(preserve_amplitude)`

Imposes an **outward** travelling phase ramp on the wake bank:

```
q_i  =  r_i · cos(−k·p_i),   q̇_i = −r_i · ω · sin(−k·p_i)
k    =  2π · shed_waves_per_cloth / span        (1.5 waves per cloth)
```

van der Pol is an attractor in amplitude but **neutral in phase**, so
identical autonomous oscillators keep whatever phase relationship they start
with — travel direction is therefore correct *by construction* rather than
emerging from a coupling gain. `preserve_amplitude` keeps each oscillator's
current magnitude, so moving a Wall or Direction re-aims the travel without
restarting the flutter from silence.

### F6 · `rebuild_gust_tables()`

Precomputes the time-invariant spatial part of (a) and (b) above:
`C_s,i = cos(k_s·p_i)`, `S_s,i = sin(k_s·p_i)`, laid out `[node·8 + site]`
so one node's eight sites are contiguous. Wrinkle tables additionally bake
in the `d^2.8` tip envelope, and carry an **anti-alias clamp** holding the
finest wrinkle to ≥ 8 nodes per cycle. Rebuilt only when geometry or
`wave_density` changes — never per tick.

### F7 · Control smoothing + node → gain mapping

**The only smoothing in the plugin**, and it runs on the string's own
coordinate rather than on the band gain. Per sim tick, per node:

```
disp_i  = (1 − Velocity)·u_i + Velocity·(30·v_i / Tension)
sd_i   += α·(disp_i − sd_i)          α = 1 − exp(−1/(τ·tick_rate))
```

Then once per block, per node:

```
dB_i = clamp(sd_i, −1, +1) · RangeDB
G_i  = 2^(dB_i / 6.0206)             ( ≡ 10^(dB_i/20) )
```

**Why the smoother is here and not in the bank.** `disp` is already the
log-amplitude coordinate: the x axis is log frequency and `disp` maps
*linearly* to dB. So all state lives on a Range-free quantity and **Range
enters exactly once, memorylessly**, which makes the drawn curve independent
of it by construction. A one-pole on the *linear* gain is not
scale-invariant — measured before the change, the drawn curve moved up to
**0.298 of full scale** from a Range change alone and boosts ran **2.21×**
faster than cuts at Range 18. Measured after: **4.95e-14** and **1.00×**.

Running at the tick rate rather than per audio sample is also an
**anti-alias filter** ahead of the block-rate sampling the bank sees, and
makes the knob independent of the host's buffer size.

`G_i` is the block **target**; **F8** walks there.

### F8 · `FilterBank::process_frame(l, r, stereo)`

The bank holds **no time constant of its own**. Given a block target it
precomputes one geometric step per band, `r_b = (G_b / g_b)^(1/n)`, then per
**sample frame** — once, not once per channel:

```
g_b  *= r_b                                   exactly linear in dB
v1    = bandpass(x)                           TPT SVF, Q = 1.0 fixed
big   = g_b − 1
                                              ── Steady, if amount > 0 ──
mag   = max(|v1_L|, |v1_R|)                   MONO detector
env_b += (mag > env_b ? atk : rel)·(mag − env_b)    5 ms / 150 ms
if env_b > 0.25:  big /= 1 + Steady·(env_b/0.25 − 1)
g_eff_b = 1 + big                             ← what the display draws
y     += v1 · big
```

Output is `x + Σ_b v1_b·big_b` — a dry path plus summed peaking bands. This
can **boost or cut** a band but cannot **move** one. Steady is
**feedforward** (the follower reads the input-side bandpass), so it cannot
oscillate.

**Geometric, not linear.** `g *= r` lands on the target exactly after `n`
frames and every intermediate value is linear in dB, for one multiply. A
linear ramp in gain was measured at up to **24 dB** of mid-block error at
Smoothing 2 ms.

**Once per frame, not once per channel.** The previous entry point took a
channel index and stepped the shared smoother on each call, so stereo
smoothed at **half** the set time and a mono instance did not match a
stereo one.

**One mono detector.** Per the channel-coherence rule, Steady derives one
duck from `max(|v1_L|,|v1_R|)` and applies it identically to both channels,
so there is exactly one true curve to draw. `gains()` returns `g_eff`,
Steady included.


### F9 · `OutputLeveler::process(l, r)`

```
rms   += rms_α · (max(l²,r²) − rms)                  ~0.5 s
trim_t = clamp( sqrt(0.0158489 / rms), 0.2512, 3.9811 ) ^ Leveler
trim  += trim_α · (trim_t − trim)                    ~1 s
lim    = 0.891 / lim_env   if lim_env > 0.891        −1 dBFS, 1 ms / 100 ms
l, r  ×= trim · lim
```

Target `0.0158489` is the **mean square** of −18 dBFS. The limiter is
**unconditional** — `Leveler = 0` means "no RMS trim", *not* bypass.
Control signal is `max(|L|,|R|)`, so the stereo image cannot shift.

### F10 · `rebuild(precision)`

Sets `n_nodes_` and `sim_divisor_`, constructs a new `StringSim` at
`tick_rate = fs/divisor`, calls `bank_.prepare(N, fs, 2)` and rebuilds the
band map `f(i) = 30·600^(i/(N−1))`, clamped to `0.45·fs`. **Allocates on the
audio thread** — a known defect.

### F11 · `apply_physics(mode)`

Resets anchors, force model and taper, then configures the selected mode.
Also writes Damping / Stiffness / Tension knob values from the editor.

### F12 · `flutter_rate_compensation(ratio)` + `mix_boost(c, prec)` + `dial_for_peak(...)`

Plugin-side mapping helpers; defined in full under the knobs that use them
(Flutter Hz, Motion, Character).

---

## §2b — Per-Knob Modifiers and Reset

### Modifiers (added 2026-10-02)

Every **continuous** knob carries two extra controls, drawn as LEDs flanking
its label and clickable:

| LED | Position | States |
|---|---|---|
| **Magnitude** | left of the label | green = low, orange = medium, red = high |
| **Power** | right of the label | yellow = on, black = off |

```
magnitude: value = lo + (raw − lo) · {0.5, 1.0, 2.0}   low / medium / high
```

**Magnitude scales travel above the minimum**, so **High can exceed the
nominal maximum** — that is the point. Medium is the calibrated setting.
Every continuous knob has one.

**What power-off means is per knob**, and a blanket rule was wrong here. The
first version collapsed every knob to its minimum, which produced a real bug:
with Character, Wind, Waves and Wrinkle all "off" the string still drifted in
large slow curves, because Character's minimum is 0 = **all gust wind**,
Wind's minimum is the *slowest* rate (measured 0.091 Hz — an 11-second
period) and Waves' minimum is the *most stretched* pattern. "Everything off"
was really "full wind, slowest, largest".

| Mode | Knobs | Off means |
|---|---|---|
| `kPowerToMin` | Motion, Steady, Velocity, Wrinkle, Leveler, Stiffness, Smoothing, Range dB, Damping | the knob's minimum — genuinely neutral for these (Motion 0, Steady 0, Smoothing 2 ms, Range 3 dB, Damping 0.05) |
| `kPowerGate` | **Wind**, **Flutter Hz** | the whole **mechanism** is silenced, not the value: Wind zeroes the gust field, Flutter Hz zeroes shedding |
| `kPowerNone` | Character, Waves, Tension | **no power LED** — "off" is meaningless for a balance or a scale |

So **Wind power off is the way to turn the gusts off**, and Flutter Hz power
off silences shedding; between them, plus Motion, every excitation source has
an independent kill switch. Verified on screen: with Wind powered off and
RESET pressed, the curve is flat and stays flat.

When a knob with a power LED is off, **both** its LEDs go dark. Knobs with no
power LED keep their magnitude LED lit.

**Not AudioParameters.** 19 knobs × 2 would add 38 entries to the host's
automation list for what is a tuning aid, so these ride in the state tree
(`knobMods`) like the colour theme. Promote any that prove essential later.

**Physics, Precision, Direction and Test Noise have no modifiers at all** —
they are modes and switches where "half range" is meaningless.

**Interaction with the Motion budget.** `dial_for_peak` clamps the solved
source dial at **2.0**, not 1.0. At Medium the dial never exceeds 1 and the
Motion calibration holds exactly; at High it deliberately over-drives past
the calibrated peak and past the ±1 EQ clamp, visibly. Clamping at 1 made
High a no-op at the top of the Motion dial.

### Reset

`RESET` in the header flattens the string — zeroes displacement and velocity
— and **touches nothing else**. Wind, flutter and every knob keep running, so
the cloth immediately starts moving again from rest. Serviced on the audio
thread via an atomic flag, before the sim ticks. Replaces the workaround of
winding Damping up to kill energy.

---

## §3 — Knob Reference

### GLOBAL

---

#### 1 · Physics  `physics`
**0…3 step 1 · default 0 (Free) · Global**

| Assigns | Value |
|---|---|
| `sim_->force_model` | `kStaticField` (0,2,3) or `kClothAero` (1) |
| `sim_->anchor_high` | `true` in Flagpole |
| `sim_->set_density_taper` | `0.0`, or `2.0` in Flagpole |
| `sim_->set_cloth_aero(192·N/128, N, 1.5)` | Flagpole only |
| `sim_->aero_rho_cd` | `0.6` in Flagpole |
| `sim_->set_force_field(field)` | half-sine of amplitude `8·N/128` in Gust |
| Damping / Stiffness / Tension | overwritten per mode (editor side) |

**What it does.** Chooses the force model. Free = nothing but what you draw.
Flagpole = the full cloth-aero model. Gust = one static force field.
Elastic = tight spring-back via knob values only.

**Functions.** → **F11**, which configures **F4**; the model then gates
**F3**.

**Interactions.** Overwrites three STRING knobs on selection, so it stomps
manual edits. **All eight CLOTH & WIND knobs are inert outside Flagpole**,
which is why that panel disables itself. Modes 2 and 3 are thin — three knob
values plus, for Gust, a static field.

---

#### 2 · Precision  `precision`
**0…2 step 1 · default 0 · Global**

| Value | `n_nodes_` | `sim_divisor_` | Tick @48 k | Resolution |
|---|---|---|---|---|
| 0 | 128 | 4 | 12 kHz | 87 cents/node |
| 1 | 256 | 2 | 24 kHz | 43 cents/node |
| 2 | 512 | 2 | 24 kHz | 21.7 cents/node |

Also indexes three calibration tables: `kNodeWindGain`, `kNodeShedGain`,
`kMixBoost` — the two excitation sources scale differently with resolution,
so each carries its own per-precision gain.

**Functions.** → **F10**. Sim and bank are reconstructed.

**Interactions.** Allocates on the audio thread. Wrinkle needs value 2 to
work at all. Cost: **6.9 % / 21.7 % / 43.0 %** of one core at 48 kHz;
everything scales linearly with sample rate.

---

#### 3 · Range dB  `drive`
**3…24 dB · default 18 · Global**

| Assigns | `max_gain_db_` |
|---|---|

**What it does.** The ± dB span the normalized string maps onto. A node at
+1 becomes `+RangeDB`.

**Functions.** → **F7**. Also the **display normalization**, so it rescales
curve and audio identically and cannot break WYSIWYG.

**Interactions.** Master depth for the whole effect. At 18 dB a full-scale
swing is a **36 dB peak-to-trough** EQ — set this before tuning anything
physical, or you will tune the physics around a too-hot master.

**It does not touch the physics, and it does not move the picture.** It has
one writer and two readers (the gain map and the display normalization),
never reaches `sim_`, and since 2026-10-02 all smoothing state lives
upstream of it, so the drawn curve is **exactly** invariant to it: measured
deviation **4.95e-14** of full scale across a 16× Range change, moving or
still. Before that fix the same measurement gave **0.298**. The canvas now
labels its dB axis from the *effective* Range (post magnitude modifier).

---

#### 4 · Smoothing  `smoothing`
**2…100 ms · default 30 · Global**

| Assigns | `smooth_alpha_ = 1 − exp(−1/(τ·tick_rate))`, `τ = ms/1000` |
|---|---|

**What it does.** Time constant of the one-pole on the **control signal** —
`disp`, not the band gain. **This is the sound of the plugin**: the gains are
always moving, which is frequency modulation of the filters and produces
sidebands. Whether that reads as lush or cheap is almost entirely this
timescale.

**Functions.** → **F7**.

**Changed 2026-10-02.** Moved out of `FilterBank` and into the control path
(see **F7** for why), and the double-step bug fixed: the old smoother
advanced once *per channel*, so **a stereo instance smoothed at half the set
time** (30 ms behaved as 15 ms) and did not match a mono one. It also read
the raw parameter in `processBlock` and therefore ignored its own magnitude
modifier.

**Consequence for tuning: the default now feels about 2× slower than it used
to in stereo.** The default is deliberately left at 30 ms — where it should
sit is an ear call, not a measurement.

**Interactions.** The only lag between string and displayed curve, accepted
by design. Highest-leverage under-the-hood control in the plugin.

---

#### 5 · Steady  `steady`
**0…1 · default 0.35 · Global**

| Assigns | `bank_.steady_amount_` (threshold fixed at `0.25`) |
|---|---|

**What it does.** Per-band dynamic EQ. Each band follows its own bandpass
magnitude (5 ms / 150 ms) and, above threshold, squeezes its own
contribution.

**Functions.** → **F8**.

**Fixed 2026-10-02 — two defects.** The follower was kept **per channel**,
so Steady ducked L and R by different amounts: a dynamic stereo-image mover,
against the channel-coherence rule, and it meant **no single curve was
correct** — so the display showed *none* of the ducking while claiming to.
Measured hidden gain reduction at the default Steady 0.35: **1.98 dB**, and
**4.52 dB** at Steady 1.0. That was a live violation of the hard anchor.

The detector is now **mono** — `max(|v1_L|,|v1_R|)`, one follower per band,
one duck applied identically to both channels — and `gains()` returns the
Steady-inclusive gain. Measured after: **0.00 dB hidden** at every amount.
It also read the raw parameter and ignored its magnitude modifier; fixed.

**Interactions.** Feedforward, so it cannot oscillate. Costs ~2 ops/band/
sample since the bandpass already exists. Its ducking **is** visible on the
curve — as of 2026-10-02 this is actually true.

**Steady is the one place Range still interacts with the curve's shape**,
because the duck multiplies the *linear* `big`. That is real behaviour, not
a display artifact, and the curve shows it exactly.

---

#### 6 · Leveler  `leveler`
**0…1 · default 0.5 · Global**

| Assigns | `leveler_.amount_` |
|---|---|

**What it does.** Depth of a slow RMS auto-trim toward −18 dBFS, correction
clamped to ±12 dB. Applied as `g^Leveler`, so 0.5 gives exactly half the dB
correction.

**Functions.** → **F9**.

**Interactions.** **The peak limiter is always on regardless of this knob** —
0 means "no RMS trim", not "bypass". Being a global gain it can pump slowly
on varying material; that is inherent to any auto-trim.

---

#### 7 · Test Noise  `test`
**0…1 · default 0 · Global**

| Assigns | `test_on = (value > 0.5)` |
|---|---|

Replaces the input with internally generated pink noise (Paul Kellet). A
switch, not a continuous control. Lets the standalone demo the string with
no audio wiring.

---

### STRING — applies in every Physics mode

---

#### 8 · Damping  `damping`
**0.05…18 /s · default 1.5 · String**

| Assigns | `sim_->damping_` via `set_damping()` |
|---|---|

**What it does.** Viscous damping — a **rate**. Higher = energy leaves
faster, so a drawn shape settles sooner.

**Functions.** → **F2**, as `− damping · damping_mult_i · v_i`. Note it does
**not** divide by node mass: damping is a rate, not a force. `damping_mult_i`
is 1 in the cloth and up to 25 in the tail (**F4**).

**Interactions.** Under wind it is **largely decoupled from amplitude** —
a 100× change moves peak only 0.84 → 0.50, because aerodynamic drag now
dominates dissipation. It still governs how a *deposit* settles. The old
40 /s top end made drawing look broken (≈25 ms decay); range cut to 18.

---

#### 9 · Stiffness  `stiffness`
**0…20 /s² · default 4 · String**

| Assigns | `sim_->stiffness_` via `set_stiffness()` |
|---|---|

**What it does.** A spring pulling every node toward flat, independent of
its neighbours. Where tension resists *curvature*, this resists
*displacement itself*.

**Functions.** → **F2**, as `− stiffness · u_i`, inside the group that
divides by node mass.

**Interactions.** **Weaker than its range suggests.** Its main audible job
used to be setting the frequency of the slow global sway, but the DC and
tilt blockers (**F1** steps 3–4) remove both of those modes every tick. A
prime candidate for rescaling.

---

#### 10 · Tension  `tension`
**50…600 nodes/s · default 300 · String**

| Assigns | Used by |
|---|---|
| `sim_->c2_ = min(c, 1/dt)²` | **F2** wave term |
| `tension_norm = (c/300)^0.6` | wind speed |
| `f0 = c / span` | Flutter Hz compensation |
| `c_tip = c·2^(taper/2)` | Wrinkle resonance (**F3b**) |
| `v_i / Tension` | Velocity coupling (**F7**) |

**What it does.** Wave speed. Sets how fast disturbances travel along the
spectrum and how strongly the string resists curvature. Clamped to the CFL
limit `1/dt`.

**Interactions.** **The most entangled knob in the plugin** — it reaches
five separate places. Each is deliberately normalized against it (the wind
speed term, the flutter `f0`, the velocity divisor), so the *other* knobs
keep their meaning as Tension moves; but Tension itself changes travel rate,
curvature response, wrinkle tuning and settling together. Tune it early.

---

#### 11 · Velocity  `velocity`
**0…1 · default 0 · String**

| Assigns | `vel_amt` in **F7** |
|---|---|

**What it does.** Blends the gain mapping between the string's *position*
and its *velocity*:

```
disp_i = (1 − Velocity)·u_i + Velocity·(30 · v_i / Tension)
```

At 0 the EQ follows shape; at 1 it follows how fast the string is moving —
plucked rather than swaying.

**Fixed 2026-10-02** (user: "not noticing velocity doing much"). It was
additive and scaled by `1/Tension` alone, which made it nearly inert:
measured, it contributed only **3.4%** of the rms displacement at full.
Cause — `|v|/|u|` is about **10** at Tension 300, so the divisor was ~30×
too large. Dividing by Tension is still right in principle (v/u scales with
wave speed, which is what keeps the knob tension-independent); only the
constant was wrong.

Made a **blend** rather than a sum, which is self-normalising: measured rms
stays within **0.074–0.107** and peak within **0.54–0.85** across the whole
dial, with **0.00% clamping at every setting**. An additive form at this
scale would have gone straight through the ±1 clamp. The mild dip at
Velocity 0.5 is `u` and `v` being roughly 90° out of phase.

---

### CLOTH & WIND — Flagpole only

---

#### 12 · Motion  `motion`
**0…1 · default 0.45 · Cloth**

| Assigns | Via |
|---|---|
| `sim_->wind_speed` | `25·wind_dial·(c/300)^0.6·(128/N)^0.25·kNodeWindGain[p]` |
| `sim_->shed_gain` | `90·shed_dial·comp·kNodeShedGain[p]` |

**What it does.** **The single amount control.** Sets a target peak string
displacement, linearly. The two source dials are *solved for*, not set.

**Mapping functions.**

```
target_peak = 0.85 · Motion · mix_boost(Character, Precision)

dial_for_peak(P, A, k) = clamp( (P/A)^(1/k), 0, 1 )

wind_dial = dial_for_peak((1−Character)·target_peak, 0.8720, 1.679)
shed_dial = dial_for_peak(   Character ·target_peak, 1.2756, 0.707)
```

The two constants per source are the **measured** peak-vs-dial power laws:
wind `peak = 0.8720·dial^1.679` (3.7 % error), shed
`peak = 1.2756·dial^0.707` (1.8 % error). 0.85 is the target at Motion = 1;
the binding limit is Character = 0, where wind alone tops out at 0.872.

```
mix_boost(c, p):  piecewise-linear over c ∈ {0, 0.33, 0.66, 1}, per precision
    p=0 (128):  0.969  1.462  1.106  0.847
    p=1 (256):  1.007  1.340  1.017  0.999
    p=2 (512):  0.968  0.978  0.805  1.012
```

Two **incoherent** sources do not add their peaks exactly — the peaks rarely
coincide — so a mix under-delivers. The deficit is asymmetric (wind's
exponent 1.679 is far steeper than shedding's 0.707) and varies with
resolution, hence a 3×4 table.

**Functions.** → **F3a** (gusts) and **F3c** (shedding) → **F2**.

**Measured.** Peak tracks Motion monotonically and near-proportionally
(ratio 0.86 → 1.08). Spread across Character: **1.06× / 1.11× / 1.38×** at
128 / 256 / 512 nodes. Nothing clamps at any setting.

**Interactions.** Set this first. Everything else in the panel is character,
rate, scale or geometry.

---

#### 13 · Character  `character`
**0…1 · default 0.0 · Cloth**

| Assigns | The split of `target_peak` between `wind_speed` and `shed_gain` |
|---|---|

**What it does.** **Where the motion comes from**, not how much.

- **0** — all **gust wind**: a travelling velocity field driving the string
  through drag. Needs the wind to keep blowing.
- **1** — all **vortex shedding**: a limit cycle, so the string flaps with
  no gust wind at all.

**Functions.** → same path as Motion, plus `mix_boost` depends on it.

**Interactions.** At Character = 1 the air is still but the drag term is
still live, so the string feels aerodynamic damping from its own motion —
correct physics, and baked into the calibration. Default 0 reproduces the
previous wind-only behaviour exactly. Shedding is what lets a *drawn
deposit* keep moving, so this is the axis with the least accumulated
knowledge behind it.

---

#### 14 · Wind  `wind`
**0…1 · default 0.37 · Cloth**

| Assigns | `sim_->wind_phase_speed = (10 + 490·Wind)·(N/128)` nodes/s |
|---|---|

**What it does.** Gust **phase velocity** — how fast the flapping pattern
travels along the cloth. *Not* how hard it blows.

**Functions.** → **F3a**, advancing `site_phase_s` each tick.

**Interactions.** The `N/128` term keeps a setting meaning the same thing in
every precision mode; without it the pattern crossed a 512-node string 4×
slower than a 128-node one.

---

#### 15 · Flutter Hz  `flutter_hz`
**0.1…20 Hz · default 3.0 · Cloth**

| Assigns | Value |
|---|---|
| `sim_->shed_freq_hz` | the raw value |
| `sim_->shed_gain` | multiplied by `flutter_rate_compensation(f/f0)` |

**What it does.** Nominal wake shedding frequency. In Curves this **is** the
audible rate at which the EQ bands wobble, so it is set in Hz and tuned by
ear — the x-axis is log-frequency, not space, so no real Strouhal number
exists here.

**Mapping function.**

```
f0   = Tension / span,   span = (N−1)·(wall_hi − wall_lo)
comp = clamp( 0.213 / R(f/f0), 0.25, 8.0 )

R = peak-normalized response, log-interpolated over f/f0:
    ratio  0.15 0.30 0.50 0.75 1.00 1.50 2.50 4.00 7.00 12.0 25.0
    R      0.23 0.43 0.75 0.93 1.00 0.93 0.63 0.34 0.117 0.043 0.010
```

Raw, this dial swung amplitude **~130×**: the shedding drives the string's
fundamental resonantly near `f0`, and above that the string's own mass and
tension lowpass it away. The response shape is approximately **universal in
`f/f0`** (three of four tested configurations agree within 0.19), so one
table adapts to Tension, node count and the wall window automatically.
`0.213` is `R`
at the default operating point, so the default setting is **unchanged**.

**Functions.** → **F3c** (`ω = 2π·shed_freq_hz`), and `shed_gain` → **F3c**.

**Measured, before → after.** Full-range swing **133× → 6.8×**; across the
working band 0.3–8 Hz, **23× → 2.03×**. Residual at the extremes is the 8×
boost clamp binding, which is honest — very fast flutter genuinely is subtle.

---

#### 16 · Waves  `waves`
**0…1 · default 0.4 · Cloth**

| Assigns | `sim_->wave_density = 0.5 + 4.5·Waves` |
|---|---|

**What it does.** Spatial **density** of the gust pattern — how many waves
fit along the cloth. 0.5 = stretched, 5.0 = packed. Scales all eight gust
wavenumbers together, and the four wrinkle wavenumbers too, so the two stay
coherent.

**Functions.** → **F6** (triggers a table rebuild on change) → **F3a**, **F3b**.

**Range extended 2026-10-02** (user: "not noticing waves doing much"):
**0.5–3.0 → 0.5–5.0**. Measured spatial centroid against density:

| density | 0.5 | 1.5 | 3.0 | 5.0 | 8.0 | 12.0 |
|---|---|---|---|---|---|---|
| spatial centroid | 1.03 | 1.21 | 1.38 | 1.50 | 1.63 | 1.38 |
| peak | 0.70 | 0.74 | 0.60 | 0.41 | 0.30 | 0.21 |

The old top of 3.0 sat well short of the usable range, which reverses past
~8. Medium now reaches 5.0, High magnitude about 9.5.

**Honest limitation:** Waves is inherently a *modest* control — the string's
own dynamics dominate its spatial content, so the centroid spans only ~1.6×
over any usable range. Finer forcing also couples less efficiently, so peak
falls ~1.7× across the new range. That amplitude side-effect is **not
compensated**: Motion is the amplitude control and absorbs it.

---

#### 17 · Wrinkle  `wrinkle`
**0…1 · default 0 · Cloth**

| Assigns | `sim_->wrinkle = 3.9 · Wrinkle` |
|---|---|

**What it does.** Fine detail concentrated in the **last third** — how the
cloth resolves. Four extra gust components with wavenumbers {12, 17, 23, 31},
a `d^2.8` tip envelope, and phase velocity locked to the **tip wave speed**
`c·2^(taper/2)`.

**Boosted 2026-10-02 (+30% requested).** Amplitude `3.0 → 3.9`, and the tip
envelope exponent `4.0 → 2.8`, which widens the affected region: measured as
the span where the envelope exceeds 0.1, reach goes from **43.8% → 56.1%** of
the cloth, a **1.28×** increase. Measured tip/pole detail ratio in the
validation scenario rose **4.27× → 6.20×**.

**Why the resonance lock is essential.** At those wavenumbers the tension
restoring term `c²k²` is ~1944 against a stiffness of ~2 — short waves are
suppressed roughly **1000×** and can only be excited **on resonance**, and
that resonance is only a few percent wide. Two cheaper designs (warping the
gust coordinate; extra components at the wind's phase velocity) both did
essentially nothing for exactly this reason.

**Functions.** → **F6** (tables, with the anti-alias clamp) → **F3b**.

**Measured.** Tip/pole detail ratio 1.11 (off) → 1.45 at 1.5 → 1.82 at 3.0,
**with the pole end unchanged** (0.83 → 0.87).

**Interactions.**
- **No longer silenced by Wind.** Wrinkle was scaled by `wind_speed`, so
  gating the gust field off killed the wrinkles too. It now has its own
  amplitude reference (`sim_->wrinkle_speed`, fed the *ungated* gust speed) —
  it is a separate mechanism with its own power switch.
- **Still outside the excitation budget.** Unlike Motion and Character it
  adds on top: at Motion 1 it multiplies peak by **1.57×** at Character 0
  and 1.12× at Character 0.5, so near maximum it can push the EQ into the
  clamp. Outstanding work.
- **Resolution-dependent.** Effective at 512 nodes, marginal at 256, useless
  at 128. Finer wavenumber sets are *less* effective, not more — past a
  point the modes are too stiff even on resonance.

---

#### 18 · Direction  `direction`
**0/1 step 1 · default 0 · Cloth**

| Assigns | `sim_->set_wind_origin(value > 0.5 ? 1 : 0)` |
|---|---|

**What it does.** Which end the wind blows **from**. 0 = pole at the high
end (whip at the lows); 1 = pole at the low end (whip at the highs).

**Functions.** → **F4**, which mirrors envelope, taper, termination, tail
damping and gust sites, then **F5** re-aims the flutter phase ramp so travel
direction flips with the geometry.

**Interactions.** A two-position switch presented as a rotary.

---

#### 19 · Walls  `wall_lo` / `wall_hi`
**0…1 each · defaults 0.0 and 1.0 · dragged on the canvas, not knobs**

| Assigns | `sim_->set_walls(lo, hi)` |
|---|---|

**What it does.** Two hard boundaries as fractions of the node array,
defining the active window. Outside them the string is **held at rest**
(`u = v = 0` every tick), so any window anywhere along the spectrum can be
the cloth. Dragged by the posts on the canvas; the dead zones are shaded.
Click-to-position as well as drag.

**Replaced the old Cloth-length knob**, which could only shorten from the
free end and — measured — **did not actually work**:

| Length | nodes still moving |
|---|---|
| 0.80 | 98.2% |
| 0.60 | 99.0% |
| 0.20 | 64.5%, reaching the far end |

The "near-rigid tail" (`m_inv` 0.02 + 25× damping) was never a wall, because
the **DC and tilt blockers subtract a global mean and trend from every
node** and therefore kept shoving the dead region around. Both blockers now
operate on the **active window only**, which is what makes a wall hold.

**Functions.** → **F4** (geometry spans the wall window, with the pole at
whichever end the wind origin selects) → **F5**, **F6**. The hard clamp runs
in **F1** after the blockers, before `enforce_anchors`. The Flutter Hz `f0`
span follows the window too.

**Measured — walls hold exactly.** Peak displacement outside the window is
**0.00e+00** at every setting tested (windows of 100%, 80%, 40%, 30%, 10%).

**Interactions.** A narrower window captures less wind, so peak falls with
it (0.88 at full width → 0.20 at 40% → 0.01 at 10%). Motion compensates.
Defaults 0 and 1 are a true no-op, which is what keeps the frozen validation
scenarios bit-identical.

---

## §4 — Reverse Index: which knobs write each parameter

The entanglement map. A parameter written by more than one knob is where
knobs can fight.

| Engine parameter | Written by | Status |
|---|---|---|
| `wind_speed` | Motion, Character, Tension, Precision | budgeted — Motion is the only *amount* |
| `shed_gain` | Motion, Character, Flutter Hz, Tension, Walls, Precision | budgeted + rate-compensated |
| `wrinkle` | Wrinkle | **unbudgeted, adds on top** |
| `c2_` | Tension | — |
| `damping_` | Damping | — |
| `stiffness_` | Stiffness | — |
| `wind_phase_speed` | Wind, Precision | — |
| `wave_density` | Waves | — |
| `shed_freq_hz` | Flutter Hz | — |
| `wall_lo` / `wall_hi` | Walls (canvas) | hard; blockers windowed to match |
| `wind_origin` | Direction | — |
| `force_model`, anchors, taper | Physics | — |
| gain targets | Velocity, Tension, Range dB | — |
| `smooth_alpha_` (control one-pole) | Smoothing, Precision (tick rate) | — |
| `bank_.steady_amount_` | Steady | — |
| `leveler_.amount_` | Leveler | — |
| `n_nodes_`, `sim_divisor_` | Precision | — |
| *(all of the above)* | the per-knob **power / magnitude** modifiers | §2b |

**Tension reaches five places** and is the most entangled remaining control,
though each consumer normalizes against it deliberately.

---

## §5 — Known Defects and Tuning Targets

1. **Wrinkle is outside the excitation budget** — 1.57× peak multiplier at
   Character 0. The last piece of the Motion/Character work.
2. **Range dB default 18** gives a 36 dB peak-to-trough span. Probably too
   hot; set it before tuning physics.
3. **Stiffness is weaker than its range** now that DC and tilt are blocked.
   Needs rescaling or a redefined job.
4. ~~**Velocity is unexplored**~~ — **fixed 2026-10-02.** Was contributing
   3.4% at full; now a position/velocity blend with the constant corrected.
5. **Physics modes 2 and 3 are thin** — knob values plus, for Gust, a static
   field. Candidates for becoming presets, freeing Physics to be a real mode
   switch.
6. **Three stepped parameters are rotaries** (Physics, Precision, Direction).

10. ~~**Steady's ducking was hidden from the curve**~~ — **fixed 2026-10-02**
    (up to 4.52 dB of real gain reduction was invisible; mono detector now,
    `gains()` is Steady-inclusive).
11. ~~**Range dB leaked into the drawn curve**~~ — **fixed 2026-10-02**
    (smoother moved onto the control signal; 0.298 → 4.95e-14).
12. ~~**Stereo smoothed at half the set time**~~ — **fixed 2026-10-02**
    (the gain advanced once per channel instead of once per frame).
13. **Smoothing's default is now un-tuned.** Fixing the double-step made
    stereo 2× slower at the same setting. 30 ms is inherited, not chosen.
7. **Precision rebuild allocates on the audio thread.**
8. **Per-node Q is unimplemented.** `Q = 1.0` fixed for every band, despite
   per-node Q being a recorded architectural decision.
9. **96 kHz halves the headroom** — 512 nodes ≈ 85 % of one core.

---

## §6 — Invariants

Two rules that constrain all future work:

**WYSIWYG.** The bright curve is drawn from `FilterBank::gains()` with the
same ±1 clamp the audio path uses — never from the sim. Any amplitude
mechanism must keep the string inside ±1 in normal use, or the EQ flat-tops.

**Channel coherence.** One instance is one string: no process inside a
single instance may treat L and R differently. Stereo interest comes from
running separate *instances* per channel — the sandbox design — so internal
divergence is a dynamic image-mover and therefore a defect. Driving a
*different spectrum* (stereo width alone, or alongside EQ) is a wanted
feature to be built deliberately, not something that should fall out of
per-channel followers by accident.

**Bit-exact cross-validation.** The C++ engine is validated against the
Python spec cell by cell: **10 scenarios, 73,856 cells, 0 mismatches**. Every
physics change must land in both with identical arithmetic order, and the
pre-existing scenario CSVs must stay byte-identical.

```
python prototype/run_scenarios.py        # Python spec
build/Release/scenarios.exe prototype    # C++ engine
build/Release/bench.exe 48000            # per-core cost
build/Release/gain_test.exe              # Range invariance, dB symmetry, Steady
```
