# Curves — Technical Specification **v0.0.2**

**Status: current.** Supersedes `SPEC-0.0.1.md`, which is kept for history.

An exhaustive knob-by-knob reference covering all **30 parameters** in
**28 entries** (the two Walls share one, as do the two Randomisers). For every control it states:

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

### Scope note — two axes, and what this document covers

Curves is a **physics sandbox for spectral control**: a physics-driven string
whose shape is read off as a parameter across the frequency axis. It
generalises two ways, and the control set below is divided by it.

| Axis | Today | Where it lives below |
|---|---|---|
| **What drives the string** (physics mode) | Free, **Flagpole** (deep), Gust, Elastic | Physics selects it. The eight **CLOTH & WIND** controls belong to *Flagpole specifically* and are inert elsewhere. |
| **What the string drives** (subscriber) | EQ gain only | **F7**/**F8**. Stereo width, per-node Q and cutoff modulation are planned subscribers on the same control signal. |

So: **GLOBAL** and **STRING** are generic; **CLOTH & WIND** is one mode's
panel. The cloth is not the product — it is the first mode developed in
depth. See `ORCHESTRATION.md` §1.

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
| **B** Node → gain map | `processBlock` | once per node per chunk |
| **C** Filter bank | `FilterBank` | every sample, per band |
| **D** Output stage | `OutputLeveler` | every sample |
| **E** Structure | `rebuild` / `apply_physics` | on change only |

Stage **A** is swapped by the **physics mode**; stage **B** is the *first and
only built subscriber*. The boundary between them is the whole architecture:
`StringSim` publishes nodes and knows nothing about filters, so a second
subscriber (stereo width, per-node Q) attaches at **B** without touching
**A**, and a new physics mode replaces **A** without touching **B**.

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

**(d) Mid-band ripples.** Four more components filling the gap between the
gust sites (1.7–6 waves across the cloth) and wrinkle (25–64):

```
W_i += wind_speed · mid_ripple · (1/4) Σ_j [ sin(μ_j)·MC_j,i − cos(μ_j)·MS_j,i ]
μ_j += mid_k_j · c_mid · dt,  c_mid = c · 2^(taper·mid_lock_d/2) · mid_travel
mid_k_j = 2π · mid_waves · kMidK_j / span,   kMidK = {0.72, 1.0, 1.37, 1.9}
```

Tables bake in a `sin(π·dd)^1.4` bump envelope whose peak is warped to sit at
`mid_lock_d = 0.65`, plus the **Flap** term `+ flap·d²·(1 − bump)` which fills
the envelope back in toward the free end — which is exactly where the `c·2^(taper/4)` phase lock is
resonant. Wrinkle sits at the tip, this sits in the middle, the gust spans
the lot. `mid_waves` is an **absolute** wave count, deliberately *not* scaled
by `wave_density`, so Waves keeps owning the gust pattern alone. Skipped
entirely when `mid_ripple == 0`.

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

**The two drawn lines.** The **bright** line is `FilterBank::gains()` — the
gains the audio is going through, and therefore what you hear. The **faint**
line is `sim_->positions()`, raw `u`: a *control signal*, not the EQ. Four
things separate them, measured at 512 nodes:

| source of difference | max | rms |
|---|---|---|
| Smoothing 2 ms (off) | 0.016 | 0.003 |
| Smoothing 15 ms (default) | 0.081 | 0.013 |
| Smoothing 100 ms (max) | 0.190 | 0.033 |
| Steady 0.35, loud input | 0.036 | 0.006 |
| **Velocity 0.5** | **0.400** | 0.066 |
| **Velocity 1.0** | **0.812** | 0.133 |

Only **Velocity** makes the faint line a genuinely *different shape* rather
than a lagged copy — above 0 the bright line follows a blend of position and
velocity while the faint line stays pure position. Steady's contribution is
content-dependent, so it moves with the material rather than the knobs.

### F8 · `FilterBank::process_frame(l, r, stereo)`

The bank holds **no time constant of its own**. Given a block target it
precomputes one geometric step per band, `r_b = (G_b / g_b)^(1/n)`, then per
**sample frame** — once, not once per channel:

**Runs in MID/SIDE** since 2026-10-06. Verified identical to the L/R form
at **3.3e-16** while mid and side share a gain, so the move was free; the
SVF count is unchanged and the M/S conversion is 4 ops per *frame* rather
than per band, which is what makes stereo nearly free.

```
m, s  = (l+r)/2, (l−r)/2                      once per FRAME
g_b  *= r_b                                   mid gain, linear in dB
gs_b *= rs_b                                  side gain  ─┐
pan_b += pan_step_b                           placement ──┘ spatial only
v1m   = bandpass(m)                           TPT SVF, Q = 1.0 fixed
v1s   = bandpass(s)
bigm  = g_b − 1 ;  bigs = gs_b − 1
                                              ── Steady, if amount > 0 ──
env_b += (|v1m| > env_b ? atk : rel)·(|v1m| − env_b)   5 ms / 150 ms
if env_b > 0.25:  bigm, bigs ÷= 1 + Steady·(env_b/0.25 − 1)
g_eff_b, gs_eff_b = 1 + bigm, 1 + bigs        ← display candidates
ym   += v1m · bigm
ys   += v1s · bigs + v1m · pan_b
l, r  = ym + ys, ym − ys                      once per FRAME
```

Three per-band coefficients: **mid gain** (the EQ), **side gain** (width is
their difference in dB) and **mid→side cross-feed** (placement). All three
come from the control surface and are drawable, which is what **image
provenance** (§6) requires.

Output is `x + Σ_b v1_b·big_b` — a dry path plus summed peaking bands. This
can **boost or cut** a band but cannot **move** one. Steady is
**feedforward** (the follower reads the input-side bandpass), so it cannot
oscillate, and it ducks **mid and side by the same factor** — a
content-driven width change is exactly what image provenance forbids.

> **⚠ The bands OVERLAP and their contributions ADD.** See §5: the effective
> gain is `1 + K·(g−1)` with `K ≈ 0.152·N`, so every per-band depth in this
> document is **nominal, not actual**.

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


### F8b · Which curve is drawn

`publish_display()` picks the subscriber that is **actually carrying the
shape**, in this order. Enumerating these one at a time is how the curve
went missing twice, so the set is exhaustive by construction:

| # | condition | source | axis |
|---|---|---|---|
| 0 | EQ powered on | `gains()` (mid, Steady-inclusive) | ± EQ dB |
| 1 | else, Width > 0 | `side_gains()` (Steady-inclusive) | ± dB width |
| 2 | else, Place > 0 | `pans()` (linear, signed, **not** ducked) | ± % place |
| 3 | else | — | "no subscriber on" |

Row 3 is the one case where a flat curve is correct: nothing is applied, so
drawing a shape would itself be the WYSIWYG violation. It is labelled rather
than left ambiguous, because a silently vanishing curve is what the bug
looked like.

Mix is folded in exactly on every path (`1 + Mix·(g−1)`).

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
which is why that panel disables itself.

**This is meant to be the sandbox axis.** Modes 2 and 3 are thin — three knob
values plus, for Gust, a static field — so Physics is not yet a real switch
between *different simulations*, which is the intent. Candidates that fit the
same node-bus: fluid/current flow, granular or particle impacts, magnetic
repulsion between nodes, coupled or multi-string systems. Each would bring
its own control panel; the editor currently hard-codes a Flagpole-shaped one,
so a new mode is UI work as well as physics.

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

#### 3 · EQ dB  `drive`
**3…24 dB · default 18 · SUBSCRIBERS panel** (was GLOBAL until 2026-10-06)

| Assigns | `max_gain_db_` |
|---|---|

**What it does.** The ± dB span the normalized string maps onto. A node at
+1 becomes `+RangeDB`.

**Functions.** → **F7**. Also the **display normalization**, so it rescales
curve and audio identically and cannot break WYSIWYG.

**Interactions.** Master depth for the whole effect. At 18 dB a full-scale
swing is a **36 dB peak-to-trough** EQ — set this before tuning anything
physical, or you will tune the physics around a too-hot master.

**Power LED gates the EQ entirely** (`kPowerGate`, changed 2026-10-06). Off
means the mid gain is flat — not "3 dB" — so the EQ and the spatial
subscriber can genuinely run independently.

**It does not touch the physics, and it does not move the picture.** It has
one writer and two readers (the gain map and the display normalization),
never reaches `sim_`, and since 2026-10-02 all smoothing state lives
upstream of it, so the drawn curve is **exactly** invariant to it: measured
deviation **4.95e-14** of full scale across a 16× Range change, moving or
still. Before that fix the same measurement gave **0.298**. The canvas now
labels its dB axis from the *effective* Range (post magnitude modifier).

---

#### 4 · Smoothing  `smoothing`
**2…100 ms · default 15 · Global**

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

**Default 30 → 15 ms (2026-10-02).** Fixing the double-step doubled the real
smoothing at any setting, so 30 ms lagged twice as far as that number used to;
15 ms restores the previous stereo feel.

**This lag is the only thing separating the two drawn lines** — as of
2026-10-02 it is also the only thing setting it. The gain targets used to be
aimed **once per host block**, from the control signal as it stood at the
*start* of the block, with the bank arriving only at the *end*: roughly a
block and a half of lag on top of this knob, growing with the buffer.
Measured max |sim − drawn| at Smoothing 2 ms (the knob fully off):

| buffer | 32 | 64 | 128 | 256 | 512 |
|---|---|---|---|---|---|
| before | 0.0164 | 0.0206 | 0.0288 | 0.0451 | **0.0771** |
| after | 0.0164 | 0.0164 | 0.0164 | 0.0164 | **0.0164** |

The ramp is re-aimed every **32 samples** (`kGainChunk`) instead of once per
block — one `exp2` per node and one `pow` per band per chunk, measured at
**1.49% of one core** at 512 nodes. Latency is now constant and
buffer-independent. Before the fix the plugin audibly **sounded different at
different buffer sizes**.

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

#### 7 · Mix  `mix`
**0…1 · default 1.0 · Global**

| Assigns | `mix_amount_`, applied in the per-sample loop |
|---|---|

**What it does.** Dry/wet. `1` is fully processed, `0` is bypass.

```
out = dry + Mix·(wet − dry)      then → F9
```

**Interactions.** Sits **after the bank and before the Leveler**, so the
Leveler trims what you actually hear rather than the wet path alone. Its
minimum is genuinely neutral, so it takes the standard `kPowerToMin` power
mode.

**Power LED only — no magnitude.** "Half travel" and "double travel" are
meaningless on a 0…100% blend, and a High setting would put 100% out of
reach. `has_magnitude()` is per knob and returns false here; `knob()` then
passes the raw value through.

**The curve follows it, exactly** (fixed 2026-10-02 — it did not, for the
few hours between Mix landing and this being noticed, which was a live
WYSIWYG violation). The bank emits `x + Σ v1_b·big_b` and Mix scales only
the added part, so the effective per-band gain is **exactly**
`1 + Mix·(g − 1)`. For a band sitting at +18 dB:

| Mix | 1.00 | 0.75 | 0.50 | 0.25 | 0.00 |
|---|---|---|---|---|---|
| applied | 18.00 dB | 15.86 | 13.01 | 8.74 | 0.00 |

---

#### 8 · Test Src  `test`
**0…5 step 1 · default 0 (off) · Global**

| Assigns | `test_src`, replacing the input when non-zero |
|---|---|

Six positions: **off / pink / white / brown / tone / pulse**. The knob's
label names the live one. Chosen to probe different things about a *moving*
EQ rather than just to make noise:

| source | what it is for |
|---|---|
| **pink** | the reference, perceptually flat (Paul Kellet) |
| **white** | bright — what the top octaves are doing |
| **brown** | low-heavy — the bass end moving |
| **tone** | sustained 24-harmonic stack near A1, channels detuned 55 / 55.37 Hz, so the EQ reads as **timbre and formant movement** on a pitched sound |
| **pulse** | ~7 clicks/sec, short decaying burst — exposes **ringing, smoothing and transient** behaviour that steady noise hides |

**Levels are measured, not eyeballed** — rms / peak: white 0.202 / 0.350,
brown 0.180 / 0.719, tone 0.197 / 0.394, pulse 0.105 / 0.795 (sparse by
design; its peak is the point). Brown was first written at gain 3.2, which
measures **rms 0.806 / peak 3.968** — four times full scale. A leaky
integrator with `a = 0.996` driven by `0.04·uniform` has `sigma = 0.2585`,
so 0.70 is the correct gain.

No modifiers — it is a switch.

---

### STRING — applies in every Physics mode

---

#### 9 · Damping  `damping`
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

#### 10 · Stiffness  `stiffness`
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

#### 11 · Tension  `tension`
**50…600 nodes/s · default 300 · String**

| Assigns | Used by |
|---|---|
| `sim_->c2_ = min(c, 1/dt)²` | **F2** wave term |
| `tension_norm = (c/300)^kTensionP[prec]` | wind speed |
| `f0 = c / span` | Flutter Hz compensation |
| `c_tip = c·2^(taper/2)` | Wrinkle resonance (**F3b**) |
| `v_i / Tension` | Velocity coupling (**F7**) |

**What it does.** Wave speed. Sets how fast disturbances travel along the
spectrum and how strongly the string resists curvature. Clamped to the CFL
limit `1/dt`.

**Amplitude compensation, fixed 2026-10-02** (user: "Tension compresses the
waveform physics amplitude-wise a bit more than I would prefer"). The
`tension_norm` term exists so a given Motion means the same amplitude at any
Tension. It was a flat `^0.6` and under-corrected badly below 512 nodes —
measured peak, Tension 300 → 600, Motion 0.45, Character 0:

| nodes | before | after | exponent |
|---|---|---|---|
| 128 | 0.56× | **0.79×** | 0.85 |
| 256 | 0.61× | **0.79×** | 0.80 |
| 512 | 0.94× | 0.94× | 0.60 (unchanged — was not broken) |

Whole-dial spread at 128 nodes: **3.15× → 1.46×**.

The exponent is derivable: at steady state drag balances tension,
`ρCd·W² ≈ c²k²·u`, so `u` is Tension-free only when `W ∝ c` — exponent 1.0.
Measurement lands just under it because a faster-moving string sees less
relative velocity.

**Why per-precision.** Peak against Tension is a **hump**, not a slope: it
maxes near a fixed `f0 = c/span`, so the hump sits at a different Tension in
each mode and no single exponent flattens the whole dial. The shipped values
flatten the **upper half**, where "turning Tension up makes it quieter" is
felt. A complete fix would be a measured response table in `f_gust/f0`,
exactly as **Flutter Hz** already does — noted, not built.

`(c/300)^p = 1` at Tension 300 for any `p`, so the Motion calibration at the
default Tension is untouched by this.

**Still uncompensated: the shedding path.** At Character 1 peak falls
**80.7×** across the Tension dial (0.92 at 50 → 0.011 at 600). `shed_gain`
carries only the Flutter rate compensation, which is peak-*normalized* and so
carries no information about the string stiffening as `c` rises; adding an
impedance term `(c/300)^2` only gets it to 2.95×, with an unexplained dip at
Tension 300. Deferred to the Character work.

**Interactions.** **The most entangled knob in the plugin** — it reaches
five separate places. Each is deliberately normalized against it (the wind
speed term, the flutter `f0`, the velocity divisor), so the *other* knobs
keep their meaning as Tension moves; but Tension itself changes travel rate,
curvature response, wrinkle tuning and settling together. Tune it early.

---

#### 12 · Velocity  `velocity`
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

#### 13 · Motion  `motion`
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

#### 14 · Character  `character`
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

#### 15 · Wind  `wind`
**0…1 · default 0.50 · Cloth**

| Assigns | `sim_->wind_phase_speed = 0.12·25^Wind · Tension` nodes/s |
|---|---|

**What it does.** Gust **phase velocity** — how fast the flapping pattern
travels along the cloth — expressed as a **multiple of the wave speed**, so
the dial means `f_gust/f0 = 0.12 … 3.0`. Default 0.50 gives ratio 0.60.

**Functions.** → **F3a**, advancing `site_phase_s` each tick.

**Re-ranged 2026-10-02** (user: "wind is not really affecting the string
anymore … no difference at high levels or low levels"). It was not inert, it
was mis-ranged. What you see depends on whether the string can **follow** the
gust, which is a question about `f_gust/f0` — and the old
`(10+490·Wind)·(N/128)` held `f_gust` constant across precision while
`f0 = c/span` falls as `1/N`:

| | 128 nodes | 512 nodes |
|---|---|---|
| `f_gust/f0` at the old default | 0.64 | **2.54** |
| `f_gust/f0` at the dial top | 1.65 | **6.67** |

At 512 nodes the whole upper dial sat beyond the string's ability to respond.
Measured peak across the dial, before: **0.21 / 0.45 / 0.28 / 0.24 / 0.18** —
both *ends* land in the same place. After: **0.14 / 0.40 / 0.44 / 0.51 /
0.32**.

Normalizing to the wave speed makes the knob precision- **and**
Tension-independent by construction, exactly as **Flutter Hz** is normalized
by `f0`.

**Damping is not a lever here** — a 10× damping change moves the observed
motion frequency by 4%, because drag dominates dissipation (ruled out by
measurement before the re-range).

**Mid-band ripples ride this knob (added 2026-10-02).** Wind sets two things
about them, and nothing about their level:

- **size** — `mid_waves = 6·(20/6)^Wind`, i.e. 6 → 20 waves across the cloth;
- **travel speed** — `mid_travel = 0.55·(1.25/0.55)^Wind`, a 2.3× range.

The amount follows Motion through `wind_speed`, and Wind's power LED gates
them because they are gusts. See **F3d**.

**Why travel is limited to 2.3×.** The ripples are *resonant* — that is the
only reason they are excitable at all — so their natural travel speed is the
string's own wave speed, and detuning away from it costs amplitude. Measured
normalized response against the travel multiplier:

| travel | 0.35 | 0.50 | 0.65 | 0.80 | **1.00** | 1.20 | 1.45 | 1.80 | 2.40 |
|---|---|---|---|---|---|---|---|---|---|
| R | 0.286 | 0.346 | 0.476 | 0.794 | **1.000** | 0.593 | 0.286 | 0.166 | 0.114 |

Asymmetric: a *slower* forcing still pushes the string quasi-statically,
while a faster one outruns it. `mid_resonance_compensation()` divides R out
so Wind stays a speed control.

**The compensation clamp is tight, deliberately.** That table measures
**detail** (curvature), but a slow-travelling forcing produces large
*displacement* while producing little detail — so compensating detail fully
inflated peak to **1.98×** the no-mid baseline at Wind 0. `[0.80, 1.25]`
holds peak to **≤ 1.28×** across the dial while still roughly doubling the
low-Wind detail against no compensation.

**Honest limitation.** Amplitude across the dial is still *non-monotonic*, and
the per-node motion frequency does not rise cleanly with the knob at 512
nodes — the string rings in its own modes rather than tracking the gust. The
spectral-centroid metric used for "motion frequency" is **not validated** and
should not be trusted for fine judgements; the peak figures are solid.

---

#### 16 · Flutter Hz  `flutter_hz`
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

#### 17 · Waves  `waves`
**0…1 · default 0.24 · Cloth**

| Assigns | `sim_->wave_density = 0.5 + 7.5·Waves`, plus a gust-dial compensation |
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

**Amplitude-compensated and extended 2026-10-02.** Finer gust patterns
couple to the string less efficiently, so the knob used to *drain* it —
measured rms |u| against density, 40 s per point:

| density | 0.5 | 1.0 | **2.3** | 3.5 | 5.0 | 7.0 | 9.5 | 18.0 |
|---|---|---|---|---|---|---|---|---|
| rms, before | 0.139 | 0.126 | **0.090** | 0.057 | 0.039 | 0.032 | 0.029 | 0.027 |

**2.3x quieter** from the default to the old top of the dial, which is why
it needed turning "way, way up" before the texture change read as anything.
Worse, the 6–12 band stayed *flat* across the whole sweep — what held it up
was the **mid band** (deliberately independent of `wave_density`) while the
gust contribution died away underneath it.

`wave_density_compensation()` divides the loss out, as a multiplier on the
wind dial (`peak ~ dial^kWindP`, so a response ratio `R` needs
`dial x R^(1/kWindP)`). After:

| density | 0.5 | **2.3** | 5.0 | 8.0 |
|---|---|---|---|---|
| rms | 0.088 | **0.090** | 0.087 | 0.080 |
| 1–2 band (blooms) | 0.0488 | 0.0482 | 0.0363 | 0.0301 |
| 6–12 band | 0.0118 | 0.0163 | 0.0318 | **0.0373** |

Level flat to **1.14x** across the dial (was 4.3x), and the 6–12 band now
**triples**: Waves moves energy from the big blooms up into mid detail at
roughly constant level, instead of just fading everything out.

**Range 5.0 → 8.0.** Measured, every band flattens past ~7, so 8 is where
the knob genuinely stops doing anything rather than an arbitrary stop.
Default knob **0.40 → 0.24**, keeping the shipped density at 2.3 so the stock
character is unchanged.

**Where the big slow blooms come from.** Not Waves — the **eight gust sites
beating**. Their temporal multipliers are deliberately incommensurate, so at
the default operating point they run at 0.265–0.786 Hz and the pairwise
differences land at 11.6, 13.4, 13.9, 19.0, 19.6, 25.2, 28.4 and 88.9 s.
Measured over 150 s the signed low-mode amplitude swings −0.108 … +0.069
with large excursions every 10–25 s and never repeats — quasi-periodic, not
random, and it goes *both* ways because it is a sum of sinusoids rather than
a rectified envelope. The two lowest-wavenumber sites (1.66 and 1.91 waves
across the cloth) are what put that beat into the 1–2 band, which is why
raising Waves phases the blooms out: it moves those sites up out of it.

**Honest limitation:** Waves is inherently a *modest* control — the string's
own dynamics dominate its spatial content, so the centroid spans only ~1.6×
over any usable range. Finer forcing also couples less efficiently, so peak
falls ~1.7× across the new range. That amplitude side-effect is **not
compensated**: Motion is the amplitude control and absorbs it.

---

#### 18 · Wrinkle  `wrinkle`
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

#### 19 · Flap  `flap`
**0…1 · default 0.45 · Cloth**

| Assigns | `sim_->mid_flap` |
|---|---|

**What it does.** How hard the **mid-band ripples drive the free end**.

**Why it is needed.** The mid-band envelope is a bump whose peak is warped to
`mid_lock_d`, and it returns to **exactly zero at `d = 1`**:

| d | 0.65 | 0.80 | 0.90 | 0.95 | 0.99 | 1.00 |
|---|---|---|---|---|---|---|
| envelope | 1.000 | 0.709 | 0.311 | 0.122 | 0.013 | **0.000** |

So the ripples shrinking toward the end was never damping — **they were
never forced there** (owner's read: "it may be the fact that is the end of
the simulation"). Flap adds `flap·d²·(1 − bump)`, which fills the tip in
*without touching the middle*. `flap = 0` is exactly the old envelope.

**Measured** at the shipped shape, Wind 0.5 — note `f2` barely moves:

| flap | f2 | f3 | tip | tip/f3 | peak |
|---|---|---|---|---|---|
| 0.00 | 10454 | 9838 | 3649 | 0.37 | 1.11× |
| **0.45** | 10755 | 10029 | 4303 | 0.43 | **1.27×** |
| 1.00 | 11139 | 10317 | 6523 | 0.63 | 1.61× |

**Honest limit.** The tip is **off-resonance**: local wave speed there is
`c·2^(taper/2) = 2.00c` against a lock at `c·2^(taper·0.65/2) = 1.57c`, a
**1.27× detune**, so it couples less per unit forcing than the middle no
matter how hard it is driven — which is why even `flap = 1` leaves the tip
below the `f3` peak. Shifting the lock toward the tip with Flap would buy
more whip at the cost of moving the whole band's centre. Not done.

**Unbudgeted, like Wrinkle.** Flap adds *on top of* the Motion budget
(1.45× peak across its range) rather than inside it. Harmless at default
Motion — peak 0.55 against the ±1 clamp — but it will clip near Motion 1.

---

#### 20 · Randomisers  `wind_rnd` / `flutter_rnd`
**0…1 each · default 0 · small dials on the top-right of Wind and Flutter Hz**

| Assigns | `sim_->gust_spread` / `sim_->shed_spread`, plus their seeds |
|---|---|

**Drag** sets how much; a plain **click re-rolls the seed**. 0 is the fixed,
perfectly regular behaviour.

**What they perturb.** Both mechanisms are built from **fixed constant
sets** — `kSiteMult_`/`kSiteK_` for the gust, one shared frequency for the
wake — which is exactly why they are repeatable and, given long enough,
regular. A seed perturbs those constants, so a new seed is different
**weather**, not a different amount. At spread 0 every perturbed value
equals its constant, so the frozen baselines are untouched.

**Wind** re-rolls the eight gust sites' rate and wavenumber, which re-rolls
the beat structure behind the big slow blooms. Peak across seeds at spread
0.5: 0.417 / 0.578 / 0.685 — different weather, no runaway.

**Flutter perturbs amplitude and a STATIC phase offset, never frequency**,
and the reason is load-bearing:

| spread | 0.00 | 0.25 | 0.50 | 0.75 | 1.00 |
|---|---|---|---|---|---|
| frequency spread — direction | +0.269 | **−0.180** | **−0.273** | **−0.267** | **−0.251** |
| frequency spread — peak | 0.602 | 0.156 | 0.204 | 0.252 | 0.280 |
| **shipped** — direction | +0.269 | **+0.281** | **+0.288** | **+0.289** | **+0.286** |
| **shipped** — peak | 0.602 | 0.579 | 0.555 | 0.529 | 0.501 |

Outward travel comes from a seeded phase ramp that survives only because
the oscillators are **identical** and van der Pol is phase-neutral. Give
them different frequencies and the ramp steepens with time until the
direction **inverts** — the same finding already on record from
2026-10-01: *a frequency gradient cannot set a travel direction*. Making
the spread spatially smooth did not help (−0.43); the gradient is the
problem, not the roughness. A **static** phase offset persists instead of
accumulating, so it degrades direction gracefully and never flips it.

**Seeds are not AudioParameters** — a seed is not a continuous quantity to
automate — but they ride in the state tree, so they save with the preset.

**Functions.** → `rebuild_randomisation()`, then **F3a** (gusts) and
**F3c** (wake). Build-time only; measured cost at 512 nodes is unchanged.

---

#### 21 · Direction  `direction`
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

#### 22 · Walls  `wall_lo` / `wall_hi`
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

**Grab ergonomics, fixed 2026-10-02.** Reported as "we lost the ability to
drag the left side wall in". The walls were never broken — they were
*unhittable*. `wall_at()` rejected anything outside `plot_bounds` expanded
only **vertically**, so a wall parked at an edge sits exactly on the
boundary with half its 12 px grab zone outside the test, and the dB axis
labels added the same day sit on precisely that spot. Verified by driving
the UI: dragging from inside the plot worked throughout, from just outside
did nothing. Now expanded horizontally too, reach **12 → 18 px**, and parked
posts dim to 0.60 instead of 0.35 so they stay visible against the dB
numbers.

**Measured — walls hold exactly.** Peak displacement outside the window is
**0.00e+00** at every setting tested (windows of 100%, 80%, 40%, 30%, 10%).

**Interactions.** A narrower window captures less wind, so peak falls with
it (0.88 at full width → 0.20 at 40% → 0.01 at 10%). Motion compensates.
Defaults 0 and 1 are a true no-op, which is what keeps the frozen validation
scenarios bit-identical.

---

### SUBSCRIBERS — what the one control signal drives

Each subscriber reads the **same** smoothed control signal and carries its
own depth and limits. EQ dB (**#3**) is the first; the spatial pair is the
second. Per-node Q and cutoff modulation slot in here without a redesign.

---

#### 23 · Width  `width`
**0…24 dB · default 0 · Subscribers**

| Assigns | side gain target, `exp2((d·EQdB + d·Width·w)/6.0206)` |
|---|---|

**What it does.** How far the **side** gain departs from the mid at full
deflection — in the same dB unit as EQ dB, because that is literally what it
is. The width ratio is `2^(d·Width/6.0206)`.

Since mid and side are the same shape at different depths, **one curve is
honest for both**, which is what the dual-labelled axis relies on.

**Measured.** Tracks the EQ's own behaviour to **0.18%** at the reference
point. Zero is exactly neutral, and the stage is skipped outright when both
Width and Place are zero, so it costs nothing unused.

**Needs stereo input.** Side is zero on a mono source, so width does
nothing there. A decorrelator is the only real fix — deferred, see §5.

---

#### 24 · Place  `pan_depth`
**0…1 · default 0 · Subscribers**

| Assigns | `pan` target, `d · Place · w`; the bank adds `pan · v1m` to the side |
|---|---|

**What it does.** Per-band **placement**. Unlike width, it works on a mono
source, and it is **mono-safe**: measured `rms(L+R)` is unchanged to
**1.000×** across the whole range, so it is a balance change rather than a
cancellation.

> **⛔ BLOCKED — not usable at its stated range.** Band overlap (§5) means
> ~0.15·N bands contribute at any frequency and their pan contributions add,
> so a hard pan arrives at roughly `Place = 0.014` at 512 nodes. The whole
> visible travel is past the end of its useful range. Width and EQ are
> unaffected because they inherit the factor *equally* and stay consistent
> with each other; placement has an **absolute** meaning and does not.

---

#### 25 · Low Hz  `stereo_lo`
**20…2000 Hz · default 120 · Subscribers**

#### 26 · High Hz  `stereo_hi`
**1000…18000 Hz · default 12000 · Subscribers**

| Assigns | `spatial_w_[i]`, a per-band taper on the spatial **deviation** |
|---|---|

**What they do.** Damp width and placement outside the window, leaving the
EQ alone. **A taper on the deviation, not a filter on the audio**: outside
it, width goes to 1 and pan to 0, i.e. those bands are simply left as they
were. One octave of smoothstep at each edge, so the drawn curve stays
continuous.

Widening the bass is almost always wrong and the log axis puts bass at the
left end, which is why this exists. Cached — it only rebuilds when the cuts
or the band map move.

---

#### 27 · Band Q  `q_mode`
**0…1 step 1 · default 0 (Flat) · Subscribers**

| Assigns | `FilterBank::set_q` and `set_overlap_correction` |
|---|---|

**What it does.** Chooses how wide each band is, and therefore how much the
bands add up. A property of the **bank**, so it applies to every subscriber
at once — EQ, width and placement all inherit it.

- **Flat** — `Q = 1`, no overlap correction. Bands are ~1.4 octaves wide
  against a `9.23/N`-octave spacing. This is the historical bank
  **bit-for-bit**, kept so the two can be A/B'd by ear.
- **Matched** — band width equals band spacing (`Q = sqrt(2^BW)/(2^BW − 1)`,
  giving 19.85 / 39.86 / 79.88 at 128 / 256 / 512 nodes) plus the analytic
  `1/K` overlap correction.

**Measured.** See §5 B1 for the full set. Flat delivers ~24× (128 nodes) to
~96× (512) the asked depth, passes **0.00 dB** of any spectral detail
tested, peaks at **17–67× full scale**, inverts cuts, and leaves the drawn
curve wrong by **31–43 dB**. Matched delivers the asked depth to ~0.05 dB on
boosts at both node counts, never inverts, peaks at 1.5×, and cuts the
WYSIWYG error to 6.8 dB — all of which is in the cuts, for the quadrature
reason in §5.

**The title bar says which mode is live** (`SUBSCRIBERS (Matched Q)`),
because a 0/1 rotary cannot carry something this consequential.

**Two visible consequences in Flat, both expected** now that the curve draws
the true response (O10):

- **The bright curve goes blunt.** At `Q = 1` the bank really does smear
  over ~1.4 octaves, so the curve loses nearly all the detail the faint
  string line has. Switching to Matched makes it track the string again.
  That contrast *is* the resolution finding, on screen.
- **The dB axis widens.** It is labelled from `1 + K·(g−1)` when the
  correction is off, so Range 18 reads about ±44 dB at 128 nodes — which is
  what the bank is actually doing. Otherwise the honest curve would simply
  peg at the clamp and the default would look broken rather than merely be
  wrong. In Matched the correction has divided `K` out and the axis equals
  the knob again. It is static — it moves only with Range, node count and Q
  — so it does not jitter with the string.

> **Not yet the default.** Every sound tuned so far was tuned against Flat,
> and the measurements say nothing about how Matched *sounds*. See P7, and
> expect EQ dB to need raising well above its current 3.74.

---

#### 28 · Resonance  `resonance`
**0…1 · default 0 · Subscribers**

| Assigns | `Q = matched_q(N) · 2^(3·Resonance)` — ×1 to ×8 |
|---|---|

**What it does.** Narrows the bands past matched. Two things follow, and
both are the point: each band **rings longer**, and the bands stop filling
the gaps between one another, so the response becomes a **comb**.

**Only meaningful with `Band Q = Matched`** — it is ignored in Flat.

**Measured.** `T60 = 6.91·Q/(π·f)`, confirmed at every point tested. At 128
nodes: 754 ms at 58 Hz / 58 ms at 754 Hz / 7 ms at 9.4 kHz at Resonance 0,
rising to 3.0 s / 232 ms / 25 ms at Resonance 0.67. Ring time falls as
`1/f` because constant Q is constant ring in *cycles*, so the texture is
strongly bass-weighted. Inter-band ripple is the cost: 0.93 dB at 0,
3.24 dB at 0.33, 5.73 dB at 1.0. Output level *falls* as Resonance rises
(peak 1.52 → 0.75), since narrow bands pass less broadband energy, and
nothing went non-finite up to Q = 639.

**What it is for.** A slowly-moving physics curve read across a bank of
ringing resonators is the same device as crossfading note-sample impulse
responses — the comb is drawn by the string instead of baked into a sample,
the decay is a knob, and it stays drawable. Resonance is the knob that buys
that texture; at 0 the bank is a clean EQ.

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
| `wind_phase_speed` | Wind, **Tension** | normalized: the dial is a multiple of `c` |
| `wave_density` | Waves | — |
| `mid_waves`, `mid_travel`, `mid_ripple` | Wind, Motion | size + travel from Wind, amount from Motion |
| `mid_flap` | Flap | **unbudgeted, adds on top** |
| `shed_freq_hz` | Flutter Hz | — |
| `wall_lo` / `wall_hi` | Walls (canvas) | hard; blockers windowed to match |
| `wind_origin` | Direction | — |
| `force_model`, anchors, taper | Physics | — |
| gain targets | Velocity, Tension, Range dB | — |
| `smooth_alpha_` (control one-pole) | Smoothing, Precision (tick rate) | — |
| `bank_.steady_amount_` | Steady | — |
| `leveler_.amount_` | Leveler | — |
| `mix_amount_` | Mix | dry/wet, pre-Leveler |
| side gain (per band) | **Width**, EQ dB | width = their difference in dB |
| pan (per band) | **Place** | mid→side cross-feed |
| `spatial_w_` | **Low Hz**, **High Hz** | taper on the spatial deviation only |
| `n_nodes_`, `sim_divisor_` | Precision | — |
| *(all of the above)* | the per-knob **power / magnitude** modifiers | §2b |

**Tension reaches five places** and is the most entangled remaining control,
though each consumer normalizes against it deliberately.

---

## §5 — Status: blocked, pending, open

Reorganised 2026-10-06 by what each item *demands*, rather than as a flat
list. Fixed items are kept with a date so a defect is not rediscovered.

### ⛔ BLOCKED — needs a decision before the work can continue

*Nothing is blocked. B1 was the only blocker; the route is chosen, built and
measured (below), and it unblocked B2. What remains is an ear test, which is
**P7**.*

**B1 · Band overlap — ROUTE CHOSEN AND BUILT (2026-10-06). Default is still
`Band Q = Flat` pending ears; see P7.**

The original finding, kept because the numbers are the baseline for the A/B:

`out = x + Σ bp_b·(g_b − 1)`. With **Q = 1** the bands are ~1.4 octaves wide
while the spacing is `9.23/N` octaves, so **~0.15·N bands overlap** at any
frequency and their contributions **add**:

```
actual = 1 + K·(g − 1),    K ≈ 0.152·N
```

| bands | nominal | actual | fitted K |
|---|---|---|---|
| 48 | +6 dB | +17.14 dB | 6.23 |
| 128 | +6 dB | +25.12 dB | 17.11 |
| **512** | **+6 dB** | **+36.92 dB** | **69.47** |

Three consequences:

1. At 512 nodes the EQ is ~70× deeper than the knob says. Already
   compensated by ear — the owner runs EQ dB at **3.74** against a default
   of 18.
2. **Cuts invert.** Once `K·(1−g) > 1` the effective gain goes negative: a
   nominal −3 dB at 512 nodes returns **+25.85 dB** with flipped polarity.
3. **Precision cross-couples with depth** — K runs 17 → 69 from 128 to 512
   nodes, so changing resolution changes EQ depth ~4×.

This is **the unimplemented per-node Q (O4) coming due**. Width and EQ
inherit the factor *equally* and stay consistent, so both remain usable;
**Place does not** (see B2).

#### What the measurements changed about the framing

I had framed this as a choice between **`1/K` normalisation** and **per-node
Q matched to spacing**, and said both "reshuffle every sound tuned by ear".
Measuring it (`q_test`) showed the framing was wrong in three ways.

**1. They are not alternatives — the correction is needed either way, and it
is derivable, not fitted.** `K` is the sum of the Q-normalised bandpass
responses at a band centre, which the bank now computes in closed form from
its own band map (`FilterBank::overlap_factor`). Analytic vs measured:

| bands | Q | K analytic | K measured | +6 dB asked → delivered |
|---|---|---|---|---|
| 128 | 1.00 | 23.87 | 23.88 | 6.00 dB |
| 128 | 19.85 | 1.69 | 1.69 | 6.00 dB |
| 512 | 1.00 | 96.06 | 96.08 | 6.00 dB |
| 512 | 79.88 | 1.70 | 1.70 | 6.00 dB |

**2. Matched Q's real prize is not depth, it is that `K` stops scaling with
node count** — 1.69 at 128 nodes and 1.70 at 512, against 23.9 and 96.1 at
Q = 1. One constant then corrects every depth at every precision. Verified
across +12 → −12 dB on both node counts, **no polarity inversion anywhere**.

**3. Q = 1 passes no spectral detail at all.** This is the finding that
actually decides it. Survival of a ±6 dB pattern alternating every *n* bands
(12 dB = fully passed):

| bands | Q | every 2 | every 4 | every 8 | every 16 |
|---|---|---|---|---|---|
| 128 | 1.00 | 0.00 dB | 0.00 dB | 0.01 dB | 0.35 dB |
| 128 | 19.85 (matched) | 4.30 dB | 5.52 dB | 8.59 dB | 10.38 dB |
| 512 | 1.00 | 0.00 dB | −0.00 dB | 0.00 dB | 0.00 dB |
| 512 | 79.88 (matched) | 4.25 dB | 5.45 dB | 8.46 dB | 10.22 dB |

At Q = 1 the bank is a ~1.4-octave smear: it responds to the string's
**coarse average** with ~24–96× the asked depth and to its *shape* with
nothing. **Precision buys no audible resolution at Q = 1** — 512 nodes
measured 0.00 dB at every scale tested. At matched Q the resolution is set
by node spacing instead, which is what Precision was supposed to mean.

**4. Flat Q is also the loudest and least safe setting.** Driven with a
sweeping ±18 dB shape on noise at input rms 0.115:

| bands | mode | peak | rms |
|---|---|---|---|
| 128 | Flat Q = 1 (today) | **16.995** | 4.045 |
| 512 | Flat Q = 1 (today) | **67.030** | 15.988 |
| 128 | Matched + 1/K | 1.520 | 0.334 |
| 512 | Matched + 1/K | 1.486 | 0.342 |

17× full scale at 128 nodes, 67× at 512. The Leveler has been absorbing an
enormous gain error, and that is why EQ dB had to be run at 3.74 of 24.
Nothing produced a non-finite sample at any Q up to 639.

**5. It is a WYSIWYG violation, not a nicety — the top invariant.** Drawn
curve vs measured response, smooth ±12 dB shape, worst of four probe bands:

| mode | 128 nodes | 512 nodes |
|---|---|---|
| Flat Q = 1 (today) | **31.19 dB** | **43.30 dB** |
| Flat Q = 1 + 1/K | 8.44 dB | 8.48 dB |
| Matched (Res 0) | 6.84 dB | 6.82 dB |
| Matched ×2 | 4.67 dB | 4.68 dB |
| Matched ×4 | 2.10 dB | 2.12 dB |
| Matched ×8 | 0.67 dB | 0.68 dB |

So invariant 1 is currently out by 31–43 dB. This is a third violation, and
unlike Steady and Mix it is **structural rather than a missed stage**.

**The residual after correction is entirely in the cuts, and the cause is
not neighbour boosts leaking in** — at the probe bands every neighbour is
also a cut. It is the quadrature term: `out = x·(1 + A + jB)` and the
correction only makes `A` right. On a sloped shape the neighbours' imaginary
contributions stop cancelling, and `|1 + A + jB|` is dominated by `B`
exactly where `1 + A` is small, i.e. in a deep cut. **A parallel additive
bank cannot notch deeply on a slope.** Raising Q shrinks the neighbour
weights and so shrinks `B`, which is why the column above falls monotonically.

#### What was built

Two parameters, in SUBSCRIBERS (they are bank properties, so they apply to
every subscriber at once, not just the EQ):

- **`Band Q`** — `Flat` (Q = 1, no correction: **bit-identical** to the
  historical bank, verified by `overlap_probe`, `gain_test` and the 12
  frozen scenarios) or `Matched` (band width = band spacing, plus the
  analytic `1/K`). The flag exists so the two can be A/B'd by ear.
- **`Resonance`** — pushes Q above matched, ×1 → ×8. Narrower bands ring
  longer and stop filling the gaps between each other.

The correction folds into the single multiply the inner loop already had
(`ok_ = k_/K`, where `k_ = 1/Q` normalises the SVF's Q-scaled bandpass), so
**the display needed no change**: `gains()` still reports the nominal value,
which is now also the response. WYSIWYG holds structurally rather than by
remembering to mirror it. Cost: bank 11.5% → 11.74% of a core at 512 nodes.

> **A trap worth recording.** The SVF's `v1` is the *Q-scaled* bandpass —
> `|v1|` peaks at Q, not at 1. At Q = 1 that is invisible, which is why the
> bank was correct by accident. Without the `k = 1/Q` normalisation, Q = 80
> would have been **80× too deep**.

#### Ring time — what the high-Q route actually buys

`T60 = 6.91·Q/(π·f)`, measured to match at every point tested. 128 nodes:

| Q | 58 Hz | 754 Hz | 9.4 kHz |
|---|---|---|---|
| 1.00 | 40 ms | 3 ms | ~0 ms |
| 19.85 (matched) | 754 ms | 58 ms | 7 ms |
| 79.40 (Res .67) | 3031 ms | 232 ms | 25 ms |

Constant Q means constant ring in *cycles*, so ring time falls as `1/f`. The
texture is therefore strongly low-frequency weighted — natural (a real
resonator behaves this way) but worth knowing before chasing broadband
shimmer. Ripple between band centres is the cost: 0.93 dB at matched,
3.24 dB at ×2, 5.73 dB at ×8, at which point the response is a comb rather
than an EQ. That comb is the point for texture work and a defect for EQ
work, which is exactly why `Resonance` is a knob and not a constant.

**B2 · Place — UNBLOCKED.** `pan` is corrected by the same `ok_` scalar, so
with `Band Q = Matched` the factor is 1.70 rather than ~69 and the knob's
travel is honest. Needs an ear check at its current range (P7).

### ⏳ PENDING — decided, waiting on ears or a later pass

- **P1 · M2 feel gate.** Flagpole "physical, not animated", zipper-free.
  Still the formal milestone exit.
- **P2 · Smoothing's default is un-tuned.** Fixing the per-channel
  double-step made stereo 2× slower at the same number; 15 ms is a
  restoration, not a choice.
- **P3 · Decorrelator.** Width does nothing on a mono source because the
  side is zero. The only real fix, and a genuine piece of work rather than a
  parameter. Deferred deliberately.
- **P4 · Magnitude multipliers `{0.5, 1.0, 2.0}`** remain a guess. Medium is
  calibrated; Low and High are untested by ear.
- **P5 · Motion budget under-delivers ~37%** at the default operating point
  (0.371 predicted, 0.234 measured) — the power law is likely stale since
  the Waves range extension and the wall/blocker windowing. Needs a re-fit.
- **P6 · Shedding has no Tension compensation** — peak falls 80.7× across the
  Tension dial at Character 1. Belongs with the Character work.

- **P7 · `Band Q = Matched` needs an ear test, and the default depends on
  it.** Everything measurable says Flat is broken — 31–43 dB WYSIWYG error,
  17–67× full-scale output, **zero** spectral resolution, depth coupled to
  Precision, cuts inverting. Nothing measurable says how Matched *sounds*,
  and every sound tuned so far was tuned against Flat. Flipping the default
  is an owner call, and it changes how projects saved before 2026-10-06
  sound (old states carry no `q_mode`, so they take the constructor
  default).

  **Checklist, with what counts as a pass.** Work top to bottom; items 1–3
  are expected-behaviour confirmations, not judgements, and getting them out
  of the way first stops them being mistaken for faults later.

  **1 · Flat mode now draws a much blunter curve. Expected, not a bug.**
  The bright curve is the bank's true response as of O10, and at `Q = 1`
  the bank genuinely smears over ~1.4 octaves. So in Flat the bright line
  will lose nearly all the detail the faint string line has.
  *Pass:* the bright line in Flat is visibly smooth/blunt while the faint
  line wiggles; flipping to Matched makes the bright line track the faint
  one closely. **That contrast is the resolution finding, visible.**

  **2 · Flat mode's dB axis now reads much wider than the knob.** With
  Range at 18 the axis will label around ±44 dB at 128 nodes, because that
  is what the bank is really doing. In Matched the axis equals the knob
  again.
  *Pass:* axis label changes when `Band Q` is flipped, and in Matched it
  matches the Range knob.

  **3 · Matched should be dramatically quieter at the same settings.**
  Measured 17× → 1.5× peak at 128 nodes, 67× → 1.5× at 512.
  *Pass:* flipping to Matched drops the level hard, and the Leveler's gain
  reduction falls with it. If Matched is *not* much quieter, something is
  wrong — stop and say so.

  **4 · Re-tune EQ dB.** 3.74 of 24 was compensation for a ~24× overshoot
  that no longer exists.
  *Check:* what value in Matched gives the depth 3.74 gave in Flat. Expect
  it to land far higher. Record the number — it recalibrates the default.

  **5 · Re-check Steady.** Its detector was Q-sensitive and is now
  normalised (verified Q-invariant to 0.01 dB on a tone). But a *narrower*
  band genuinely captures less broadband energy, so the same threshold
  means something different against real material.
  *Check:* does Steady still tame what it used to at the same setting? If
  it now feels weak on broadband material, the threshold wants rescaling
  with bandwidth — that is a real follow-up, not a bug.

  **6 · Re-check the Leveler.** It was absorbing an enormous gain error and
  will have far less to do.
  *Check:* whether its default 0.5 is still right, or now over-works a
  signal that no longer needs it.

  **7 · Place (B2) is unblocked — check its travel.** The cross-feed is
  corrected by the same scalar, so hard pan should arrive near the top of
  the knob rather than at 0.014.
  *Pass:* placement sweeps smoothly across most of the knob's range. Mono
  compatibility is structural (`L+R = 2·mid` identically, so the cross-feed
  cannot change it) and does not need testing again.

  **8 · Width.** Inherits the same correction.
  *Check:* Width in dB now means what it says against the EQ's dB.

  **9 · Resonance sweep — the actual new sound.** At 0 the bank is a clean
  EQ. Ring time is `6.91·Q/(π·f)`, so it is strongly bass-weighted.
  *Listen at 0 / 0.33 / 0.67 / 1.0 for:* ring appearing on low content
  first (754 ms at 58 Hz already at 0, 3 s at 0.67); the response turning
  comb-like as the gaps stop being filled (ripple 0.93 → 3.24 → 5.73 dB);
  and whether the texture is the shimmer the convolution trick produces.
  *Pass:* it is musically useful somewhere in the range. If the ring is
  only ever bass mud, the `1/f` weighting is the reason and a
  frequency-dependent Q is the follow-up.

  **10 · Precision should finally be audible.** At `Q = 1` it measured
  **0.00 dB** of resolution difference at every scale, so 128 vs 512 was
  inaudible. In Matched, resolution is set by node spacing.
  *Pass:* in Matched, switching 128 → 512 audibly sharpens the spectral
  detail. **If it does not, the resolution claim is wrong and I want to
  know.** In Flat it should still sound the same at any Precision.

  **11 · Smoothing against ring.** Ring is a *new* time constant that
  Smoothing does not control — Smoothing governs how fast the curve moves,
  not how long a band rings.
  *Check:* at high Resonance, whether fast Smoothing plus long ring smears
  into mush, and whether the two controls feel like they fight.

  **12 · CPU and glitching.** The honest curve costs 1.17% of a core at 512
  nodes (193 µs per publish at 60 Hz); the bank itself is unchanged at
  11.74%. 512 nodes with wind was already ~101% of a core before any of
  this.
  *Check:* small buffer sizes (64/128) at 512 nodes for dropouts, and CPU
  when the input goes **silent** at high Resonance — long ring-downs are
  where denormals would bite if `ScopedNoDenormals` were not covering it.

  **13 · Mix.** Folded into the response convolution exactly (the sum is
  linear in the per-band excess).
  *Pass:* at Mix 0.5 the drawn curve is still exactly what you hear.

  **14 · The honest answer to "should Matched be the default".** It is a
  sound judgement, not a measurement. The measurements only establish that
  Flat is wrong in five independent ways — they say nothing about which one
  you want to *make records with*.

### 🔧 OPEN — known, costed, nobody blocked

- **O1 · Wrinkle and Flap sit outside the excitation budget** (1.57× and
  1.45×) — they add on top rather than inside it.
- **O2 · Physics modes 2 and 3 are thin.** Physics is not yet a real switch
  between distinct simulations, which is axis 1 of the product.
- **O3 · Adding a physics mode is UI work too.** CLOTH & WIND is a
  Flagpole-shaped panel hard-coded alongside the generic ones.
- **O11 · Q is global; the string could drive it per node.** This is the
  original O4 vision and the harder half of it. `set_q` is already O(N) with
  `tan` cached precisely so per-band modulation is affordable (1 divide +
  3 multiplies per band per block). It needs a second control-signal
  subscriber and a decision about what drives Q — the same string shape as
  gain, its velocity, or an independent mode.
- **O13 · Physics runs 3.75× faster at low Precision.** Wave speed is in
  *nodes* per second, so a fixed Tension gives a fundamental `f0 = c/span`
  that falls as `1/N`. Measured control-signal rate **9.44 / 4.41 /
  2.52 Hz** at 128 / 256 / 512 with an identical patch, so Precision
  changes the *tempo* of the cloth and not just its resolution.
  Scaling `c` by `n/128` flattens it to **1.47×** and is stable (no
  non-finite samples, peak bounded, scenarios unaffected) — **but it wrecks
  the level**, because `kNodeWindGain`, `node_norm` and `kTensionP` were all
  calibrated against the unscaled speed: the control rms spread goes from
  1.4× (0.0406 / 0.0454 / 0.0569) to **6.2×** (0.0406 / 0.0145 / 0.0065).
  Trading a rate error for a worse level error is not a fix, so it was
  backed out. Doing it properly means re-deriving those three per-precision
  tables at the scaled speed, which is a measured-calibration pass of its
  own and wants ears at the end. `dual_test` section 9 is the harness, and
  its rate metric is validated against wave speed first (Tension 300 → 600
  gives 7.69 → 14.67 Hz, 1.91× against a predicted 2×).
- **O12 · Pre-distortion — BUILT (2026-10-09), and it corrected my own
  diagnosis.** The bank delivers `target ⊛ kernel`, so it is now sent
  `target ⊘ kernel`: the inverse of the kernel's REAL part, precomputed
  once per Q change by early-stopped Landweber (self-regularising — where
  the forward kernel passes nothing the iteration simply has not
  converged, so it under-corrects rather than demanding impossible gains)
  and applied as one short convolution per chunk.

  It fixes the **excursions** completely from matched Q upward: a ±18 dB
  request now measures −17.8/+18.0 dB at matched and an exact ±18.0 at
  matched ×4. The cut/boost skew is gone.

  **It does not fix the mean, and that corrected my diagnosis.** I had
  attributed the +7.3 dB standing bias to linear averaging of the real
  part. It is not: inverting the real part exactly leaves the mean at
  +6.8 dB. The bias is the **quadrature term** — the response is
  `|1 + Re + jIm|`, real per-band gains can only set `Re`, and the
  magnitude is therefore always ≥ the target, inflated wherever `Im` is
  large (on slopes). Real gains in a parallel bank cannot cancel it. The
  only lever is narrower bands, which shrink the neighbour contributions
  that produce `Im`: mean **+6.8 dB at matched Q, +2.9 at matched ×4,
  +1.2 at matched ×8**.

  Blended by Band Q, because at Band Q 0 the bank transmits 0.00 dB of
  detail — a genuine null space — and deconvolving there made the mean
  *worse* (+11.3 dB against +7.3 uncorrected). The bottom of the knob is
  therefore left exactly as it was and the top gets the correction in
  full. Costs about 3 points of CPU at 512 nodes.

- **O4 · Per-node Q — the GLOBAL half is now built** (`Band Q`,
  `Resonance`; see B1). What remains is per-node modulation, tracked as
  O11. Previously read: `Q = 1.0` fixed, despite being a
  recorded architectural decision. See B1 — this is the root of it.
- **O5 · Stiffness is weaker than its range** now that DC and tilt are
  blocked.
- **O6 · Precision rebuild allocates on the audio thread.**
- **O7 · Three stepped parameters are rotaries** (Physics, Precision,
  Direction).
- **O8 · 96 kHz halves the headroom** — 512 nodes ≈ 85% of one core.
- **O9 · CLAP has no deploy step** — build tree only.

### ✅ FIXED — kept so they are not rediscovered

- ~~**O10 · The curve drew the nominal per-band gain, not the response**~~
  (2026-10-06). The bands overlap and add, so the drawn value never was what
  the audio did — **31 dB out at 128 nodes, 43 dB at 512**. The curve is now
  `FilterBank::response_curve`, the bank's exact summed response. Verified
  against the measured magnitude response at **0.12 dB worst case** across
  every Band Q and Resonance setting, down from 31–43 dB (q_test J).
  Because the bands are geometrically spaced and Q is uniform, the response
  is a **convolution with a fixed kernel**, rebuilt only when Q or the band
  map moves — no transcendentals per point. Costs 12 µs at 128 nodes and
  193 µs at 512 per publish, i.e. 0.07% / 1.17% of a core at the 60 Hz
  publish rate.
  > **A narrower window was tried and measured wrong.** Sizing it by
  > bandwidth-in-bands (7× the −3 dB width, 12 bands at matched Q) still
  > left 3.95 dB of error at 128 nodes and 6.46 dB at 512 — barely better
  > than the nominal curve. The real part of `Hn` falls off as
  > `(BW/lag)²` and truncates happily; the **imaginary** part falls off as
  > `k/u`, geometric in the frequency *ratio*, and `u` is only 1.16 twelve
  > bands out. It is precisely that quadrature term that dominates a cut.
  > The window is now the full band count.
- ~~**Steady's detector was Q-sensitive**~~ (2026-10-06, found while building
  Band Q and never shipped). The threshold is a fixed absolute level on the
  bandpass, but the SVF's `v1` peaks at **Q** — so at matched Q on 512 nodes
  the detector would have seen ~80× more signal and slammed to near-full
  duck on almost any input. Normalised by `k = 1/Q` like every other use of
  `v1`; now reads within **0.01 dB** across Q = 1, matched and matched×4
  (q_test K). Exact at Q = 1.
- ~~Velocity inert~~ (2026-10-02) — was 3.4% at full; now a position/velocity
  blend with the constant corrected.
- ~~Steady's duck hidden from the curve~~ (2026-10-02) — up to 4.52 dB
  invisible; mono detector, `gains()` Steady-inclusive.
- ~~Range dB leaked into the drawn curve~~ (2026-10-02) — smoother moved onto
  the control signal; 0.298 → 4.95e-14.
- ~~Stereo smoothed at half the set time~~ (2026-10-02) — gain advanced once
  per channel instead of once per frame.
- ~~Gain latency scaled with the host buffer~~ (2026-10-02) — targets aimed
  once per block; now re-aimed every 32 samples, constant 0.0164.
- ~~Mix ignored by the drawn curve~~ (2026-10-02) — folded in exactly.
- ~~Wind mis-ranged against f0~~ (2026-10-02) — now a multiple of the wave
  speed.
- ~~Waves drained the string~~ (2026-10-02) — 2.3× quieter across the dial;
  compensated, now flat to 1.14×.
- ~~Curve vanished with the EQ gated off~~ (2026-10-06) — display now follows
  the active subscriber (F8b).

---

## §6 — Invariants

Two rules that constrain all future work:

**WYSIWYG.** The bright curve is drawn from `FilterBank::gains()` with the
same ±1 clamp the audio path uses — never from the sim. Any amplitude
mechanism must keep the string inside ±1 in normal use, or the EQ flat-tops.

**Image provenance** — the stereo image moves only when told to. *(Renamed
and reworded from "channel coherence" 2026-10-06: the old name described a
mechanism we deliberately left behind, and would have made every stereo
feature look like a violation.)*

One instance is one string. Every per-band coefficient the bank applies —
**mid gain, side gain, and mid→side cross-feed** — must be a deterministic
function of the control surface and the knobs, and therefore **drawable as a
curve**. That is the sharp test: *if you cannot draw it, its provenance is
wrong.*

**Forbidden:** any detector that measures the channels separately and lets
the result change the balance between them. That is how the image starts
drifting as a side effect of program material, and it is why Steady's
per-channel follower was removed — no single curve was correct, so the
display showed none of it.

**Permitted:** deliberate stereo transforms driven by the control surface —
per-band width and placement — because they are intended, reproducible,
automatable and fully described by a curve.

**Content-driven gain reduction stays channel-linked.** The Leveler's
control signal is `max(|L|,|R|)` and Steady's detector is the mid, both for
this reason. Those are the rule still applying, not legacy.

Stereo has **two routes**: instancing (separate instances per channel →
decorrelated motion) and the spatial subscriber (one instance → controlled
placement). Complementary, not competing.

**Bit-exact cross-validation.** The C++ engine is validated against the
Python spec cell by cell: **12 scenarios, 119,936 cells, 0 mismatches**. Every
physics change must land in both with identical arithmetic order, and the
pre-existing scenario CSVs must stay byte-identical.

```
python prototype/run_scenarios.py        # Python spec
build/Release/scenarios.exe prototype    # C++ engine
build/Release/bench.exe 48000            # per-core cost
build/Release/gain_test.exe              # Range invariance, dB symmetry, Steady
build/Release/tension_test.exe 0.85 0 9  # Tension vs amplitude (self-validating)
build/Release/display_test.exe           # does the drawn curve track the string
build/Release/wind_test.exe              # Wind sweeps, old vs shipped mapping
build/Release/rnd_test.exe               # randomisers + the direction invariant
build/Release/stereo_test.exe            # width, placement, mono-safety
build/Release/overlap_probe.exe          # the band-overlap factor K (B1)
build/Release/q_test.exe                 # Band Q: K analytic vs measured,
                                         # resolution, ring time, levels,
                                         # WYSIWYG error (B1). Slow (~4 min):
                                         # high-Q bands need seconds to settle
build/Release/display_src.exe            # every subscriber config is drawable
build/Release/spatial_test.exe           # spatial spectrum by wavenumber band
```
