"""
Curves prototype — scenario harness.

Runs the string sim through the three scenarios that matter for the
go/no-go decision and dumps frames to CSV for plotting:

  pluck      draw a shape, release it -> does it ring and settle?
  flagpole   anchor the high end, wind from the high side blowing the
             curve down the frequency axis -> does it feel physical?
  travel     a localized bump -> does the disturbance genuinely travel
             and reflect (the "slow variable wave function")?
"""

import csv
import os
import numpy as np
from string_sim import StringSim

HERE = os.path.dirname(os.path.abspath(__file__))
N = 128
TICK_RATE = 1000.0


def save_frames(name, frames):
    path = os.path.join(HERE, f"{name}.csv")
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["scenario", "tick", "time_s", "node", "value"])
        for time_s, u in frames:
            tick = round(time_s * TICK_RATE)
            for i, val in enumerate(u):
                w.writerow([name, tick, round(time_s, 4), i, round(float(val), 6)])
    print(f"  -> {path} ({len(frames)} frames)")


def scenario_pluck():
    print("scenario: pluck (draw, release, ring, settle)")
    sim = StringSim(n_nodes=N, tick_rate=TICK_RATE,
                    wave_speed=300.0, damping=1.5, stiffness=4.0)
    sim.anchor_low = True
    sim.anchor_high = True
    sim.pluck(node=64, amount=1.0, sigma=5.0)
    e_start = sim.energy()
    frames = sim.run(seconds=2.0, capture_every=0.05)
    save_frames("pluck", frames)
    print(f"  energy start {e_start:.3f}  end {sim.energy():.5f}  "
          f"(should decay toward 0)")


def scenario_flagpole():
    print("scenario: flagpole (anchor high end, wind blows leftward, then stops)")
    sim = StringSim(n_nodes=N, tick_rate=TICK_RATE,
                    wave_speed=300.0, damping=1.2, stiffness=2.0)
    sim.anchor_high = True  # the flagpole: highest frequency pinned

    idx = np.arange(N)
    # Wind blows toward the low end (left), stronger near the low/free end
    # like a flag trailing away from its pole. The zero-mean constraint
    # (DC blocker) centers the sag, so amplitude is raised to compensate.
    wind = -12.0 * (1.0 - idx / (N - 1))
    sim.set_force_field(wind)
    frames_a = sim.run(seconds=1.5, capture_every=0.1)

    sim.clear_forces()  # wind stops -> spring-back should be visible
    frames_b = sim.run(seconds=1.0, capture_every=0.1)

    save_frames("flagpole", frames_a + frames_b)
    print(f"  sag under wind: max|u| = {np.max(np.abs(frames_a[-1][1])):.3f}")
    print(f"  after release:  max|u| = {np.max(np.abs(frames_b[-1][1])):.4f} (should sag back)")


def scenario_travel():
    print("scenario: travel (localized bump -> genuine wave propagation)")
    sim = StringSim(n_nodes=N, tick_rate=TICK_RATE,
                    wave_speed=300.0, damping=0.8, stiffness=0.0)
    sim.anchor_low = True
    sim.anchor_high = True
    sim.pluck(node=24, amount=1.0, sigma=3.0)
    frames = sim.run(seconds=0.8, capture_every=0.02)
    save_frames("travel", frames)
    print(f"  wavefront crossing time ~ {N / 300.0:.3f}s per traverse")


def scenario_recover():
    print("scenario: recover (loose+undamped string, then tension back up)")
    sim = StringSim(n_nodes=N, tick_rate=TICK_RATE,
                    wave_speed=40.0, damping=0.05, stiffness=4.0)
    sim.anchor_low = True
    sim.anchor_high = True
    sim.pluck(node=64, amount=1.0, sigma=8.0)
    frames = sim.run(seconds=1.0, capture_every=0.25)   # loose: modes barely couple
    sim.set_wave_speed(300.0)                           # tension restored
    frames += sim.run(seconds=2.0, capture_every=0.25)
    save_frames("recover", frames)
    late = [u for t, u in frames if t >= 1.25]
    dc = max(abs(float(np.mean(u))) for u in late)
    print(f"  late |mean(u)| max: {dc:.6f}  (DC blocker keeps this ~0 — no stuck breathing)")
    print(f"  late max|u|:        {max(float(np.max(np.abs(u))) for u in late):.4f}  (energy still travels)")


def scenario_cloth():
    print("scenario: cloth (pole anchored, free whip end, length 1.0, dir high->low)")
    sim = StringSim(n_nodes=N, tick_rate=TICK_RATE,
                    wave_speed=300.0, damping=1.2, stiffness=2.0)
    sim.anchor_high = True
    sim.set_flagpole_wind(phase_velocity_nodes_s=192.0, wavelength_nodes=128.0,
                          envelope_exponent=1.5)
    sim.wind_strength = 400.0
    sim.set_density_taper(2.0)
    sim.set_cloth_length(1.0)
    sim.flap_limit = 0.6
    uframes, vframes = [], []
    for s in range(4000):
        sim.tick()
        if s % 100 == 0:
            uframes.append((s * sim.dt, sim.u.copy()))
            vframes.append(sim.v.copy())
    save_frames("cloth", uframes)
    late_u = [u for t, u in uframes if t >= 1.5]
    vlast = vframes[15:]
    pole_u = max(float(np.max(np.abs(u[7 * N // 8:]))) for u in late_u)
    tip_u = max(float(np.max(np.abs(u[:N // 8]))) for u in late_u)
    pole_v = max(float(np.max(np.abs(v[7 * N // 8:]))) for v in vlast)
    tip_v = max(float(np.max(np.abs(v[:N // 8]))) for v in vlast)
    print(f"  pole_u {pole_u:.3f}  tip_u {tip_u:.3f}  (u-ratio {tip_u / max(pole_u, 1e-6):.1f}x)")
    print(f"  pole_v {pole_v:.1f}  tip_v {tip_v:.1f}  (v-ratio {tip_v / max(pole_v, 1e-6):.1f}x — the whip)")


def scenario_cloth_rev():
    print("scenario: cloth reversed (same, wind runs low->high, length 0.8)")
    sim = StringSim(n_nodes=N, tick_rate=TICK_RATE,
                    wave_speed=300.0, damping=1.2, stiffness=2.0)
    sim.anchor_high = True
    sim.set_flagpole_wind(phase_velocity_nodes_s=192.0, wavelength_nodes=128.0,
                          envelope_exponent=1.5)
    sim.wind_strength = 400.0
    sim.set_density_taper(2.0)
    sim.set_cloth_length(0.8)  # termination inside the string
    sim.flap_limit = 0.6
    sim.set_wind_origin(1)     # pole at the LOW end: whip at the highs
    uframes = []
    for s in range(4000):
        sim.tick()
        if s % 100 == 0:
            uframes.append((s * sim.dt, sim.u.copy()))
    save_frames("cloth_rev", uframes)
    late_u = [u for t, u in uframes if t >= 1.5]
    tail_u = max(float(np.max(np.abs(u[:N // 8]))) for u in late_u)  # beyond termination
    whip_u = max(float(np.max(np.abs(u[7 * N // 8:]))) for u in late_u)  # whip at the HIGHS
    print(f"  tail_u {tail_u:.3f} (damped)   whip_u {whip_u:.3f} (at the highs)")


def scenario_aero():
    """Cloth AERO: the wind is a velocity field and the force comes from
    RELATIVE velocity, so the string can fight the wind. Checks the whip is
    intact, amplitude is bounded without a flap limiter, nothing blows up."""
    print("scenario: aero (relative-velocity drag, no flap limiter)")
    sim = StringSim(n_nodes=N, tick_rate=TICK_RATE,
                    wave_speed=300.0, damping=1.2, stiffness=2.0)
    sim.anchor_high = True
    sim.set_cloth_aero(phase_velocity_nodes_s=192.0, wavelength_nodes=128.0,
                       envelope_exponent=1.5)
    # Calibrated Force = 1.0 at this node count (see ORCHESTRATION section 14):
    # wind_speed = 37 * force * (tension/300)**0.6 * (128/n)**0.25
    sim.wind_speed = 37.0
    sim.aero_rho_cd = 0.6
    sim.set_density_taper(2.0)
    sim.set_cloth_length(1.0)
    # flap_limit deliberately left OFF: drag is the amplitude mechanism now.
    uframes, vframes = [], []
    for s in range(4000):
        sim.tick()
        if s % 100 == 0:
            uframes.append((s * sim.dt, sim.u.copy()))
            vframes.append(sim.v.copy())
    save_frames("aero", uframes)
    late_u = [u for t, u in uframes if t >= 1.5]
    vlast = vframes[15:]
    pole_u = max(float(np.max(np.abs(u[7 * N // 8:]))) for u in late_u)
    tip_u = max(float(np.max(np.abs(u[:N // 8]))) for u in late_u)
    pole_v = max(float(np.max(np.abs(v[7 * N // 8:]))) for v in vlast)
    tip_v = max(float(np.max(np.abs(v[:N // 8]))) for v in vlast)
    peak = max(float(np.max(np.abs(u))) for u in late_u)
    finite = all(np.all(np.isfinite(u)) for _, u in uframes)
    print(f"  pole_u {pole_u:.3f}  tip_u {tip_u:.3f}  (u-ratio {tip_u / max(pole_u, 1e-6):.1f}x)")
    print(f"  pole_v {pole_v:.1f}  tip_v {tip_v:.1f}  (v-ratio {tip_v / max(pole_v, 1e-6):.1f}x - the whip)")
    print(f"  late peak|u| {peak:.3f}  (bounded by drag, no limiter; under the +-1 EQ clamp)")
    print(f"  finite: {'yes' if finite else 'NO - BLEW UP'}")


if __name__ == "__main__":
    scenario_pluck()
    scenario_flagpole()
    scenario_travel()
    scenario_recover()
    scenario_cloth()
    scenario_cloth_rev()
    scenario_aero()
    print("done.")
