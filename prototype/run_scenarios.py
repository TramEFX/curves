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
    # Pre-tilt-blocker baseline: frozen regression tripwire.
    sim.remove_tilt = False
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
    # Pre-tilt-blocker baseline: frozen regression tripwire.
    sim.remove_tilt = False
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
    # Pre-tilt-blocker baseline: frozen regression tripwire.
    sim.remove_tilt = False
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
    # Pre-tilt-blocker baseline: frozen regression tripwire.
    sim.remove_tilt = False
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
    # Pre-tilt-blocker baseline: frozen regression tripwire.
    sim.remove_tilt = False
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
    # Pre-tilt-blocker baseline: frozen regression tripwire.
    sim.remove_tilt = False
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
    # Pre-tilt-blocker baseline: frozen regression tripwire.
    sim.remove_tilt = False
    sim.anchor_high = True
    sim.set_cloth_aero(phase_velocity_nodes_s=192.0, wavelength_nodes=128.0,
                       envelope_exponent=1.5)
    # A deliberately hot wind that exercises the drag model; this is a
    # frozen baseline value, NOT the shipping dial top (the plugin maps
    # Force = 1.0 to 25 * (tension/300)**0.6 * (128/n)**0.25).
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


def scenario_tilt():
    """Tilt blocker: an asymmetric deposit dumps most of its energy into
    mode 1 (a linear tilt), which tension cannot restore. Without the
    blocker that mode rings as a slow global rocking that outlives every
    real wave. With it, the tilt stays at zero and only real waves remain."""
    print("scenario: tilt (mode-1 blocker; asymmetric deposit)")
    sim = StringSim(n_nodes=N, tick_rate=TICK_RATE,
                    wave_speed=300.0, damping=0.4, stiffness=4.0)
    # remove_tilt defaults to True - this scenario exercises the blocker.
    sim.displace(0.6 * np.tanh(np.linspace(-2.5, 2.5, N)))
    mid = 0.5 * (N - 1)
    xi = np.arange(N) - mid
    sxx = float(np.sum(xi * xi))
    frames = []
    late_tilt = 0.0
    late_peak = 0.0
    for s in range(8000):
        sim.tick()
        if s % 200 == 0:
            frames.append((s * sim.dt, sim.u.copy()))
        if s * sim.dt >= 3.0:
            late_tilt = max(late_tilt, abs(float(np.sum(xi * sim.u)) / sxx) * mid)
            late_peak = max(late_peak, float(np.max(np.abs(sim.u))))
    save_frames("tilt", frames)
    print(f"  late tilt displacement {late_tilt:.6f}  (blocker holds it at ~0 - no slow rocking)")
    print(f"  late peak|u|           {late_peak:.6f}  (only genuine waves remain)")


def scenario_shed():
    """Vortex shedding: the wake oscillator is a genuine limit cycle, so the
    string must flap with the gust wind switched OFF ENTIRELY. That is the
    whole point - motion that is alive rather than driven. Also checks the
    wake decorrelates into cells instead of one global in-phase breath."""
    print("scenario: shed (wake oscillator, ZERO gust wind)")
    sim = StringSim(n_nodes=N, tick_rate=TICK_RATE,
                    wave_speed=300.0, damping=1.2, stiffness=2.0)
    sim.set_cloth_aero(phase_velocity_nodes_s=192.0, wavelength_nodes=128.0,
                       envelope_exponent=1.5)
    sim.wind_speed = 0.0     # no gusting at all - shedding must self-sustain
    sim.aero_rho_cd = 0.6
    sim.set_density_taper(2.0)
    sim.set_cloth_length(1.0)
    sim.shed_gain = 60.0
    frames = []
    peak = 0.0
    probe_idx = N // 4
    prev = 0.0
    crossings = 0
    t_first, t_last = -1.0, 0.0
    coh = []
    # Wave momentum flux, accumulated online (see src/scenarios_main.cpp):
    # u_t IS the velocity array, so -sum(u_t * u_p) is positive when waves
    # travel away from the pole. p is distance from the pole, so dp/di flips
    # sign with the wind origin.
    fnum = 0.0
    fden_v = 0.0
    fden_p = 0.0
    dpdi = 1.0 if sim.wind_origin == 1 else -1.0
    for s in range(6000):
        sim.tick()
        t = s * sim.dt
        if s % 100 == 0:
            frames.append((t, sim.u.copy()))
        if t >= 2.0:
            peak = max(peak, float(np.max(np.abs(sim.u))))
            cur = float(sim.u[probe_idx])
            if prev != 0.0 and ((cur > 0.0) != (prev > 0.0)):
                crossings += 1
                if t_first < 0.0:
                    t_first = t
                t_last = t
            prev = cur
            coh.append(abs(float(np.mean(sim._q)))
                       / max(float(np.mean(np.abs(sim._q))), 1e-9))
            u_p = 0.5 * (sim.u[2:] - sim.u[:-2]) * dpdi
            vv = sim.v[1:-1]
            fnum -= float(np.sum(vv * u_p))
            fden_v += float(np.sum(vv * vv))
            fden_p += float(np.sum(u_p * u_p))
    save_frames("shed", frames)
    rate = (0.5 * (crossings - 1) / (t_last - t_first)
            if crossings > 2 and t_last > t_first else 0.0)
    finite = bool(np.all(np.isfinite(sim.u)))
    print(f"  late peak|u|      {peak:.6f}  (self-sustained with NO gust wind - the alive test)")
    print(f"  probe modulation  {rate:.6f} Hz  (shed_freq_hz = 3.0)")
    flux = fnum / np.sqrt(fden_v * fden_p) if fden_v * fden_p > 0 else 0.0
    print(f"  travel direction  {flux:.6f}  (POSITIVE = waves move AWAY from the anchor, with the wind)")
    print(f"  wake coherence    {float(np.mean(coh)):.3f}  (0 = vortex cells, 1 = one global breath)")
    print(f"  finite: {'yes' if finite else 'NO - BLEW UP'}")


def scenario_wrinkle():
    """Tip wrinkles: fine detail concentrated in the LAST THIRD. Asserts the
    detail is localized - tip curvature rises while the pole end stays put.
    Curvature (the second spatial difference) is the density proxy: it
    weights short wavelengths heavily and is cheap and deterministic in both
    languages, unlike an FFT."""
    print("scenario: wrinkle (tip-resonant fine detail, last third)")
    # Wrinkles need resolution: at 128 nodes their wavelengths land at
    # 10.7-4.1 nodes, at or past what the grid can carry, and the tip then
    # measures LESS detailed than the pole. This scenario runs at 512 nodes
    # - the regime the feature is for. Wrinkle is a high-precision control.
    WN = 512
    sim = StringSim(n_nodes=WN, tick_rate=TICK_RATE,
                    wave_speed=300.0, damping=1.2, stiffness=2.0)
    sim.set_cloth_aero(phase_velocity_nodes_s=192.0 * WN / 128.0,
                       wavelength_nodes=float(WN), envelope_exponent=1.5)
    sim.wind_speed = 25.0 * (128.0 / WN) ** 0.25
    sim.aero_rho_cd = 0.6
    sim.set_density_taper(2.0)
    sim.set_cloth_length(1.0)
    sim.wrinkle = 2.0
    frames = []
    tip_c = tip_a = pole_c = pole_a = 0.0
    peak = 0.0
    for s in range(6000):
        sim.tick()
        t = s * sim.dt
        if s % 100 == 0:
            frames.append((t, sim.u.copy()))
        if t < 2.0:
            continue
        curv = np.abs(sim.u[:-2] - 2.0 * sim.u[1:-1] + sim.u[2:])
        mag = np.abs(sim.u[1:-1])
        peak = max(peak, float(np.max(np.abs(sim.u))))
        third = WN // 3
        tip_c += float(np.sum(curv[:third - 1]))
        tip_a += float(np.sum(mag[:third - 1]))
        pole_c += float(np.sum(curv[2 * third - 1:]))
        pole_a += float(np.sum(mag[2 * third - 1:]))
    save_frames("wrinkle", frames)
    tipd = tip_c / max(tip_a, 1e-12)
    poled = pole_c / max(pole_a, 1e-12)
    print(f"  tip   wrinkle density {tipd:.6f}")
    print(f"  pole  wrinkle density {poled:.6f}   (ratio {tipd / max(poled, 1e-12):.2f}x - detail is localized at the END)")
    print(f"  late peak|u| {peak:.6f}   finite: {'yes' if np.all(np.isfinite(sim.u)) else 'NO - BLEW UP'}")


def scenario_mid():
    """Mid-band ripples: the wavenumbers between the gust (1.7-6 waves across
    the cloth) and wrinkle (25-64). Resonance-locked to the MID-cloth wave
    speed with a raised-cosine bump envelope, so the extra detail lands in
    the middle third. Curvature is the density proxy, as in the wrinkle
    scenario - cheap and deterministic in both languages."""
    print("scenario: mid (mid-band ripples, resonance-locked mid-cloth)")
    MN = 512
    sim = StringSim(n_nodes=MN, tick_rate=TICK_RATE,
                    wave_speed=300.0, damping=1.2, stiffness=2.0)
    sim.set_cloth_aero(phase_velocity_nodes_s=192.0 * MN / 128.0,
                       wavelength_nodes=float(MN), envelope_exponent=1.5)
    sim.wind_speed = 25.0 * (128.0 / MN) ** 0.25
    sim.aero_rho_cd = 0.6
    sim.set_density_taper(2.0)
    sim.set_cloth_length(1.0)
    sim.mid_ripple = 1.5
    sim.mid_waves = 10.0
    # the shipped shape, so this scenario covers what the plugin uses
    sim.mid_lock_d = 0.65
    sim.mid_env = 1.4
    sim.mid_travel = 0.83
    sim.mid_flap = 0.45
    frames = []
    mid_c = end_c = 0.0
    peak = 0.0
    for s in range(6000):
        sim.tick()
        t = s * sim.dt
        if s % 100 == 0:
            frames.append((t, sim.u.copy()))
        if t < 2.0:
            continue
        curv = np.abs(sim.u[:-2] - 2.0 * sim.u[1:-1] + sim.u[2:])
        idx = np.arange(1, MN - 1)
        peak = max(peak, float(np.max(np.abs(sim.u))))
        # integer bounds chosen to match the C++ loop exactly
        m = (idx >= MN // 3) & (idx < 2 * MN // 3)
        mid_c += float(np.sum(curv[m]))
        end_c += float(np.sum(curv[~m]))
    save_frames("mid", frames)
    print(f"  mid-third curvature {mid_c:.6f}  outer {end_c:.6f}"
          f"  (ratio {mid_c / max(end_c, 1e-9):.2f}x)")
    print(f"  late peak|u| {peak:.6f}   finite: {'yes' if np.all(np.isfinite(sim.u)) else 'NO - BLEW UP'}")


def scenario_random():
    """Seeded spread on both the gust sites and the wake bank. At spread 0
    the randomiser is identity, so the frozen baselines say nothing about
    this path -- it needs its own scenario in both engines."""
    print("scenario: random (seeded spread on both gust and wake)")
    RN = 256
    sim = StringSim(n_nodes=RN, tick_rate=TICK_RATE,
                    wave_speed=300.0, damping=1.2, stiffness=2.0)
    sim.anchor_high = True
    sim.force_model = 2
    sim.set_cloth_aero(phase_velocity_nodes_s=192.0 * RN / 128.0,
                       wavelength_nodes=float(RN), envelope_exponent=1.5)
    sim.wind_speed = 25.0 * (128.0 / RN) ** 0.25
    sim.aero_rho_cd = 0.6
    sim.set_density_taper(2.0)
    sim.set_cloth_length(1.0)
    sim.shed_gain = 30.0
    sim.shed_freq_hz = 3.0
    sim.gust_spread = 0.7
    sim.gust_seed = 20260103
    sim.shed_spread = 0.6
    sim.shed_seed = 7
    frames = []
    peak = 0.0
    for s in range(6000):
        sim.tick()
        t = s * sim.dt
        if s % 100 == 0:
            frames.append((t, sim.u.copy()))
        if t < 2.0:
            continue
        peak = max(peak, float(np.max(np.abs(sim.u))))
    save_frames("random", frames)
    print(f"  late peak|u| {peak:.6f}   finite: {'yes' if np.all(np.isfinite(sim.u)) else 'NO - BLEW UP'}")


if __name__ == "__main__":
    scenario_pluck()
    scenario_flagpole()
    scenario_travel()
    scenario_recover()
    scenario_cloth()
    scenario_cloth_rev()
    scenario_aero()
    scenario_tilt()
    scenario_shed()
    scenario_wrinkle()
    scenario_mid()
    scenario_random()
    print("done.")
