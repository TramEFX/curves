"""
Curves — string simulation prototype.

The control string: a 1D damped wave equation over N nodes. This is the
physics engine that owns the nodes; everything else (drawing, forces,
audio analysis) writes to it through the interfaces below. The filter
bank will *read* node positions; this prototype just proves the dynamics.

Physics (finite difference, unit node spacing):
    a_i = c^2 * (u[i-1] - 2u[i] + u[i+1])   <- wave propagation (tension)
        - k * u[i]                          <- restoring force toward flat
        - d * v[i]                          <- viscous damping
        + f[i]                              <- external force field (wind)

Stability (CFL): wave_speed * dt <= 1.0 with unit node spacing.
Enforced by assertion.

Units: time in seconds, displacement normalized (roughly -1..+1 in practice).
"""

import numpy as np


class StringSim:
    def __init__(self, n_nodes=128, tick_rate=1000.0,
                 wave_speed=300.0, damping=1.5, stiffness=4.0):
        """
        n_nodes:    control-string resolution.
        tick_rate:  sim ticks per second (in the plugin this will be
                    sample_rate / tick_divisor, e.g. every 2-4 samples).
        wave_speed: propagation speed in nodes/second. Higher = stiffer
                    string, disturbances travel faster.
        damping:    velocity damping (1/s). Higher = settles faster.
        stiffness:  restoring force toward flat (1/s^2). Higher = the
                    whole string sags back to zero when forces stop.
        """
        assert n_nodes >= 4
        self.n = n_nodes
        self.dt = 1.0 / tick_rate
        assert wave_speed * self.dt <= 1.0, "CFL violation: wave_speed * dt > 1"
        self.c2 = wave_speed * wave_speed
        self.damping = damping
        self.stiffness = stiffness

        self.u = np.zeros(n_nodes)   # positions (what the filter bank reads)
        self.v = np.zeros(n_nodes)   # velocities (for velocity coupling later)
        self.force = np.zeros(n_nodes)

        # ---- boundary conditions (the anchors) ----
        self.anchor_low = False
        self.anchor_high = False

        # ---- force models ----
        # kStaticField (0): self.force used as-is.
        # kFlagpoleWind (1): traveling forcing wave — amplitude grows with
        # distance from the anchored (high) end, phase travels toward the
        # free end. Small motion at the pole, whip at the tip.
        # kStaticField (0) / kFlagpoleWind (1) / kClothAero (2).
        self.force_model = 0
        # ---- cloth aero model (kClothAero) ----
        # The gust pattern becomes a transverse wind VELOCITY field (nodes/s)
        # instead of a force. Per node:
        #   v_rel = W_i - v_i
        #   F_i   = aero_rho_cd * |v_rel|*v_rel / (1 + s_i^2),  s_i = du/dx
        # Drag rises quadratically with how fast the node moves through the
        # air, so flap amplitude finds its own equilibrium — the ceiling is
        # physics, not a limiter. See src/string_sim.h for the full rationale.
        self.wind_speed = 0.0
        self.aero_rho_cd = 0.6
        self._w_field = np.zeros(self.n)
        self._m_inv_max = 1.0
        self._aero_vrel_max = 1e9
        self.wind_strength = 0.0
        self.wind_phase_speed = 192.0
        self.wave_density = 1.0  # Waves knob: spatial density of the wind pattern
        self.wind_origin = 0  # 0 = pole at high end (whip at lows), 1 = pole at low end
        # ---- tip wrinkles (see src/string_sim.h for the full rationale) ----
        # Four extra gust components windowed by a d^4 tip envelope, with high
        # spatial wavenumbers AND a phase velocity locked to the LOCAL WAVE
        # SPEED AT THE TIP. The resonance lock is the whole trick: at these
        # wavenumbers c^2*k^2 is ~1944 against a stiffness of ~2, so short
        # waves are suppressed ~1000x and can only be excited on resonance.
        # wrinkle = 0 (default) skips the stage entirely.
        self.wrinkle = 0.0
        # Negative = follow wind_speed (original behaviour, relied on by the
        # frozen scenarios). See src/string_sim.h.
        self.wrinkle_speed = -1.0
        self.MID_K = np.array([0.72, 1.0, 1.37, 1.9])
        self.WRINKLE_K = np.array([12.0, 17.0, 23.0, 31.0])
        # Tip-envelope exponent (see src/string_sim.h): lowering it widens
        # the affected region toward the pole. 4.0 -> 2.8 is +28% reach.
        self.WRINKLE_ENV = 2.8
        self._wrinkle_cos = None
        self._wrinkle_sin = None
        self._wrinkle_phase = np.zeros(4)
        # ---- mid-band ripples (see src/string_sim.h for the rationale) ----
        # The gust sites cover 1.7-6 waves across the cloth and wrinkle 25-64;
        # measured spectra showed nothing in between. Built like wrinkle and
        # for the same reason (these wavenumbers are only reachable ON
        # RESONANCE), but locked to the MID-cloth wave speed c*2^(taper/4)
        # with a raised-cosine bump centred where that lock is resonant.
        # Deliberately NOT scaled by wave_density: mid_waves is an absolute
        # count of waves across the span, set by the Wind knob.
        # mid_ripple = 0 (default) skips the stage entirely.
        self.mid_ripple = 0.0
        self.mid_waves = 10.0
        self._mid_cos = None
        self._mid_sin = None
        self._mid_phase = np.zeros(4)
        self._tables_mid = -1.0
        self.flap_limit = 1e9  # Force semantics: flap peak ceiling; 1e9 = off
        self._flap_gain = 1.0
        self._wind_k = 0.1
        self._wind_phase = 0.0
        self._wind_env = None
        self._density_taper = 0.0
        self._m_inv = np.ones(self.n)
        self._damping_mult = np.ones(self.n)
        self._cloth_length = 1.0
        # Walls: hard boundaries as fractions of the node array. Defaults
        # 0 and 1 are a no-op (see src/string_sim.h for why they are hard
        # and why the DC/tilt blockers must be windowed).
        self._wall_lo = 0.0
        self._wall_hi = 1.0
        self._wall_i0 = 0
        self._wall_i1 = self.n - 1
        self._wind_exponent = 1.8
        # ---- vortex shedding / flutter (the "alive" mechanism) ----
        # A van der Pol WAKE OSCILLATOR per node (see src/string_sim.h for
        # the full rationale):
        #   q'' = -eps*w_i*(q^2 - 1)*q' - w_i^2*q + coupling*v_i
        #   F_i = shed_gain * q_i * wind_env_i
        # The (q^2 - 1) term is negative damping below |q| = 1 and positive
        # above, so q is a genuine limit cycle: the string flaps with NO
        # external gusting. w_i carries a gradient along the cloth so
        # neighbours detune and the wake forms vortex CELLS rather than one
        # global in-phase breath. The coupling term produces lock-in.
        # shed_gain = 0 (default) disables the stage entirely.
        self.shed_gain = 0.0
        self.shed_freq_hz = 3.0
        self.shed_eps = 0.3
        self.shed_coupling = 0.02
        # Seeded wake wavelength, in wavelengths per cloth span. The wake is
        # given an OUTWARD travelling phase ramp rather than a frequency
        # gradient: van der Pol is neutral in phase, so identical autonomous
        # oscillators keep whatever phase relationship they start with. A
        # frequency gradient instead makes a ramp that steepens with time and
        # ran INWARD (see src/string_sim.h for the full rationale).
        self.shed_waves_per_cloth = 1.5
        # q = qd = 0 is an UNSTABLE FIXED POINT of the van der Pol
        # oscillator: with both exactly zero and the string at rest it never
        # departs, so a deterministic nonzero seed is required to start it.
        self.SHED_SEED = 0.01
        # Gust-field spatial tables (see src/string_sim.cpp): the gust sum is
        # sin(psi_s - k_s*p_i) and the k_s*p_i part is time-invariant, so
        # cos/sin of it are tabulated. Per-tick cost becomes 8 sin + 8 cos
        # TOTAL instead of 8 transcendental calls PER NODE.
        self._site_cos = None
        self._site_sin = None
        self._tables_density = -1.0
        self._q = np.full(self.n, self.SHED_SEED)
        self._qd = np.zeros(self.n)
        self._cloth_d = np.zeros(self.n)

        # Tilt blocker (see src/string_sim.h): the DC blocker removes mode 0
        # because tension is blind to uniform displacement; mode 1 (a linear
        # TILT) has the same pathology — the Laplacian of a linear ramp is
        # zero in the interior — so it rings as a slow global rocking that
        # outlives every real wave and, under wind, eats most of the +-1
        # range. Removing the least-squares linear trend every tick kills it.
        self.remove_tilt = True
        # sum of (i - (n-1)/2)^2 = n(n^2-1)/12, the tilt projection denominator
        self._tilt_sxx = self.n * (self.n * self.n - 1.0) / 12.0

    # ---- writer interfaces (everything is a writer to the same nodes) ----

    def displace(self, value_at_node):
        """Set positions directly from an array (drawing / shape deposit).
        Velocities are zeroed, as if the string was held and released."""
        assert len(value_at_node) == self.n
        self.u[:] = value_at_node
        self.v[:] = 0.0

    def pluck(self, node, amount=1.0, sigma=4.0):
        """Displace with a gaussian bump centered on a node."""
        idx = np.arange(self.n)
        self.displace(amount * np.exp(-0.5 * ((idx - node) / sigma) ** 2))

    def set_force_field(self, field):
        """Persistent external force per node (the 'wind'). Stays applied
        until changed or cleared."""
        assert len(field) == self.n
        self.force[:] = field

    def set_wave_speed(self, c):
        """Re-tension the string live (clamped to the CFL limit)."""
        self.c2 = min(c, 1.0 / self.dt) ** 2

    def set_flagpole_wind(self, phase_velocity_nodes_s=192.0, wavelength_nodes=64.0,
                          envelope_exponent=1.8):
        """Configure the cloth-wind force model. Knob semantics (project
        decision): WIND = phase velocity of the flapping pattern (nodes/s),
        FORCE = flap-distance ceiling (flap_limit). Wind generates along the
        whole length via three distributed gust sites; wind_direction lets
        the pattern run low->high as well as high->low."""
        self.force_model = 1
        self.wind_phase_speed = phase_velocity_nodes_s
        self._wind_exponent = envelope_exponent
        self._wind_k = 2 * np.pi / max(8.0, wavelength_nodes)
        # Distributed gust sites (travel coordinate: nodes from origin anchor):
        # five sites, incommensurate temporal multipliers and per-site
        # spatial scales — rich, smooth-traveling wind detail.
        self._site_pos = np.array([0.12, 0.32, 0.52, 0.72, 0.9]) * (self.n - 1)
        self._site_phase = np.zeros(8)
        self._site_mult = np.array([1.0, 0.787, 1.618, 1.264, 0.755, 1.414, 2.236, 0.9])
        self._site_k = np.array([1.0, 1.21, 0.83, 1.55, 0.72, 2.05, 1.35, 2.6])
        self._recompute_cloth()

    def is_wind_model(self):
        """Both wind models share the cloth geometry (envelope, taper,
        length, origin, gust sites); only the force law differs."""
        return self.force_model in (1, 2)

    def set_cloth_aero(self, phase_velocity_nodes_s=192.0, wavelength_nodes=64.0,
                       envelope_exponent=1.8):
        """Configure the cloth AERO model: same gust geometry as the
        flagpole wind, but the pattern is a transverse wind velocity field
        and the force comes from relative velocity (see __init__ notes).
        Force knob = wind_speed; no flap ceiling to tune."""
        self.force_model = 2
        self.wind_phase_speed = phase_velocity_nodes_s
        self._wind_exponent = envelope_exponent
        self._wind_k = 2 * np.pi / max(8.0, wavelength_nodes)
        self._site_pos = np.array([0.08, 0.22, 0.38, 0.52, 0.66, 0.78, 0.88, 0.96]) * (self.n - 1)
        self._site_phase = np.zeros(8)
        self._site_mult = np.array([1.0, 0.787, 1.618, 1.264, 0.755, 1.414, 2.236, 0.9])
        self._site_k = np.array([1.0, 1.21, 0.83, 1.55, 0.72, 2.05, 1.35, 2.6])
        self._recompute_cloth()

    def set_walls(self, lo, hi):
        lo = min(max(lo, 0.0), 1.0)
        hi = min(max(hi, 0.0), 1.0)
        if hi < lo:
            lo, hi = hi, lo
        min_span = 4.0 / max(1.0, self.n - 1.0)
        if hi - lo < min_span:
            hi = min(1.0, lo + min_span)
        self._wall_lo, self._wall_hi = lo, hi
        self._recompute_cloth()

    def set_cloth_length(self, fraction):
        """Where the animation TERMINATES, as a fraction of the node array
        measured from the anchored pole. The end node is NOT anchored — it
        moves freely and whips. Beyond the termination: no wind, heavy
        damping (the animation genuinely ends there)."""
        self._cloth_length = min(max(fraction, 0.1), 1.0)
        self._recompute_cloth()

    def set_wind_origin(self, origin):
        """Direction knob: which side the wind blows FROM. 0 = pole at the
        high end (whip at lows), 1 = pole at the low end (whip at highs)."""
        if self.wind_origin == origin:
            return
        self.wind_origin = origin
        self._recompute_cloth()

    def set_density_taper(self, octaves):
        """Heavy at the anchored pole, light toward the termination — the
        whip mechanism (energy flux into lighter mass). 0 = uniform. Clamped
        to keep the CFL condition valid at the lightest node."""
        c = np.sqrt(self.c2)
        max_ratio = (1.0 / self.dt) / c if c > 1e-9 else 4.0
        self._density_taper = min(octaves, max(0.0, np.log2(max(1.0, max_ratio))))
        self._recompute_cloth()

    def _recompute_cloth(self):
        """Cloth geometry measured from the origin pole (wind_origin: 0 =
        pole at the high end, 1 = pole at the low end). Cloth spans
        `cloth_length` of the array from the pole; the tail beyond the
        termination damps hard. The termination node is NEVER anchored."""
        low_pole = (self.wind_origin == 1)
        if self.is_wind_model():
            self.anchor_high = not low_pole
            self.anchor_low = low_pole
        self._wall_i0 = int(round(self._wall_lo * (self.n - 1)))
        self._wall_i1 = int(round(self._wall_hi * (self.n - 1)))
        self._wall_i0 = min(max(self._wall_i0, 0), self.n - 1)
        self._wall_i1 = min(max(self._wall_i1, self._wall_i0), self.n - 1)
        pole = self._wall_i0 if low_pole else self._wall_i1
        span_total = max(1.0, float(self._wall_i1 - self._wall_i0))
        idx = np.arange(self.n)
        p = (idx - pole) if low_pole else (pole - idx)
        inside = (idx >= self._wall_i0) & (idx <= self._wall_i1)
        in_cloth = inside & (p <= span_total * self._cloth_length + 0.5)

        env = np.zeros(self.n)
        minv = np.ones(self.n)
        mult = np.ones(self.n)
        if self.is_wind_model():
            d = np.minimum(p / max(1.0, span_total * self._cloth_length), 1.0)
            env[in_cloth] = d[in_cloth] ** self._wind_exponent
            minv[in_cloth] = 2.0 ** (self._density_taper * d[in_cloth])
            into_tail = np.abs(p - span_total * self._cloth_length) / max(1.0, 0.1 * self.n)
            mult[~in_cloth] = 1.0 + 24.0 * np.minimum(into_tail[~in_cloth], 1.0)
            minv[~in_cloth] = 0.02
        self._wind_env = env
        self._m_inv = minv
        self._damping_mult = mult
        # normalized distance from the pole (0..1), for the shedding gradient
        cd = np.zeros(self.n)
        if self.is_wind_model():
            cd[in_cloth] = d[in_cloth]
            cd[~in_cloth] = 1.0
        self._cloth_d = cd
        # Geometry or origin changed, so re-impose the outward phase ramp.
        self.seed_wake_phase(True)
        self.rebuild_gust_tables()
        self._site_pos = np.array([0.08, 0.22, 0.38, 0.52, 0.66, 0.78, 0.88, 0.96]) * span_total * self._cloth_length
        self._w_field = np.zeros(self.n)
        # Heaviest inverse mass bounds how hard drag can push a node — what
        # the explicit-integration stability guard is sized against.
        self._m_inv_max = max(1.0, float(np.max(minv)))

    def rebuild_gust_tables(self):
        """Tabulate the time-invariant spatial part of the gust field. Cheap
        but not free (8*n sin + 8*n cos), so it runs only when the geometry
        or wave_density changes, never per tick."""
        p = (np.arange(self.n) if self.wind_origin == 1
             else (self.n - 1 - np.arange(self.n))).astype(float)
        k = self._wind_k * self.wave_density * self._site_k      # (8,)
        arg = p[:, None] * k[None, :]                            # (n, 8)
        self._site_cos = np.cos(arg)
        self._site_sin = np.sin(arg)
        # Wrinkle tables, with the d^4 tip envelope baked in so the per-node
        # work stays a plain multiply-add.
        span = max(1.0, (self.n - 1.0) * self._cloth_length)
        d = np.clip(p / span, 0.0, 1.0)
        tip = np.power(d, self.WRINKLE_ENV)   # see header: +30% reach
        # Anti-alias clamp: the finest wrinkle must stay resolvable on the
        # node grid (>= 8 nodes per cycle), or the stencil aliases it
        # instead of carrying it. The set is scaled together.
        k_top = self._wind_k * self.wave_density * self.WRINKLE_K[3]
        k_limit = 2 * np.pi / 8.0
        k_scale = (k_limit / k_top) if k_top > k_limit else 1.0
        self._wrinkle_k = self._wind_k * self.wave_density * self.WRINKLE_K * k_scale
        warg = p[:, None] * self._wrinkle_k[None, :]
        self._wrinkle_cos = tip[:, None] * np.cos(warg)
        self._wrinkle_sin = tip[:, None] * np.sin(warg)
        # Mid-band tables. Absolute wavenumber (mid_waves * MID_K waves across
        # the span), with a raised-cosine bump centred at mid-cloth baked in.
        mid_limit = 2 * np.pi / 8.0
        mk = 2 * np.pi * self.mid_waves * self.MID_K / span
        self._mid_k = np.minimum(mk, mid_limit)
        bump = 0.5 * (1.0 - np.cos(2 * np.pi * d))
        marg = p[:, None] * self._mid_k[None, :]
        self._mid_cos = bump[:, None] * np.cos(marg)
        self._mid_sin = bump[:, None] * np.sin(marg)
        self._tables_density = self.wave_density
        self._tables_mid = self.mid_waves

    def seed_wake_phase(self, preserve_amplitude):
        """(Re)impose the outward travelling phase ramp on the wake bank.
        Called whenever cloth geometry or wind origin changes, so flipping
        Direction flips the travel direction. preserve_amplitude keeps each
        oscillator's magnitude and only re-phases it, so dragging Length or
        Direction does not restart the flutter from silence."""
        w = 2 * np.pi * self.shed_freq_hz
        p = (np.arange(self.n) if self.wind_origin == 1
             else (self.n - 1 - np.arange(self.n))).astype(float)
        span = max(1.0, (self.n - 1.0) * self._cloth_length)
        k = 2 * np.pi * self.shed_waves_per_cloth / span
        if preserve_amplitude:
            qd_over_w = self._qd / w if w > 1e-9 else np.zeros(self.n)
            r = np.sqrt(self._q * self._q + qd_over_w * qd_over_w)
            r = np.maximum(r, self.SHED_SEED)   # never seed below the seed
        else:
            r = np.full(self.n, self.SHED_SEED)
        phi = -k * p
        self._q = r * np.cos(phi)
        self._qd = -r * w * np.sin(phi)

    def _update_wind_forces(self):
        if not self.is_wind_model():
            return
        # Explicit integration of quadratic drag is stable only while the
        # drag's effective damping rate stays well under 1/dt. That rate is
        # bounded by aero_rho_cd * |v_rel| * m_inv_max, so cap |v_rel| to
        # keep rate*dt <= 0.5. Recomputed per tick (both inputs are live).
        if self.force_model == 2:
            denom = self.aero_rho_cd * self._m_inv_max * self.dt
            self._aero_vrel_max = 0.5 / denom if denom > 1e-12 else 1e9
        # wave_density is live (the Waves knob), so the tables follow it -
        # but only on change, never per tick.
        if (self._site_cos is None or self._tables_density != self.wave_density
                or self._tables_mid != self.mid_waves
                or self._site_cos.shape[0] != self.n):
            self.rebuild_gust_tables()
        self._site_phase += (self._wind_k * self.wind_phase_speed
                             * self._site_mult * self.dt)
        # Entire time-varying part: 16 transcendental calls per tick,
        # independent of node count.
        k = self._wind_k * self.wave_density * self._site_k
        psi = self._site_phase + k * self._site_pos
        sp = np.sin(psi)
        cp = np.cos(psi)
        # Accumulate in site order 0..7 so each node's additions happen in
        # the same sequence as the C++ inner loop (bit-exactness).
        w = np.zeros(self.n)
        for s in range(8):
            w += sp[s] * self._site_cos[:, s] - cp[s] * self._site_sin[:, s]
        if self.force_model == 2:
            # transverse wind VELOCITY (nodes/s) — same units as self.v
            self._w_field = self.wind_speed * self._wind_env * (w / 8.0)
            if self.wrinkle != 0.0:
                # Phase velocity locked to the tip wave speed c*sqrt(2^taper)
                # so these drive short tip waves ON resonance. c2 is live
                # (Tension), so this is recomputed per tick - 8 scalars.
                c_tip = np.sqrt(self.c2) * 2.0 ** (0.5 * self._density_taper)
                self._wrinkle_phase = (self._wrinkle_phase
                                       + self._wrinkle_k * c_tip * self.dt)
                wsp = np.sin(self._wrinkle_phase)
                wcp = np.cos(self._wrinkle_phase)
                ww = np.zeros(self.n)
                for j in range(4):
                    ww += (wsp[j] * self._wrinkle_cos[:, j]
                           - wcp[j] * self._wrinkle_sin[:, j])
                wr_amp = (self.wrinkle_speed if self.wrinkle_speed >= 0.0
                          else self.wind_speed)
                self._w_field = self._w_field + wr_amp * self.wrinkle * (ww / 4.0)
            if self.mid_ripple != 0.0:
                # Phase velocity locked to the MID-cloth wave speed, the same
                # resonance trick wrinkle uses at the tip.
                c_mid = np.sqrt(self.c2) * 2.0 ** (0.25 * self._density_taper)
                self._mid_phase = (self._mid_phase
                                   + self._mid_k * c_mid * self.dt)
                msp = np.sin(self._mid_phase)
                mcp = np.cos(self._mid_phase)
                mm = np.zeros(self.n)
                for j in range(4):
                    mm += (msp[j] * self._mid_cos[:, j]
                           - mcp[j] * self._mid_sin[:, j])
                self._w_field = (self._w_field
                                 + self.wind_speed * self.mid_ripple * (mm / 4.0))
            self.force[:] = 0.0
        else:
            self.force[:] = self.wind_strength * self._wind_env * (w / 8.0)

        # Vortex shedding: integrate the per-node wake oscillator and add its
        # force. Skipped at shed_gain 0 so the frozen scenarios are
        # bit-identical and the stage costs nothing.
        if self.shed_gain != 0.0:
            wi = 2 * np.pi * self.shed_freq_hz
            q, qd = self._q, self._qd
            qdd = (-self.shed_eps * wi * (q * q - 1.0) * qd
                   - wi * wi * q + self.shed_coupling * self.v)
            self._qd = qd + self.dt * qdd
            self._q = q + self.dt * self._qd
            self.force[:] += self.shed_gain * self._q * self._wind_env

    def clear_forces(self):
        self.force[:] = 0.0

    # ---- boundary conditions ----

    def _enforce_anchors(self):
        if self.anchor_low:
            self.u[0] = 0.0
            self.v[0] = 0.0
        if self.anchor_high:
            self.u[-1] = 0.0
            self.v[-1] = 0.0

    # ---- integration ----

    def tick(self):
        """One sim step (velocity Verlet — bounded energy for oscillatory
        modes when wave_speed*dt <= 2; symplectic Euler would grow without
        bound, |lambda| = sqrt(1 + (omega*dt)^2) per step)."""
        # Flap limiter (Force semantics): starve the WIND at the ceiling —
        # the string state is never scaled, so waves stay true to the
        # simulation and deposits are never crushed.
        if self.flap_limit < 50.0:
            peak = float(np.max(np.abs(self.u)))
            target = (self.flap_limit / peak
                      if peak > self.flap_limit and peak > 1e-9 else 1.0)
            alpha = ((1.0 - np.exp(-self.dt / 0.005)) if target < self._flap_gain
                     else 1.0 - np.exp(-self.dt / 0.75))
            self._flap_gain += alpha * (target - self._flap_gain)
        else:
            self._flap_gain = 1.0
        self._update_wind_forces()
        u, v, f, c2 = self.u, self.v, self.force, self.c2
        dt, k, d = self.dt, self.stiffness, self.damping

        def accel(u_now, v_now):
            # Interior: 4th-order Laplacian — far lower numerical dispersion
            # than the 3-point stencil, so traveling shapes arrive intact.
            # Near-edge nodes and ends: 2nd-order; free ends use Neumann
            # (zero-slope ghosts) so waves reflect cleanly.
            lap = np.zeros(self.n)
            lap[2:-2] = (-u_now[:-4] + 16.0 * u_now[1:-3] - 30.0 * u_now[2:-2]
                         + 16.0 * u_now[3:-1] - u_now[4:]) / 12.0
            lap[1] = u_now[0] - 2.0 * u_now[1] + u_now[2]
            lap[-2] = u_now[-3] - 2.0 * u_now[-2] + u_now[-1]
            if not self.anchor_low:
                lap[0] = 2.0 * (u_now[1] - u_now[0])
            if not self.anchor_high:
                lap[-1] = 2.0 * (u_now[-2] - u_now[-1])
            a = c2 * lap - k * u_now + f * self._flap_gain
            # Aerodynamic drag against RELATIVE velocity (kClothAero): the
            # term that lets the string fight the wind. Uses v_now, not
            # self.v — Verlet evaluates acceleration twice per tick with
            # different velocities, and the stored value would lag a half
            # step and cost stability. 1/(1+s^2) is the angle-of-attack
            # factor: a steep element lies along the flow, catching less.
            if self.force_model == 2:
                slope = np.empty(self.n)
                slope[1:-1] = 0.5 * (u_now[2:] - u_now[:-2])
                slope[0] = u_now[1] - u_now[0]
                slope[-1] = u_now[-1] - u_now[-2]
                vrel = np.clip(self._w_field - v_now,
                               -self._aero_vrel_max, self._aero_vrel_max)
                a = a + self.aero_rho_cd * np.abs(vrel) * vrel / (1.0 + slope * slope)
            # Variable density: force terms divide by node mass (tension,
            # stiffness spring, external force). Damping is a rate (1/s)
            # scaled per node by the cloth-termination multiplier (the tail
            # damps hard so the animation ends at the cloth length).
            return self._m_inv * a - d * self._damping_mult * v_now

        a = accel(u, v)
        v_half = v + 0.5 * dt * a
        u += dt * v_half
        v = v_half + 0.5 * dt * accel(u, v_half)
        self.v[:] = v

        # DC blocker (see src/string_sim.cpp): tension is blind to uniform
        # displacement, so constrain the string to average zero — remove the
        # mean position and mean velocity every tick.
        # Both blockers operate on the ACTIVE WINDOW only (see the C++).
        a0, a1 = self._wall_i0, self._wall_i1
        an = a1 - a0 + 1
        self.u[a0:a1 + 1] -= self.u[a0:a1 + 1].mean()
        self.v[a0:a1 + 1] -= self.v[a0:a1 + 1].mean()

        # Tilt blocker: remove the least-squares linear trend (mode 1), same
        # reasoning as the mean. The centered basis is orthogonal to the
        # mean, so this composes cleanly with the DC removal above.
        if self.remove_tilt and an >= 4:
            mid = 0.5 * (a0 + a1)
            sxx = an * (an * an - 1.0) / 12.0
            xi = np.arange(a0, a1 + 1) - mid
            self.u[a0:a1 + 1] -= (float(np.sum(xi * self.u[a0:a1 + 1])) / sxx) * xi
            self.v[a0:a1 + 1] -= (float(np.sum(xi * self.v[a0:a1 + 1])) / sxx) * xi

        # HARD WALLS: everything outside the window is held at rest.
        if a0 > 0 or a1 < self.n - 1:
            self.u[:a0] = 0.0; self.v[:a0] = 0.0
            self.u[a1 + 1:] = 0.0; self.v[a1 + 1:] = 0.0

        self._enforce_anchors()

    def run(self, seconds, capture_every=0.0):
        """Advance the sim. Returns list of (time, u.copy()) snapshots;
        capture_every=0 means only the final state."""
        steps = int(round(seconds / self.dt))
        snap = max(1, int(round(capture_every / self.dt))) if capture_every else steps
        frames = []
        for s in range(steps):
            self.tick()
            if s % snap == 0:
                frames.append((s * self.dt, self.u.copy()))
        return frames

    def energy(self):
        """Proper conserved-quantity estimate: kinetic + stretch + spring.
        sum(u^2 + v^2) is NOT the energy of a wave equation — most energy
        lives in the stretch (gradient) term, weighted by c^2."""
        grad = np.diff(self.u)
        return float(np.sum(self.v * self.v)
                     + self.c2 * np.sum(grad * grad)
                     + self.stiffness * np.sum(self.u * self.u))
