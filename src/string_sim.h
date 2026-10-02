// Curves — control string simulation.
//
// The physics engine that owns the nodes. Everything else (drawing, forces,
// audio analysis, shape generator) writes to it through the writer
// interfaces below; the filter bank reads positions() every block.
//
// Physics: 1D damped wave equation, finite difference, unit node spacing:
//   a_i = c^2 * (u[i-1] - 2u[i] + u[i+1])   <- wave propagation (tension)
//       - k * u[i]                          <- restoring force toward flat
//       - d * v[i]                          <- viscous damping
//       + f[i]                              <- external force field (wind)
//
// Integration: velocity Verlet (bounded energy for oscillatory modes;
// semi-implicit Euler is marginally unstable). CFL: wave_speed * dt <= 1.
//
// C++ port of prototype/string_sim.py — the Python module is the spec.

#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <utility>
#include <vector>

namespace curves {

struct StringParams {
    double tick_rate = 1000.0;  // sim ticks per second (sample_rate / divisor)
    double wave_speed = 300.0;  // nodes per second; higher = tauter string
    double damping = 1.5;       // 1/s velocity damping
    double stiffness = 4.0;     // 1/s^2 restoring force toward flat
};

class StringSim {
public:
    StringSim(int n_nodes, const StringParams& params);

    // ---- writer interfaces (everything is a writer to the same nodes) ----

    // Set positions directly (drawing / shape deposit). Zeroes velocities,
    // as if the string was held and released.
    void displace(const std::vector<double>& values);
    // Gaussian bump centered on a node.
    void pluck(int node, double amount = 1.0, double sigma = 4.0);
    // Persistent external force per node (the "wind"), applied every tick
    // until changed or cleared.
    void set_force_field(const std::vector<double>& field);
    void clear_forces();

    // ---- boundary conditions (the anchors) ----
    bool anchor_low = false;
    bool anchor_high = false;

    // ---- force models ----
    // kStaticField: f_ used as-is (persistent field set by the caller).
    // kFlagpoleWind: traveling forcing wave injected along the string —
    //   amplitude grows with distance from the anchored (high) end, phase
    //   travels toward the free (low) end. Small motion near the anchor,
    //   whip at the tip: cloth behavior, not a tweened ramp.
    // kClothAero: the wind is a VELOCITY field, not a force field. Force on
    //   each node comes from relative velocity (wind - node), quadratic:
    //   the string can fight the wind, drag rises as it moves faster, and
    //   amplitude saturates on its own. No flap limiter needed — the
    //   ceiling is physics, not a bolted-on gain.
    enum ForceModel { kStaticField = 0, kFlagpoleWind = 1, kClothAero = 2 };
    int force_model = kStaticField;
    // Both wind models share the cloth geometry (envelope, taper, length,
    // origin, gust sites); only the force law differs.
    bool is_wind_model() const {
        return force_model == kFlagpoleWind || force_model == kClothAero;
    }

    // Flagpole wind parameters. The knob semantics (project decision):
    // WIND = how fast the flapping pattern propagates (phase velocity,
    // nodes/s) — NOT modulation magnitude; FORCE = a flap-distance ceiling
    // (flap_limit, live-settable per block). Wind generates along the whole
    // length via three distributed gust sites; wind_direction lets the
    // pattern run low->high as well as high->low.
    void set_flagpole_wind(double phase_velocity_nodes_s, double wavelength_nodes,
                           double envelope_exponent);

    // ---- cloth aero model (kClothAero) ----
    // Same gust geometry as the flagpole wind, but the gust pattern is a
    // TRANSVERSE WIND VELOCITY field (nodes/s) rather than a force. Per node:
    //   v_rel = W_i - v_i                       (relative velocity)
    //   F_i   = aero_rho_cd * |v_rel|*v_rel / (1 + s_i^2)
    // where s_i = du/dx is the local slope. The 1/(1+s^2) term is the
    // angle-of-attack factor: a steeply sloped element lies along the flow
    // and catches less of it, a flat element catches the most. That is what
    // breaks up rigid-body motion into travelling waves and nodes.
    //
    // Consequences (project decisions):
    //  - Amplitude is self-limiting: drag grows as v^2 and opposes motion,
    //    so there is a natural equilibrium set by wind_speed. flap_limit is
    //    unnecessary in this mode and is left off.
    //  - A string at rest feels the full wind; a string already moving with
    //    the wind feels almost none. Drawn deposits are never crushed.
    //  - The string is DC-free, so the SPATIALLY UNIFORM part of the drag is
    //    removed every tick. Wind cannot produce a static offset here (it
    //    never could). Visible sag comes from the envelope's shape.
    void set_cloth_aero(double phase_velocity_nodes_s, double wavelength_nodes,
                        double envelope_exponent);
    // Wind speed (nodes/s): the magnitude of the transverse wind velocity
    // field. This is the Force knob — amplitude rises monotonically with it
    // and saturates by drag, with no ceiling to tune. Live-settable.
    double wind_speed = 0.0;
    // Lumped aerodynamic coefficient (0.5*rho*Cd*A as one number). Not a
    // measurable physical
    // constant here: the x-axis is log-frequency, not space, so this is a
    // tuning knob with physical STRUCTURE, which is where its value lies.
    double aero_rho_cd = 0.6;
    double wind_strength = 0.0;
    double wind_phase_speed = 192.0;  // nodes/s, live-settable per block
    // Wave concentration (Waves knob, project decision): spatial density of
    // the wind pattern — how many waves fit along the cloth. 1.0 = base;
    // >1 packs more (smaller) waves in, <1 stretches them. Live-settable.
    double wave_density = 1.0;
    // Wind origin (Direction knob, project decision): which side the wind
    // blows FROM — 0 = pole at the high end (whip at lows), 1 = pole at the
    // low end (whip at HIGHS). Mirrors the entire cloth geometry. Pure
    // reverse-playback of the pattern is a separate queued option.
    int wind_origin = 0;

    // Cloth length (fraction of the node array, measured from the anchored
    // pole): where the animation TERMINATES. The end node is NOT anchored —
    // it moves freely and whips (user decision: pinning it kills the whip).
    // Beyond the termination, forcing is zero and damping is multiplied so
    // the animation genuinely ends there.
    // Cloth length (fraction of the node array, measured from the anchored
    // pole): where the animation TERMINATES. The end node is NOT anchored —
    // it moves freely and whips (user decision: pinning it kills the whip).
    // Beyond the termination, forcing is zero and damping is multiplied so
    // the animation genuinely ends there.
    void set_cloth_length(double fraction);
    double cloth_length() const { return cloth_length_; }
    void set_wind_origin(int origin);

    // Flap limiter — Force semantics (project decision): a ceiling on how
    // far the cloth may flap, NOT a drive gain. Shape-preserving: the whole
    // string is scaled by one smoothed gain so the true simulated peak
    // touches the limit — waves stay round, never flattened (user decision).
    // Inactive unless set.
    double flap_limit = 1e9;

    // Density taper in octaves: mass per node doubles every unit of the
    // normalized distance from the anchored (high) end... inverted — the
    // string is HEAVY at the anchor and LIGHT toward the free end, so wave
    // speed and acceleration grow toward the tip. That is the physical whip
    // mechanism (energy flux into lighter mass), not a tweened envelope.
    // 0 = uniform. Clamped to keep CFL valid at the current dt.
    void set_density_taper(double octaves);
    double density_taper_octaves() const { return density_taper_; }

    // ---- vortex shedding / flutter (the "alive" mechanism) ----
    // A van der Pol WAKE OSCILLATOR per node, after Facchinetti-style
    // vortex-induced-vibration models:
    //   q'' = -eps*w_i*(q^2 - 1)*q' - w_i^2*q + coupling*v_i
    //   F_i = shed_gain * q_i * wind_env_i
    // The (q^2 - 1) term is NEGATIVE damping below |q| = 1 and positive
    // above, so q is a genuine limit cycle: the string flaps with NO
    // external gusting at all. That is the difference between motion that
    // is driven and motion that is alive — and it is what lets the damping
    // knob behave, since a deposit no longer needs the wind to be re-excited.
    //
    // DIRECTION (user requirement, 2026-10-01): the waves must propagate AWAY
    // from the anchor, with the wind. The first design decorrelated the wake
    // with a FREQUENCY gradient (thinner elements shed faster), which was
    // wrong: a frequency gradient makes a phase ramp that steepens linearly
    // with time (dphi/dp = (dw/dp)*t), so its direction is locked by the sign
    // of dw/dp and it ran INWARD (measured outward fraction 0.203). Adding
    // convective coupling could not beat it, because the ramp keeps steepening.
    //
    // The fix exploits the fact that these oscillators are autonomous and
    // IDENTICAL: van der Pol is an attractor in amplitude but NEUTRAL in
    // phase, so whatever phase relationship they start with, they keep. So
    // the bank is seeded with an outward travelling phase ramp,
    //   q_i(t) = r * cos(w*t - 2*pi*p_i*waves_per_cloth/span)
    // where p_i is distance from the pole. Constant phase then moves to
    // larger p as t grows: away from the anchor, by construction, with the
    // wavelength set explicitly rather than emerging from a coupling gain.
    // Measured outward fraction 0.70-0.88, and the ramp survives 40 s runs.
    // Staggered phases also give low wake coherence (0.003), so the cells
    // survive without needing the frequency gradient at all.
    //
    // The coupling term is what produces LOCK-IN: structure velocity pulls
    // the wake's phase, so when shedding lands near a structural resonance
    // the two synchronize.
    //
    // NOTE ON UNITS: shed_freq_hz is a real modulation rate in Hz, and in
    // Curves that rate is directly audible — it is how fast the EQ bands
    // wobble. It is therefore tuned BY EAR in the musical band (roughly
    // 0.5-8 Hz; above ~20 Hz it stops being motion and starts making
    // sidebands), never derived from a fictional Reynolds number.
    //
    // shed_gain = 0 (default) disables the whole stage, so the frozen
    // validation scenarios are untouched.
    double shed_gain = 0.0;      // flutter amount; 0 = off
    double shed_freq_hz = 3.0;   // shedding rate (audible modulation rate)
    double shed_eps = 0.3;       // van der Pol nonlinearity (limit-cycle)
    double shed_coupling = 0.02; // structure velocity -> wake (lock-in)
    // Seeded wake wavelength, in wavelengths per cloth span. 1.5 measured
    // most consistent across cloth lengths (shorter cloths are reflection-
    // limited by the near-rigid tail, which is physics, not a bug).
    double shed_waves_per_cloth = 1.5;
    // q = qd = 0 is an unstable fixed point of the van der Pol oscillator, so
    // a deterministic nonzero seed is required or it never starts.
    static constexpr double kShedSeed = 0.01;

    // Tilt blocker (project decision, 2026-10-01). The DC blocker removes
    // mode 0 (uniform displacement) because tension is blind to it. Mode 1 —
    // a linear TILT across the spectrum — has the same pathology: the
    // Laplacian of a linear ramp is zero in the interior, so tension cannot
    // act on it either and only the artificial `stiffness` spring and the
    // two Neumann end nodes restore it. The result is a slow global rocking
    // (~0.6-0.9 Hz, barely tunable by stiffness) that outlives every real
    // wave and, under wind, is continuously re-excited until it consumes
    // most of the +-1 range. Removing the least-squares linear trend every
    // tick kills it. Semantic consequence, mirroring the DC decision: a
    // tilted curve is not a string state — spectral tilt belongs to a
    // dedicated control applied in the gain mapping, where it is static and
    // cannot ring.
    bool remove_tilt = true;

    // ---- integration ----
    void tick();
    // Advance and capture snapshots (time, positions).
    // capture_every == 0 captures only the final state.
    std::vector<std::pair<double, std::vector<double>>> run(double seconds,
                                                            double capture_every = 0.0);

    // ---- readers (the filter bank reads these) ----
    const std::vector<double>& positions() const { return u_; }
    const std::vector<double>& velocities() const { return v_; }

    // ---- live parameter setters (audio thread; cheap, per block) ----
    void set_damping(double d) { damping_ = d; }
    void set_stiffness(double k) { stiffness_ = k; }
    // Re-tensions the string. Clamped to the CFL limit for the current dt.
    void set_wave_speed(double c) {
        const double clamped = std::min(c, 1.0 / dt_);
        c2_ = clamped * clamped;
    }

    // Conserved-quantity estimate: kinetic + stretch + spring.
    // sum(u^2 + v^2) is NOT the energy of a wave equation — most energy
    // lives in the stretch term, weighted by c^2.
    double energy() const;

    int size() const { return n_; }

private:
    void compute_accel(std::vector<double>& a, const std::vector<double>& u_now,
                       const std::vector<double>& v_now) const;
    void update_wind_forces();
    void enforce_anchors();

    int n_;
    double dt_;
    double c2_;
    double damping_;
    double stiffness_;
    std::vector<double> u_;
    std::vector<double> v_;
    std::vector<double> f_;
    std::vector<double> w_field_;   // transverse wind VELOCITY per node (kClothAero)
    double m_inv_max_ = 1.0;        // heaviest inverse mass (aero stability guard)
    double aero_vrel_max_ = 1e9;    // |v_rel| cap, recomputed per tick from CFL
    std::vector<double> wind_env_;  // precomputed forcing envelope (pole -> termination)
    std::vector<double> m_inv_;     // inverse mass per node (density taper)
    std::vector<double> damping_mult_;  // per-node damping multiplier (tail = termination)
    double density_taper_ = 0.0;
    double envelope_exponent_ = 1.8;
    double cloth_length_ = 1.0;
    double wind_k_ = 0.1;           // spatial rad/node
    double site_phase_[8] = {};
    double site_pos_[8] = {};   // nodes from origin anchor
    // Eight gust sites (resolution upgrade): incommensurate temporal
    // multipliers and per-site spatial scales — rich, non-repeating,
    // smooth-traveling wind detail along the whole cloth. The high-k sites
    // inject small ripples that survive the trip to the termination.
    static constexpr double kSiteMult_[8] = {1.0, 0.787, 1.618, 1.264, 0.755, 1.414, 2.236, 0.9};
    static constexpr double kSiteFrac_[8] = {0.08, 0.22, 0.38, 0.52, 0.66, 0.78, 0.88, 0.96};
    static constexpr double kSiteK_[8] = {1.0, 1.21, 0.83, 1.55, 0.72, 2.05, 1.35, 2.6};
    double wind_phase_ = 0.0;
    double flap_gain_ = 1.0;  // smoothed limiter gain (fast attack, slow release)
    double tilt_sxx_ = 1.0;   // sum of centered x^2, for the tilt projection
    // Gust-field spatial tables, layout [node*8 + site] so one node's eight
    // sites are contiguous (16 separate arrays would thrash the cache).
    // The gust sum is sin(psi_s - k_s*p_i), and the k_s*p_i part does not
    // change with time, so cos(k_s*p_i) and sin(k_s*p_i) are precomputed and
    // the per-tick work becomes 8 sin + 8 cos TOTAL plus multiply-adds per
    // node, instead of 8 transcendental calls PER NODE. See the .cpp.
    std::vector<double> site_cos_, site_sin_;
    double tables_density_ = -1.0;   // wave_density the tables were built for
    std::vector<double> q_, qd_;    // wake-oscillator state per node
    std::vector<double> cloth_d_;   // normalized distance from the pole (0..1)
    void recompute_cloth();
    // (Re)impose the outward travelling phase ramp on the wake bank. Called
    // whenever the cloth geometry or wind origin changes, so flipping
    // Direction flips the travel direction. preserve_amplitude keeps each
    // oscillator's current magnitude and only re-phases it, so dragging
    // Length or Direction does not restart the flutter from silence.
    void seed_wake_phase(bool preserve_amplitude);
    // Rebuild the gust spatial tables. Cheap but not free (8*n sin + 8*n
    // cos), so it runs only when the geometry or wave_density changes, never
    // per tick.
    void rebuild_gust_tables();
};

}  // namespace curves
