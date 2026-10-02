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
    enum ForceModel { kStaticField = 0, kFlagpoleWind = 1 };
    int force_model = kStaticField;

    // Flagpole wind parameters. The knob semantics (project decision):
    // WIND = how fast the flapping pattern propagates (phase velocity,
    // nodes/s) — NOT modulation magnitude; FORCE = a flap-distance ceiling
    // (flap_limit, live-settable per block). Wind generates along the whole
    // length via three distributed gust sites; wind_direction lets the
    // pattern run low->high as well as high->low.
    void set_flagpole_wind(double phase_velocity_nodes_s, double wavelength_nodes,
                           double envelope_exponent);
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
    void recompute_cloth();
};

}  // namespace curves
