#include "string_sim.h"

#include <algorithm>
#include <cmath>

namespace curves {

StringSim::StringSim(int n_nodes, const StringParams& params)
    : n_(n_nodes),
      dt_(1.0 / params.tick_rate),
      c2_(params.wave_speed * params.wave_speed),
      damping_(params.damping),
      stiffness_(params.stiffness) {
    assert(n_nodes >= 4);
    // CFL stability: wave_speed * dt <= 1 at unit node spacing.
    assert(params.wave_speed * dt_ <= 1.0);
    u_.assign(n_, 0.0);
    v_.assign(n_, 0.0);
    f_.assign(n_, 0.0);
    q_.assign(n_, kShedSeed);
    qd_.assign(n_, 0.0);
    m_inv_.assign(n_, 1.0);      // uniform mass until a taper is configured
    damping_mult_.assign(n_, 1.0);  // uniform damping until cloth length is set
    // sum of (i - (n-1)/2)^2 = n(n^2-1)/12, the tilt projection denominator
    tilt_sxx_ = static_cast<double>(n_) *
                (static_cast<double>(n_) * n_ - 1.0) / 12.0;
}

void StringSim::displace(const std::vector<double>& values) {
    assert(static_cast<int>(values.size()) == n_);
    u_ = values;
    v_.assign(n_, 0.0);
    enforce_anchors();
}

void StringSim::pluck(int node, double amount, double sigma) {
    std::vector<double> bump(n_);
    for (int i = 0; i < n_; ++i) {
        const double x = static_cast<double>(i - node) / sigma;
        bump[i] = amount * std::exp(-0.5 * x * x);
    }
    displace(bump);
}

void StringSim::set_force_field(const std::vector<double>& field) {
    assert(static_cast<int>(field.size()) == n_);
    f_ = field;
}

void StringSim::clear_forces() {
    f_.assign(n_, 0.0);
}

void StringSim::compute_accel(std::vector<double>& a,
                              const std::vector<double>& u_now,
                              const std::vector<double>& v_now) const {
    a.assign(n_, 0.0);
    // Interior: 4th-order Laplacian — far lower numerical dispersion than
    // the 3-point stencil, so traveling shapes arrive intact and smooth
    // instead of smearing as they move down the string.
    for (int i = 2; i + 2 < n_; ++i)
        a[i] = (-u_now[i - 2] + 16.0 * u_now[i - 1] - 30.0 * u_now[i]
                + 16.0 * u_now[i + 1] - u_now[i + 2]) / 12.0;
    // Near-edge nodes and ends: 2nd-order stencil; free ends use Neumann
    // (zero-slope ghosts) so waves reflect cleanly.
    a[1] = u_now[0] - 2.0 * u_now[1] + u_now[2];
    a[n_ - 2] = u_now[n_ - 3] - 2.0 * u_now[n_ - 2] + u_now[n_ - 1];
    if (!anchor_low)
        a[0] = 2.0 * (u_now[1] - u_now[0]);
    if (!anchor_high)
        a[n_ - 1] = 2.0 * (u_now[n_ - 2] - u_now[n_ - 1]);

    for (int i = 0; i < n_; ++i)
        a[i] = c2_ * a[i] - stiffness_ * u_now[i] + f_[i] * flap_gain_;
    // Aerodynamic drag against RELATIVE velocity (kClothAero). This is the
    // term that lets the string fight the wind: drag grows quadratically
    // with how fast the node moves through the air, so amplitude finds its
    // own equilibrium instead of needing a limiter. Evaluated with v_now,
    // not v_, because Verlet calls this twice per tick with different
    // velocities — using the stored v_ would make the feedback lag a half
    // step and cost stability.
    //   s = du/dx (local slope); 1/(1+s^2) is the angle-of-attack factor:
    //   a steep element lies along the flow and catches less of it.
    if (force_model == kClothAero) {
        for (int i = 0; i < n_; ++i) {
            const double s = (i == 0)        ? (u_now[1] - u_now[0])
                             : (i == n_ - 1) ? (u_now[n_ - 1] - u_now[n_ - 2])
                                             : 0.5 * (u_now[i + 1] - u_now[i - 1]);
            const double vrel = std::clamp(w_field_[i] - v_now[i],
                                           -aero_vrel_max_, aero_vrel_max_);
            a[i] += aero_rho_cd * std::abs(vrel) * vrel / (1.0 + s * s);
        }
    }
    // Variable density: force terms divide by node mass (tension, stiffness
    // spring, external force). Damping is a rate (1/s) scaled per node by
    // the cloth-termination multiplier (tail damps hard so the animation
    // ends at the cloth length).
    for (int i = 0; i < n_; ++i)
        a[i] = m_inv_[i] * a[i] - damping_ * damping_mult_[i] * v_now[i];
}

void StringSim::set_flagpole_wind(double phase_velocity_nodes_s,
                                   double wavelength_nodes,
                                   double envelope_exponent) {
    force_model = kFlagpoleWind;
    wind_phase_speed = phase_velocity_nodes_s;
    envelope_exponent_ = envelope_exponent;
    wind_k_ = 6.283185307179586 / std::max(8.0, wavelength_nodes);
    for (int s = 0; s < 8; ++s)
        site_pos_[s] = kSiteFrac_[s] * static_cast<double>(n_ - 1);
    recompute_cloth();
}

// Same cloth geometry as the flagpole wind; the force law is what differs
// (see the header). Gust sites double as the wind-VELOCITY pattern here.
void StringSim::set_cloth_aero(double phase_velocity_nodes_s,
                               double wavelength_nodes,
                               double envelope_exponent) {
    force_model = kClothAero;
    wind_phase_speed = phase_velocity_nodes_s;
    envelope_exponent_ = envelope_exponent;
    wind_k_ = 6.283185307179586 / std::max(8.0, wavelength_nodes);
    for (int s = 0; s < 8; ++s)
        site_pos_[s] = kSiteFrac_[s] * static_cast<double>(n_ - 1);
    recompute_cloth();
}

void StringSim::set_cloth_length(double fraction) {
    cloth_length_ = std::clamp(fraction, 0.1, 1.0);
    recompute_cloth();
}

void StringSim::set_wind_origin(int origin) {
    if (wind_origin == origin)
        return;
    wind_origin = origin;
    recompute_cloth();
}

// Rebuild the cloth-geometry arrays: forcing envelope (0 at the pole, max
// at the termination node, zero in the tail), density taper compressed into
// the cloth span (heavy at the pole, light at the termination — the whip),
// and the tail damping multiplier that makes the animation terminate. The
// end node is NOT anchored; it moves freely and whips (user decision).
void StringSim::recompute_cloth() {
    // Cloth geometry measured from the origin pole (wind_origin: 0 = pole at
    // the high end, 1 = pole at the low end). Cloth spans `cloth_length` of
    // the array from the pole; the tail beyond the termination damps hard.
    // The termination node is NEVER anchored — it whips freely.
    const bool low_pole = (wind_origin == 1);
    if (is_wind_model()) {
        anchor_high = !low_pole;
        anchor_low = low_pole;
    }
    const int pole = low_pole ? 0 : n_ - 1;
    const double span_total = static_cast<double>(n_ - 1);

    wind_env_.resize(n_);
    m_inv_.resize(n_);
    damping_mult_.resize(n_);
    w_field_.assign(n_, 0.0);
    cloth_d_.assign(n_, 0.0);
    // The wake oscillator sits on an UNSTABLE FIXED POINT at q = qd = 0: with
    // both exactly zero and the string at rest it never departs, so it needs
    // a deterministic seed to break away. Preserved across geometry changes
    // (cloth length, direction) so moving those knobs does not stall the
    // flutter mid-flight.
    if (static_cast<int>(q_.size()) != n_) {
        q_.assign(n_, kShedSeed);
        qd_.assign(n_, 0.0);
    }
    for (int i = 0; i < n_; ++i) {
        const double p = low_pole ? static_cast<double>(i)
                                  : static_cast<double>(pole - i);
        const bool in_cloth = p <= span_total * cloth_length_ + 0.5;
        if (!is_wind_model()) {
            wind_env_[i] = 0.0;
            m_inv_[i] = 1.0;
            damping_mult_[i] = 1.0;
            continue;
        }
        if (!in_cloth) {
            // tail beyond the termination: near-rigid and heavily damped,
            // so the animation genuinely ends at the cloth length
            wind_env_[i] = 0.0;
            cloth_d_[i] = 1.0;
            m_inv_[i] = 0.02;
            const double into_tail = std::abs(p - span_total * cloth_length_) /
                                     std::max(1.0, 0.1 * n_);
            damping_mult_[i] = 1.0 + 24.0 * std::min(1.0, into_tail);
        } else {
            // cloth: envelope and taper grow from the pole toward the
            // termination (light end = whip)
            const double d = std::min(1.0, p / std::max(1.0, span_total * cloth_length_));
            cloth_d_[i] = d;
            wind_env_[i] = std::pow(d, envelope_exponent_);
            m_inv_[i] = std::pow(2.0, density_taper_ * d);
            damping_mult_[i] = 1.0;
        }
    }
    for (int s = 0; s < 8; ++s)
        site_pos_[s] = kSiteFrac_[s] * span_total * cloth_length_;
    // Heaviest inverse mass bounds how hard drag can push a node, which is
    // what the explicit-integration stability guard is sized against.
    m_inv_max_ = 1.0;
    for (int i = 0; i < n_; ++i)
        m_inv_max_ = std::max(m_inv_max_, m_inv_[i]);
    // Geometry or origin changed, so re-impose the outward phase ramp.
    seed_wake_phase(true);
}

void StringSim::seed_wake_phase(bool preserve_amplitude) {
    if (static_cast<int>(q_.size()) != n_) {
        q_.assign(n_, kShedSeed);
        qd_.assign(n_, 0.0);
    }
    const double two_pi = 6.283185307179586;
    const double w = two_pi * shed_freq_hz;
    const bool low_pole = (wind_origin == 1);
    const double span = std::max(1.0, (n_ - 1.0) * cloth_length_);
    const double k = two_pi * shed_waves_per_cloth / span;
    for (int i = 0; i < n_; ++i) {
        const double p = low_pole ? static_cast<double>(i)
                                  : static_cast<double>(n_ - 1 - i);
        // amplitude of the current oscillator state, in q units
        double r = kShedSeed;
        if (preserve_amplitude) {
            const double qv = q_[i], qdv = (w > 1e-9) ? qd_[i] / w : 0.0;
            r = std::sqrt(qv * qv + qdv * qdv);
            if (r < kShedSeed) r = kShedSeed;   // never seed below the seed
        }
        const double phi = -k * p;
        q_[i] = r * std::cos(phi);
        qd_[i] = -r * w * std::sin(phi);
    }
}
void StringSim::update_wind_forces() {
    if (!is_wind_model())
        return;
    // Explicit integration of a quadratic drag term is only stable while the
    // drag's effective damping rate stays well under 1/dt. That rate is
    // bounded by aero_rho_cd * |v_rel| * m_inv_max, so cap |v_rel| to keep
    // rate*dt <= 0.5. Recomputed per tick because aero_rho_cd and the taper
    // are both live-settable. Physically this is a stall cap: the model
    // stops being meaningful at extreme relative speeds anyway.
    if (force_model == kClothAero) {
        const double denom = aero_rho_cd * m_inv_max_ * dt_;
        aero_vrel_max_ = (denom > 1e-12) ? 0.5 / denom : 1e9;
    }
    for (int s = 0; s < 8; ++s)
        site_phase_[s] += wind_k_ * wind_phase_speed * kSiteMult_[s] * dt_;
    const bool low_pole = (wind_origin == 1);
    for (int i = 0; i < n_; ++i) {
        const double p_i = low_pole ? static_cast<double>(i)
                                    : static_cast<double>(n_ - 1 - i);
        double w = 0.0;
        for (int s = 0; s < 8; ++s)
            w += std::sin(site_phase_[s]
                          - wind_k_ * wave_density * kSiteK_[s] * (p_i - site_pos_[s]));
        if (force_model == kClothAero) {
            // transverse wind VELOCITY (nodes/s) — same units as v_
            w_field_[i] = wind_speed * wind_env_[i] * (w / 8.0);
            f_[i] = 0.0;
        } else {
            f_[i] = wind_strength * wind_env_[i] * (w / 8.0);
        }
    }

    // Vortex shedding: integrate the per-node wake oscillator and add its
    // force. Skipped entirely at shed_gain 0 so the stage costs nothing and
    // the frozen scenarios are bit-identical. See the header for the model.
    if (shed_gain != 0.0) {
        const double two_pi = 6.283185307179586;
        for (int i = 0; i < n_; ++i) {
            const double wi = two_pi * shed_freq_hz;
            const double q = q_[i], qd = qd_[i];
            const double qdd = -shed_eps * wi * (q * q - 1.0) * qd
                               - wi * wi * q + shed_coupling * v_[i];
            qd_[i] = qd + dt_ * qdd;
            q_[i] = q + dt_ * qd_[i];
            f_[i] += shed_gain * q_[i] * wind_env_[i];
        }
    }
}
void StringSim::set_density_taper(double octaves) {
    // Inverse mass grows toward the termination (light) end. CFL: effective
    // wave speed c*sqrt(m_inv) must satisfy c_max * dt <= 1; clamp the
    // taper so the lightest node can't violate it.
    const double c = std::sqrt(c2_);
    const double max_ratio = (c > 1e-9) ? (1.0 / dt_) / c : 4.0;
    density_taper_ = std::min(octaves, std::log2(std::max(1.0, max_ratio)));
    recompute_cloth();
}

void StringSim::tick() {
    static thread_local std::vector<double> a, v_half;
    a.resize(n_);
    v_half.resize(n_);

    // Flap limiter (Force semantics): when the peak is at the ceiling, the
    // WIND is starved (forcing scaled toward zero) — the string state itself
    // is never scaled, so waves stay true to the simulation and drawn
    // deposits are never crushed. Fast attack (5 ms), slow release (750 ms).
    if (flap_limit < 50.0) {
        double peak = 0.0;
        for (int i = 0; i < n_; ++i)
            peak = std::max(peak, std::abs(u_[i]));
        const double target =
            (peak > flap_limit && peak > 1e-9) ? flap_limit / peak : 1.0;
        const double alpha = (target < flap_gain_)
                                 ? 1.0 - std::exp(-dt_ / 0.005)
                                 : 1.0 - std::exp(-dt_ / 0.75);
        flap_gain_ += alpha * (target - flap_gain_);
    } else {
        flap_gain_ = 1.0;
    }

    update_wind_forces();

    compute_accel(a, u_, v_);
    for (int i = 0; i < n_; ++i)
        v_half[i] = v_[i] + 0.5 * dt_ * a[i];
    for (int i = 0; i < n_; ++i)
        u_[i] += dt_ * v_half[i];

    compute_accel(a, u_, v_half);
    for (int i = 0; i < n_; ++i)
        v_[i] = v_half[i] + 0.5 * dt_ * a[i];

    // DC blocker: the wave equation is blind to uniform displacement (the
    // laplacian of a constant is zero — tension can never act on it), so a
    // drawn shape's mean component would ring forever at sqrt(stiffness)
    // when damping is low. Constrain the string to average zero: remove its
    // mean position and mean velocity every tick. Global level belongs to
    // the Drive parameter, not to the string's center of mass.
    double sum_u = 0.0, sum_v = 0.0;
    for (int i = 0; i < n_; ++i) {
        sum_u += u_[i];
        sum_v += v_[i];
    }
    const double mean_u = sum_u / n_, mean_v = sum_v / n_;
    for (int i = 0; i < n_; ++i) {
        u_[i] -= mean_u;
        v_[i] -= mean_v;
    }

    // Tilt blocker: remove the least-squares linear trend (mode 1) for the
    // same reason the mean is removed — tension is blind to a linear ramp,
    // so it rings as a slow global rocking that outlives every real wave.
    // The centered basis is orthogonal to the mean, so this composes cleanly
    // with the DC removal above. See the header for the full rationale.
    if (remove_tilt) {
        const double mid = 0.5 * (static_cast<double>(n_) - 1.0);
        double sxu = 0.0, sxv = 0.0;
        for (int i = 0; i < n_; ++i) {
            const double xi = static_cast<double>(i) - mid;
            sxu += xi * u_[i];
            sxv += xi * v_[i];
        }
        const double au = sxu / tilt_sxx_, av = sxv / tilt_sxx_;
        for (int i = 0; i < n_; ++i) {
            const double xi = static_cast<double>(i) - mid;
            u_[i] -= au * xi;
            v_[i] -= av * xi;
        }
    }

    enforce_anchors();
}

std::vector<std::pair<double, std::vector<double>>> StringSim::run(
    double seconds, double capture_every) {
    const int steps = static_cast<int>(std::lround(seconds / dt_));
    const int snap =
        capture_every > 0.0 ? std::max(1, static_cast<int>(std::lround(capture_every / dt_)))
                            : steps;
    std::vector<std::pair<double, std::vector<double>>> frames;
    frames.reserve(steps / snap + 1);
    for (int s = 0; s < steps; ++s) {
        tick();
        if (s % snap == 0)
            frames.emplace_back(s * dt_, u_);
    }
    return frames;
}

void StringSim::enforce_anchors() {
    if (anchor_low) {
        u_[0] = 0.0;
        v_[0] = 0.0;
    }
    if (anchor_high) {
        u_[n_ - 1] = 0.0;
        v_[n_ - 1] = 0.0;
    }
}

double StringSim::energy() const {
    // kinetic + stretch + spring (see header note on why sum(u^2+v^2) lies)
    double e = 0.0;
    for (int i = 0; i < n_; ++i)
        e += v_[i] * v_[i] + stiffness_ * u_[i] * u_[i];
    for (int i = 0; i + 1 < n_; ++i) {
        const double g = u_[i + 1] - u_[i];
        e += c2_ * g * g;
    }
    return e;
}

}  // namespace curves
