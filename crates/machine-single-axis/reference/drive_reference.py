"""Independent reference for the closed loop of the servo drive on the single belt axis.

Written from docs/model-reference.md ("Structure of one time step", "Drive and motor", "Motion controller",
"Profile generator"), not from the Rust source, so that a place where the document and the code differ shows up.
The drive is discrete (cycle times, zero-order hold), so the reference matches the simulator sample for sample; the
mechanics and the winding between two drive samples go through SciPy's solve_ivp (DOP853, rtol 1e-10) instead of the
simulator's fixed-step RK4.

Stages, each a set of effects switched on (every other effect off), with some values changed (VALUES):
  1  rigid loss-free axis: position, velocity and current loop with the back-EMF, one point-to-point move
  2  + drive delays, velocity and torque filters, speed and torque feedforward, torque and drive current limits,
     torque saturation, the voltage limit (encoder resolution matches too, until at standstill a count that the two
     integrations put 1e-9 apart rounds the other way; it is left out so that the comparison holds to the end)
  3  + the elastic belt (stretch, damping, slack, mass), the gearbox (backlash, stiffness, damping, efficiency), guide
     friction (LuGre), motor friction, torque ripple and cogging, tooth meshing
  4  + winding heating and the DC link with its braking chopper

Writes stage<N>.csv next to this file; the Rust test tests/reference.rs runs the simulator with the `# set` lines of
the file's header and compares. Run from the repository root, with the pinned packages of
crates/machine-single-axis/reference/requirements.txt in a venv:

    python -m venv .venv
    .venv/Scripts/python -m pip install -r crates/machine-single-axis/reference/requirements.txt
    .venv/Scripts/python crates/machine-single-axis/reference/drive_reference.py 1 2 3 4

A stage with effects taken out, for finding where a difference comes from: drive_reference.py 3-gearbox_backlash
(writes stage3-gearbox_backlash.csv, which no test reads).
"""

import math
import sys
import tomllib
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp

HERE = Path(__file__).parent
CFG = tomllib.loads((HERE.parent / "default.toml").read_text(encoding="utf-8"))

STAGES = {
    1: {"back_emf"},
    2: {
        "back_emf", "drive_delay", "velocity_filter", "torque_filter", "velocity_feedforward", "torque_feedforward",
        "fine_interpolation", "torque_limit", "drive_current_limit", "torque_saturation", "voltage_limit",
    },
}
STAGES[3] = STAGES[2] | {
    "belt_stretch", "belt_damping", "belt_slack", "belt_mass", "guide_friction", "motor_friction", "gearbox_backlash",
    "gearbox_efficiency", "torque_ripple", "tooth_meshing",
}
STAGES[4] = STAGES[3] | {"winding_heating", "dc_link"}
# Values changed from default.toml per stage, so that the limits act within the move: the drive's current limit and the
# torque-speed curve while accelerating, the voltage limit at cruise speed (110.2 V against a back-EMF of 112.5 V at 250 rad/s).
VALUES = {
    2: {
        "ratings.drive_max_current_a": 0.5, "drive.peak_torque_nm": 0.3, "drive.rated_speed_rad_s": 150.0,
        "drive.no_load_speed_rad_s": 350.0, "drive.bus_voltage_v": 90.0,
    },
    # A tenth of the DC link's capacitance, so that braking from cruise charges it to the chopper's threshold; a winding
    # started warm with a short time constant, so that its temperature moves within the move.
    4: {
        "ratings.dc_link_capacitance_uf": 47.0, "motor.start_temperature_rise_k": 80.0,
        "motor.thermal_time_constant_s": 0.5,
    },
}
# The start is an eighth of a tooth pitch past a whole number of them, so that the tooth meshing loads the gearbox at
# rest and sets the side its flanks touch on (at a whole number the side would be left to rounding).
START, TARGET, DURATION = 0.100625, 0.4, 0.7  # m, m, s
EVERY = 4  # base steps between two rows of the trace
COLUMNS = [
    "time", "axis.x.position", "axis.x.velocity", "axis.x.position_setpoint", "axis.x.following_error",
    "motor.m.angle", "motor.m.speed", "motor.m.torque", "motor.m.torque_command", "motor.m.current",
    "motor.m.voltage", "motor.m.speed_command", "motor.m.following_error", "motor.m.angle_setpoint",
]
COLUMNS_4 = ["motor.m.dc_link_voltage", "motor.m.winding_temperature", "motor.m.braking_resistor_power"]


def scurve(d, v, a, j):
    """Durations and jerks of the seven segments of a move of d at the limits v, a, j (one that reaches cruise)."""
    if v * j >= a * a:
        tj, ta = a / j, v / a - a / j
    else:
        tj, ta = math.sqrt(v / j), 0.0
    tv = (d - v * (2 * tj + ta)) / v
    assert tv > 0, "the reference move has to reach its cruise speed"
    return [tj, ta, tj, tv, tj, ta, tj], [j, 0, -j, 0, -j, 0, j]


def profile_at(t, segments):
    """Position, speed and acceleration of the S-curve at t, from rest at 0."""
    s = v = a = 0.0
    for T, jerk in zip(*segments):
        h = min(max(t, 0.0), T)
        s, v, a = s + v * h + a * h * h / 2 + jerk * h**3 / 6, v + a * h + jerk * h * h / 2, a + jerk * h
        t -= T
        if t <= 0:
            break
    return s, v, a


def run(spec):
    """spec: a stage number, optionally with effects taken out: "2-drive_delay-torque_filter"."""
    stage, *without = spec.split("-")
    stage = int(stage)
    on = STAGES[stage] - set(without)
    effects = {name: name in on for name in CFG["effects"]}
    values = {"start.pose": [START], **VALUES.get(stage, {})}
    cfg = {table: dict(entries) if isinstance(entries, dict) else entries for table, entries in CFG.items()}
    for key, value in values.items():
        table, name = key.split(".")
        cfg[table][name] = value
    e = effects.__getitem__
    geo, mass, motor, gear, drive, rat, axis, prof = (
        cfg[k] for k in ("geometry", "mass", "motor", "gearbox", "drive", "ratings", "axis", "profile")
    )
    r, ratio = geo["pulley_radius_m"], gear["ratio"]
    m = mass["carriage_kg"]
    span, stroke = geo["span_min_m"], geo["stroke_m"]
    belt = cfg["belt"]
    # The whole belt runs at carriage speed: its mass sits on the pulley (model reference, single axis).
    belt_length = 4 * span + 2 * stroke
    j_pulley = cfg["pulley"]["inertia_kgm2"] + (belt["mass_kg_m"] * belt_length * r * r if e("belt_mass") else 0.0)
    j_rotor = motor["rotor_inertia_kgm2"] + gear["inertia_at_input_kgm2"]
    # Rigid model: the motor sees the rotor, the pulley and the carriage through the gearbox.
    j_motor = j_rotor + (j_pulley + m * r * r) / ratio**2
    to_motor = ratio / r  # motor rad per m
    kt, R, L = motor["torque_constant_nm_a"], motor["resistance_ohm"], motor["inductance_h"]
    # Pole pairs: the q current's reactance p w L i is the d axis's voltage (i_d = 0 without field weakening).
    pole_pairs = motor.get("pole_pairs", 0.0)
    ke = kt if e("back_emf") else 0.0
    i_max, kt_max = motor["max_current_a"], motor["torque_constant_at_max_current_nm_a"]
    sat = (1 - kt_max / kt) if e("torque_saturation") else 0.0

    def kt_of(i):
        return kt * (1 - sat * min(1.0, (i / i_max) ** 2))

    def torque_of(i):
        return i * kt_of(i)

    def current_for(tau):
        """Inverse of the saturation curve, limited to the maximum current (Newton from below)."""
        if sat == 0.0:
            return tau / kt
        top = i_max if sat <= 1 / 3 else i_max / math.sqrt(3 * sat)
        target = min(abs(tau), torque_of(top))
        i = target / kt
        for _ in range(100):
            f = torque_of(i) - target
            df = kt * (1 - 3 * sat * (i / i_max) ** 2) if i < i_max else kt * (1 - sat)
            step = f / df
            i -= step
            if abs(step) < 1e-15 * max(1.0, i):
                break
        return math.copysign(min(i, top), tau)

    dt = cfg["sim"]["dt_s"]
    n_prof = round(prof["cycle_s"] / dt)
    n_pos = round(drive["position_cycle_s"] / dt)
    n_vel = round(drive["velocity_cycle_s"] / dt)
    t_v = drive["velocity_cycle_s"]
    t_p = drive["position_cycle_s"]
    kv, kp, tn = drive["kv_1_s"], drive["velocity_kp_nm_s_rad"], drive["velocity_tn_s"]
    kpi, tni = drive["current_kp_v_a"], drive["current_tn_s"]
    w_v = drive["velocity_feedforward_factor"] if e("velocity_feedforward") else 0.0
    w_t = drive["torque_feedforward_factor"] if e("torque_feedforward") else 0.0
    a_v = 1 - math.exp(-t_v / drive["velocity_filter_s"]) if e("velocity_filter") else 1.0
    a_t = 1 - math.exp(-t_v / drive["torque_filter_s"]) if e("torque_filter") else 1.0
    beta = 1 - math.exp(-t_p / drive["precontrol_balance_s"]) if drive["precontrol_balance_s"] > 0 else 1.0
    counts = drive["encoder_counts_per_rev"] / (2 * math.pi)
    u_bus = drive["bus_voltage_v"]
    c_link = rat["dc_link_capacitance_uf"] * 1e-6
    u_threshold, p_chopper = rat["braking_threshold_v"], rat["braking_resistor_peak_w"]
    rise_rated, i_rated, tau_th = motor["rated_temperature_rise_k"], motor["rated_current_a"], motor["thermal_time_constant_s"]

    def resistance(rise):
        return R * (1 + 0.0039 * rise) if e("winding_heating") else R

    p_rated = resistance(rise_rated) * i_rated**2
    d_cur = drive["current_delay_cycles"] if e("drive_delay") else 0
    d_vel = drive["velocity_delay_cycles"] if e("drive_delay") else 0
    peak, w_rated, w0 = drive["peak_torque_nm"], drive["rated_speed_rad_s"], drive["no_load_speed_rad_s"]
    i_drive = rat["drive_max_current_a"]

    def torque_range(w):
        if not e("torque_limit"):
            lo, hi = -math.inf, math.inf
        else:
            fade = min(max((w0 - abs(w)) / (w0 - w_rated), 0.0), 1.0)
            # motoring in the direction of rotation fades, braking keeps the peak torque
            lo, hi = (-peak, peak * fade) if w >= 0 else (-peak * fade, peak)
        if e("drive_current_limit"):
            cap = torque_of(min(i_drive, i_max) if sat else i_drive)
            lo, hi = max(lo, -cap), min(hi, cap)
        return lo, hi

    vmax, amax, jmax = (axis["x"][k] for k in ("max_velocity_m_s", "max_acceleration_m_s2", "max_jerk_m_s3"))

    # Elastic model: belt sections, gearbox, friction (model reference: belt sections, guide friction, motor shaft,
    # tooth meshing, gearbox, single axis).
    EA, T0 = belt["stiffness_ea_n"], belt["pretension_n"]
    c_belt = belt["damping_n_s_m"] if e("belt_damping") else 0.0
    kappa = 1 + T0 / EA
    r_mat = r / kappa
    a0 = START / r
    mat0 = ((span + START) / kappa, (3 * span + 2 * stroke - START) / kappa)
    fx = cfg["friction_x"]
    f_c, f_s = fx["coulomb_n"], max(fx["stiction_n"], fx["coulomb_n"])
    sigma0, sigma1, b_x, v_s = fx["bristle_stiffness_n_m"], fx["bristle_damping_n_s_m"], fx["viscous_n_s_m"], fx["stribeck_m_s"]
    k_g, c_g = gear["stiffness_nm_rad"], gear["damping_nm_s_rad"]
    half_backlash = gear["backlash_rad"] / 2 if e("gearbox_backlash") else 0.0
    eta = gear["efficiency"] if e("gearbox_efficiency") else 1.0
    ripple, n_ripple = (motor["ripple_fraction"], motor["ripple_order"]) if e("torque_ripple") else (0.0, 0.0)
    cog, n_cog = (motor["cogging_nm"], motor["cogging_order"]) if e("torque_ripple") else (0.0, 0.0)
    f_tooth, pitch = (belt["tooth_force_n"], belt["tooth_pitch_m"]) if e("tooth_meshing") else (0.0, 1.0)

    def section(geo_len, geo_rate, mat, mat_rate):
        """Tension on the carriage and force on the belt material a pulley feeds in."""
        stretch = geo_len - mat
        t_el = EA * stretch / mat
        t = t_el + c_belt * (geo_rate - mat_rate)
        if e("belt_slack"):
            t = max(0.0, t)
        return t, (t + t_el * stretch / (2 * mat) if stretch >= 0 else t)

    def gearbox(twist, rate, w_out):
        """Torque in the gearbox and the torque it puts on the pulley."""
        beyond = abs(twist) - half_backlash
        if beyond <= 0:
            return 0.0, 0.0
        tau = math.copysign(k_g * beyond, twist) + c_g * rate
        if half_backlash > 0 and tau * twist < 0:
            tau = 0.0
        return tau, tau * eta ** math.tanh(tau * w_out / 0.5)

    def guide(v, z):
        g = f_c + (f_s - f_c) * math.exp(-((v / v_s) ** 2))
        dz = v - sigma0 * abs(v) * z / g
        return sigma0 * z + sigma1 * dz + b_x * v, dz

    def elastic_rates(_t, y, u, R):
        x, a, phi, v, w_a, w, z, i = y
        t1, m1 = section(span + x, v, mat0[0] + r_mat * (a - a0), r_mat * w_a)
        t2, m2 = section(3 * span + 2 * stroke - x, -v, mat0[1] - r_mat * (a - a0), -r_mat * w_a)
        f_f, dz = guide(v, z) if e("guide_friction") else (0.0, 0.0)
        tau_g, tau_out = gearbox(phi / ratio - a, w / ratio - w_a, w_a)
        tau_m = torque_of(i)
        shaft = (motor["friction_coulomb_nm"] * math.tanh(w / motor["friction_smoothing_rad_s"])
                 + motor["friction_viscous_nm_s_rad"] * w) if e("motor_friction") else 0.0
        mesh = f_tooth * math.sin(2 * math.pi * r * a / pitch)
        return [
            v, w_a, w,
            (t2 - t1 - f_f) / m,
            (tau_out + r_mat * (m1 - m2) + r * mesh) / j_pulley,
            (tau_m * (1 + ripple * math.sin(n_ripple * phi)) + cog * math.sin(n_cog * phi) - tau_g / ratio - shaft) / j_rotor,
            dz,
            (u - R * i - ke * kt_of(i) / kt * w) / L,
        ]

    def rigid_rates(_t, y, u, R):
        x, v, i = y
        w = v * to_motor
        return [v, torque_of(i) * to_motor / (j_motor * to_motor**2), (u - R * i - ke * kt_of(i) / kt * w) / L]

    elastic = e("belt_stretch")
    mechanics = elastic_rates if elastic else rigid_rates

    def rates(t, y, u, R):
        """The mechanics and the winding, and the step's electrical energy and copper loss."""
        i = y[-3]
        return [*mechanics(t, y[:-2], u, R), u * i, R * i * i]

    def motion(y):
        """Carriage position and speed, motor angle and speed, winding current."""
        if elastic:
            return y[0], y[3], y[2], y[5], y[7]
        return y[0], y[1], y[0] * to_motor, y[1] * to_motor, y[2]

    def at_rest(x):
        """Pulley and motor angle at rest with the carriage at x, and the motor torque that holds them: both belt
        sections at the pretension, the gearbox carrying the tooth meshing's torque with the twist torque / stiffness
        + backlash / 2 on its side (none without torque), the drive holding that and the cogging."""
        a = a0 + (x - START) / r
        gear = -r * f_tooth * math.sin(2 * math.pi * r * a / pitch)
        twist = math.copysign(abs(gear) / k_g + half_backlash, gear) if gear else 0.0
        phi = ratio * (a + twist)
        return a, phi, (gear / ratio - cog * math.sin(n_cog * phi)) / (1 + ripple * math.sin(n_ripple * phi))

    hold = 0.0
    if elastic:
        # The drive starts with the torque that holds the load where the carriage stands (a machine built at rest).
        a_start, phi0, hold = at_rest(START)
        y =np.array([START, a_start, phi0, 0.0, 0.0, 0.0, 0.0, current_for(hold), 0.0, 0.0])
    else:
        phi0 = START * to_motor
        y = np.array([START, 0.0, 0.0, 0.0, 0.0])
    rise = motor["start_temperature_rise_k"] if e("winding_heating") else 0.0
    # The DC link starts at the bus voltage (the holding current's L i^2 / 2 is taken from it only at an Enable).
    u_dc = u_bus
    burned = 0.0
    # The controller's setpoint starts where the motor stands.
    start = phi0 / to_motor
    segments = scurve(TARGET - start, vmax, amax, jmax)
    phi_set = w_set = a_set = tau_ff = 0.0
    set_pos = start
    prof_step = 0
    phi_cmd = phi0
    e_pos = speed_pos = 0.0
    enc_before = None
    w_f = 0.0
    i_w = tau_filt = tau_cmd = hold
    w_cmd = 0.0
    i_i = u = resistance(rise) * current_for(hold)
    u_d = 0.0
    tau_queue = [hold] * d_vel
    u_queue = [u] * d_cur
    rows = []
    steps = round(DURATION / dt)
    for n in range(steps + 1):
        t = n * dt
        x, v, phi, w, i = motion(y)
        if n % EVERY == 0:
            rows.append([
                # the phase voltage's amplitude: the winding's (u_q, u_d) times sqrt(2) / 3
                t, x, v, set_pos, e_pos / to_motor, phi, w, torque_of(i), tau_cmd, i, math.hypot(u, u_d) * math.sqrt(2) / 3,
                w_cmd, e_pos, phi_cmd,
            ] + ([u_dc, rise, burned / dt] if stage >= 4 else []))
        if n == steps:
            break
        enc = round(phi * counts) / counts if e("encoder_resolution") else phi
        # profile generator
        if n % n_prof == 0:
            s, sv, sa = profile_at(t, segments)
            set_pos = start + s
            phi_set, w_set, a_set = set_pos * to_motor, sv * to_motor, sa * to_motor
            tau_ff = j_motor * a_set
            prof_step = n
        since = (n - prof_step) * dt
        # position loop
        if n % n_pos == 0:
            target = phi_set + (w_set * since + a_set * since**2 / 2 if e("fine_interpolation") else 0.0)
            phi_cmd += beta * (target - phi_cmd)
            e_pos = phi_cmd - enc
            speed_pos = kv * e_pos
        # velocity loop
        if n % n_vel == 0:
            w_raw = (enc - enc_before) / t_v if (e("encoder_resolution") and enc_before is not None) else w
            if not e("encoder_resolution"):
                w_raw = w
            enc_before = enc
            w_f += a_v * (w_raw - w_f)
            w_cmd = speed_pos + w_v * (w_set + a_set * since)
            e_w = w_cmd - w_f
            wanted = kp * e_w + i_w + w_t * tau_ff
            tau_filt += a_t * (wanted - tau_filt)
            lo, hi = torque_range(w_f)
            new_cmd = min(max(tau_filt, lo), hi)
            grow = kp / tn * e_w * t_v
            if not ((tau_filt >= hi and grow > 0) or (tau_filt <= lo and grow < 0)):
                i_w += grow
            tau_queue.append(new_cmd)
            tau_cmd = tau_queue.pop(0)
        # current loop
        i_set = current_for(tau_cmd)
        u_max = math.sqrt(1.5) * u_dc if e("voltage_limit") else math.inf
        # braking above the speed where k_E w alone exceeds U: the circle (k_E w - R i)^2 + (p w L i)^2 <= U^2 leaves
        # braking currents from its smaller root up, the one the inverter's diodes force; with the torque limit the
        # drive does not command less braking than that (the nearest current R k_E w / a where the circle leaves none)
        back = ke * abs(w_f)
        if e("torque_limit") and w_f != 0.0 and math.isfinite(u_max) and back > u_max:
            x = pole_pairs * abs(w_f) * L
            a = R * R + x * x
            forced = min(max((R * back - math.sqrt(max(a * u_max * u_max - x * x * back * back, 0.0))) / a, 0.0), i_max)
            if forced > 0.0 and i_set * math.copysign(1.0, w_f) > -forced:
                i_set = -forced * math.copysign(1.0, w_f)
        e_i = i_set - i
        # the back-EMF it expects: at the filtered speed, with the constant saturation leaves at the winding's current
        wanted_u = kpi * e_i + i_i + ke * w_f * kt_of(i) / kt
        # the d axis first: its steady voltage, the reactance at the filtered speed; the q axis gets the rest of the circle
        u_d = -pole_pairs * w_f * L * i
        u_max = math.sqrt(max(u_max * u_max - u_d * u_d, 0.0))
        new_u = min(max(wanted_u, -u_max), u_max)
        if new_u == wanted_u:
            i_i += kpi / tni * e_i * dt
        u_queue.append(new_u)
        u = u_queue.pop(0)
        r_step = resistance(rise)
        y[-2:] = 0.0
        sol = solve_ivp(rates, (t, t + dt), y, method="DOP853", rtol=1e-10, atol=1e-13, args=(u, r_step))
        y = sol.y[:, -1]
        e_el, e_cu = y[-2:]
        # DC link: the capacitors give what the motor draws and take what it feeds back; the chopper burns what is
        # above its threshold, up to its peak power; the rectifier holds the bus voltage from below.
        if e("dc_link"):
            energy = 0.5 * c_link * u_dc**2 - e_el
            burned = min(max(energy - 0.5 * c_link * u_threshold**2, 0.0), p_chopper * dt)
            u_dc = max(math.sqrt(max(2 * (energy - burned) / c_link, 0.0)), u_bus)
        if e("winding_heating"):
            rise += dt * (rise_rated * (e_cu / dt) / p_rated - rise) / tau_th

    sets = [f"# set effects.{name} = {str(val).lower()}" for name, val in sorted(effects.items())]
    sets += [f"# set {key} = {value!r}" for key, value in values.items()]
    path = HERE / f"stage{spec}.csv"
    with path.open("w", encoding="utf-8", newline="\n") as f:
        f.write(f"# drive_reference.py stage {stage}: move {START} -> {TARGET} m, every {EVERY} base steps\n")
        f.write(f"# move {TARGET}\n# time {DURATION}\n# every {EVERY}\n")
        f.write("\n".join(sets) + "\n")
        f.write(",".join(COLUMNS + (COLUMNS_4 if stage >= 4 else [])) + "\n")
        for row in rows:
            f.write(",".join(f"{v:.12e}" for v in row) + "\n")
    print(f"wrote {path} ({len(rows)} rows)")


if __name__ == "__main__":
    for arg in sys.argv[1:] or ["1"]:
        run(arg)
