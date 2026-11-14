"""The checks of the catalogue's data: the numbers agree with their own physics and with each other where two
documents describe the same part. A value that landed in the wrong column or row breaks one of these.

Each check takes `tables` (path below `catalog/` -> rows as dicts of text) and returns a list of errors. A check whose
tables are not all there returns no error. Ported from the Rust tests in `crates/sim-machines/tests/`; the
exceptions for misprints in the vendor documents are the ones listed in `docs/catalogue.md`.
"""

import math

PI, SQRT_2 = math.pi, math.sqrt(2)
MIN = 60.0            # s per minute
ARCMIN = PI / 10800   # rad per arcmin


def num(text):
    """The number in `text`, None for ""."""
    return None if text == "" else float(text)


def need(row, column):
    try:
        return float(row[column])
    except (KeyError, ValueError):
        raise ValueError(f"{column} '{row.get(column, '')}' is not a number") from None


def close(a, b, tolerance):
    return abs(a - b) <= tolerance * abs(b)


def inside(x, lo, hi):
    # The slack absorbs the last bit of a bound converted to SI (5 arcmin printed is 5 arcmin in rad).
    return lo - 1e-12 * abs(lo) <= x <= hi + 1e-12 * abs(hi)


def have(tables, *paths):
    return all(p in tables for p in paths)


def by_part(rows):
    return {r["part_number"]: r for r in rows}


def festo_exct_values_agree_with_its_own_geometry(tables):
    """Festo EXCT: feed per turn = pi x pulley diameter, torque formula = the T-gantry's mechanics at 45 degrees."""
    if not have(tables, "festo/belt_axes.csv", "festo/belt_axis_torque_formula.csv"):
        return []
    axes, formulas, errors = tables["festo/belt_axes.csv"], tables["festo/belt_axis_torque_formula.csv"], []
    if len(axes) < 3:
        errors.append(f"only {len(axes)} sizes")
    names = {(a["series"], a["size"]) for a in axes}
    errors += [f"{f['series']}-{f['size']}: torque formula without an axis" for f in formulas if (f["series"], f["size"]) not in names]
    for a in axes:
        name = f"{a['series']}-{a['size']}"
        try:
            # One turn of the pulley feeds one circumference of belt.
            diameter = need(a, "pulley_effective_diameter_m")
            if not close(need(a, "feed_per_rev_m"), PI * diameter, 1e-3):
                errors.append(f"{name}: feed_per_rev_m")
            if not need(a, "max_no_load_torque_nm") < need(a, "max_pulley_torque_nm"):
                errors.append(f"{name}: max_no_load_torque_nm not below max_pulley_torque_nm")
            # The torque formula is the T-gantry's own mechanics at 45 degrees, where one motor carries
            # both axes: M = r/sqrt(2) m a for the load, J sqrt(2)/r a for the motor, m g r/2 to hold Z.
            r = diameter / 2
            own = [f for f in formulas if (f["series"], f["size"]) == (a["series"], a["size"])]
            if not own:
                errors.append(f"{name}: no torque formula")
            last = (0.0, 0.0, 0.0)
            for f in own:
                if not close(need(f, "load_factor_m"), r / SQRT_2, 0.01):
                    errors.append(f"{name}: load_factor_m")
                if not close(need(f, "motor_inertia_factor_1_m"), SQRT_2 / r, 0.01):
                    errors.append(f"{name}: motor_inertia_factor_1_m")
                if not close(need(f, "gravity_factor_nm_kg"), 9.81 * r / 2, 0.03):
                    errors.append(f"{name}: gravity_factor_nm_kg")
                # A longer Z axis is heavier.
                now = (need(f, "stroke_z_m"), need(f, "constant_kg_m"), need(f, "gravity_mass_kg"))
                if not all(n > l for n, l in zip(now, last)):
                    errors.append(f"{name}: Z strokes, constant_kg_m and gravity_mass_kg not in rising order")
                last = now
        except ValueError as e:
            errors.append(f"{name}: {e}")
    return errors


def siemens_s210_motors_are_plausible_and_their_converters_exist(tables):
    """S210 motors: overload, torque per current, rated power = torque x speed, paired converters exist and carry the current."""
    if not have(tables, "siemens/motors_s210.csv", "siemens/converters_s210.csv"):
        return []
    motors, converters, errors = tables["siemens/motors_s210.csv"], by_part(tables["siemens/converters_s210.csv"]), []
    if len(motors) < 90 or len(converters) < 24:
        errors.append(f"{len(motors)} motors, {len(converters)} converters")
    for name, c in converters.items():
        try:
            if not need(c, "max_output_current_a") > 2 * need(c, "rated_output_current_a"):
                errors.append(f"{name}: max_output_current_a not above twice rated_output_current_a")
            if not (need(c, "line_current_a") > 0 and need(c, "power_loss_w") > 0):
                errors.append(f"{name}: line_current_a or power_loss_w")
        except ValueError as e:
            errors.append(f"{name}: {e}")
    seen = set()
    for m in motors:
        name = m["part_number"]
        if name in seen:
            errors.append(f"{name} twice")
        seen.add(name)
        try:
            m0, i0 = need(m, "static_torque_nm"), need(m, "stall_current_a")
            if not (need(m, "max_torque_nm") > 1.5 * m0 and need(m, "max_current_a") > 1.5 * i0):
                errors.append(f"{name}: overload")
            # Torque per current: a servo motor of this class gives a few tenths to a few N m per ampere.
            if not inside(m0 / i0, 0.15, 4.5):
                errors.append(f"{name}: {m0 / i0:.3g} N m/A")
            if not need(m, "rotor_inertia_with_brake_kgm2") >= need(m, "rotor_inertia_kgm2"):
                errors.append(f"{name}: rotor_inertia_with_brake_kgm2")
            if not need(m, "mass_with_brake_kg") > need(m, "mass_kg"):
                errors.append(f"{name}: mass_with_brake_kg")
            supplies = 0
            for voltage, supply_columns in (("230v", ("230v_1ac", "230v_3ac")), ("400v", ("400v_3ac",))):
                if m[f"rated_speed_{voltage}_rad_s"] == "":
                    continue
                supplies += 1
                speed, torque, current = (need(m, f"rated_{q}_{voltage}_{u}") for q, u in (("speed", "rad_s"), ("torque", "nm"), ("current", "a")))
                # The catalogue prints a rated current of 3.75 A at 230 V for the 1FT2206-2AC, above its
                # stall current of 2.8 A and above its 3 A converter: most likely a misprint for 2.75 A.
                # The file keeps what is printed, so this one value is not checked.
                misprint = name == "1FT2206-2AC" and voltage == "230v"
                if not (speed <= need(m, "max_speed_rad_s") and torque <= m0):
                    errors.append(f"{name}: rated speed or torque at {voltage}")
                if not (misprint or current <= 1.05 * i0):
                    errors.append(f"{name}: rated_current_{voltage}_a")
                power = num(m[f"rated_power_{voltage}_w"])
                if power is not None and not close(power, torque * speed, 0.12):
                    errors.append(f"{name}: rated_power_{voltage}_w")
                # The converters the catalogue pairs with the motor exist and can carry its rated current
                # (the tightest pairing in the catalogue is a converter with 88 % of the motor's rated current).
                paired = 0
                for series in ("paired_drive_", "paired_drive_new_"):
                    for supply in supply_columns:
                        article = m[f"{series}{supply}"]
                        if article == "":
                            continue
                        paired += 1
                        if article not in converters:
                            errors.append(f"{name}: {series}{supply} {article} is not in converters_s210.csv")
                            continue
                        available = need(converters[article], "rated_output_current_a")
                        if not (misprint or available >= 0.85 * current):
                            errors.append(f"{name} needs {current} A, {article} gives {available} A")
                if not paired:
                    errors.append(f"{name}: no converter at {voltage}")
            if not supplies:
                errors.append(f"{name}: no rated data")
        except ValueError as e:
            errors.append(f"{name}: {e}")
    return errors


def siemens_s210_braking_rows_are_the_s210_converters(tables):
    """S210 braking data: one row for every S210 (6SL3210) converter of converters_s210.csv, and for no other."""
    if not have(tables, "siemens/braking_s210.csv", "siemens/converters_s210.csv"):
        return []
    braking = [r["part_number"] for r in tables["siemens/braking_s210.csv"]]
    converters = {r["part_number"] for r in tables["siemens/converters_s210.csv"] if r["part_number"].startswith("6SL3210")}
    errors = [f"{p} twice" for p in sorted(set(braking)) if braking.count(p) > 1]
    errors += [f"{p}: braking row, but no S210 converter in converters_s210.csv" for p in sorted(set(braking) - converters)]
    errors += [f"{p}: no braking row" for p in sorted(converters - set(braking))]
    return errors


def siemens_s120_motor_modules_are_plausible(tables):
    """S120 Motor Modules: base < rated < max current, type rating about 0.54 kW per ampere, D type above C type."""
    if not have(tables, "siemens/motor_modules_s120.csv"):
        return []
    modules, errors, seen = tables["siemens/motor_modules_s120.csv"], [], set()
    if len(modules) < 19:
        errors.append(f"only {len(modules)} modules")
    for m in modules:
        name = m["part_number"]
        if name in seen:
            errors.append(f"{name} twice")
        seen.add(name)
        try:
            rated, base, peak = need(m, "rated_output_current_a"), need(m, "base_load_output_current_a"), need(m, "max_output_current_a")
            if not (base < rated < peak <= 3 * rated):
                errors.append(f"{name}: currents")
            # The type rating is for an induction motor at 400 V: sqrt(3) 400 V I cos(phi) eta, about 0.54 kW per ampere.
            if not close(need(m, "rated_power_w"), 540 * rated, 0.05):
                errors.append(f"{name}: rated_power_w")
            if not need(m, "dc_link_capacitance_f") >= 110e-6 * (1 - 1e-12):
                errors.append(f"{name}: dc_link_capacitance_f")
            if not need(m, "power_loss_typical_w") <= need(m, "power_loss_max_w"):
                errors.append(f"{name}: power loss")
            # The D type is the one for short, high overloads.
            if m["duty_type"] == "D":
                for c in modules:
                    if c["duty_type"] == "C" and c["axes"] == m["axes"] and c["rated_output_current_a"] == m["rated_output_current_a"]:
                        if not peak > need(c, "max_output_current_a"):
                            errors.append(f"{name}: max_output_current_a not above the C type's")
                        break
        except ValueError as e:
            errors.append(f"{name}: {e}")
    return errors


def siemens_s120_motors_are_plausible_and_their_motor_modules_exist(tables):
    """S120 motors (1FK7, 1FT7): power = torque x speed, plausible constants, the paired Motor Module exists."""
    if not have(tables, "siemens/motors_s120.csv", "siemens/motor_modules_s120.csv"):
        return []
    motors, modules, errors, seen = tables["siemens/motors_s120.csv"], tables["siemens/motor_modules_s120.csv"], [], set()
    if len(motors) < 160:
        errors.append(f"only {len(motors)} motors")
    for m in motors:
        name = m["part_number"]
        if name in seen:
            errors.append(f"{name} twice")
        seen.add(name)
        try:
            speed, m0, rated, i0 = need(m, "rated_speed_rad_s"), need(m, "static_torque_nm"), need(m, "rated_torque_nm"), need(m, "stall_current_a")
            if not rated <= m0:
                errors.append(f"{name}: rated torque above static torque")
            # Power is torque times speed: the rated power from the rated torque, the calculated power from
            # the static one. Some rated values hold at a lower speed, which a footnote of the catalogue names.
            # The smallest motors are printed to one digit, hence the 15 W of slack.
            rated_at = speed if m["rated_values_at_speed_rad_s"] == "" else need(m, "rated_values_at_speed_rad_s")
            if not rated_at <= speed:
                errors.append(f"{name}: rated_values_at_speed_rad_s above rated speed")
            for column, torque, at in (("rated_power_w", rated, rated_at), ("calculated_power_w", m0, speed)):
                if not abs(need(m, column) - torque * at) <= 0.08 * torque * at + 15:
                    errors.append(f"{name}: {column} against {torque * at:.0f} W")
            if not inside(m0 / i0, 0.1, 6.0):
                errors.append(f"{name}: {m0 / i0:.3g} N m/A")
            pairs = need(m, "pole_pairs")
            if not (pairs == int(pairs) and 2 <= pairs <= 10):
                errors.append(f"{name}: pole_pairs")
            if not inside(need(m, "efficiency"), 0.5, 0.98):
                errors.append(f"{name}: efficiency")
            if not (need(m, "rotor_inertia_kgm2") > 0 and need(m, "mass_kg") > 0):
                errors.append(f"{name}: rotor_inertia_kgm2 or mass_kg")
            # The Motor Module the catalogue pairs with the motor carries its stall current. In the
            # module's article number "7" stands for Single or Double and the last character is open.
            current = need(m, "paired_drive_rated_current_a")
            if not current >= 0.9 * i0:
                errors.append(f"{name}: {i0} A at standstill on a {current} A Motor Module")
            article = m["paired_drive"].replace("-7", "-1", 1).rstrip(".")
            known = next((x for x in modules if x["part_number"].startswith(article)), None)
            if known is not None:
                if need(known, "rated_output_current_a") != current:
                    errors.append(f"{name}: paired_drive_rated_current_a against {known['part_number']}")
            # The 2017 catalogue still pairs some motors with the 45 A and 60 A modules that the
            # Catalog News replaced by C types; a module of that current must exist all the same.
            elif not any(num(x["rated_output_current_a"]) == current for x in modules):
                errors.append(f"{name}: no {current} A Motor Module")
        except ValueError as e:
            errors.append(f"{name}: {e}")
    return errors


def siemens_gearbox_rows_are_complete_and_plausible(tables):
    """Gearboxes NRB, NRK, NLC: rows complete and unique, a ratio has one number of stages, torques in order, values sensible."""
    errors = []
    for series in ("nrb", "nrk", "nlc"):
        path = f"siemens/gearbox_{series}.csv"
        if path not in tables:
            continue
        seen, stage_of = set(), {}
        rows = tables[path]
        for r in rows:
            name = f"{r['size']}-{r['motor_frame']}-{r['ratio']}"
            try:
                if not r["size"].lower().startswith(series):
                    errors.append(f"{name}: size not of {path}")
                if (r["size"], r["motor_frame"], r["ratio"]) in seen:
                    errors.append(f"{name} twice")
                seen.add((r["size"], r["motor_frame"], r["ratio"]))
                # A ratio belongs to one number of stages, and more stages give higher ratios.
                stages, ratio = need(r, "stages"), need(r, "ratio")
                if stage_of.setdefault(r["ratio"], stages) != stages:
                    errors.append(f"{name}: stages differ from other rows of ratio {r['ratio']}")
                if not (stages in (1, 2, 3) and ratio >= (3, 9, 60)[int(stages) - 1]):
                    errors.append(f"{name}: stages {stages:g} for ratio {ratio:g}")
                rated, peak, emergency = need(r, "rated_output_torque_nm"), need(r, "max_output_torque_nm"), need(r, "emergency_stop_output_torque_nm")
                if not 0 < rated < peak < emergency:
                    errors.append(f"{name}: torques out of order")
                if not inside(need(r, "efficiency_full_load"), 0.75, 0.99):
                    errors.append(f"{name}: efficiency_full_load")
                if not need(r, "torsional_stiffness_nm_rad") > 0:
                    errors.append(f"{name}: torsional_stiffness_nm_rad")
                # The manual leaves a few of these two open; where given they must be sensible.
                for column, lo, hi in (("backlash_rad", 5 * ARCMIN, 25 * ARCMIN), ("inertia_at_input_kgm2", 1e-6, 3e-3)):
                    if r[column] != "" and not inside(need(r, column), lo, hi):
                        errors.append(f"{name}: {column}")
            except ValueError as e:
                errors.append(f"{name}: {e}")
        if len(rows) <= 150:
            errors.append(f"{path}: only {len(rows)} rows")
    return errors


def _manual_agrees_with_s210(tables, path, least, misprints, shared_least, check_rated_torque):
    if not have(tables, path, "siemens/motors_s210.csv"):
        return []
    manual, catalogue, errors, shared = tables[path], by_part(tables["siemens/motors_s210.csv"]), [], 0
    if len(manual) < least:
        errors.append(f"only {len(manual)} motors")
    for m in manual:
        name = m["part_number"]
        try:
            if not inside(need(m, "thermal_time_constant_s"), 5 * MIN, 120 * MIN):
                errors.append(f"{name}: thermal_time_constant_s")
            power = need(m, "rated_torque_nm") * need(m, "rated_speed_rad_s")
            if name not in misprints and not abs(need(m, "rated_power_w") - power) <= 0.08 * power + 15:
                errors.append(f"{name}: rated_power_w")
            if check_rated_torque and not need(m, "rated_torque_nm") <= need(m, "static_torque_nm"):
                errors.append(f"{name}: rated torque above static torque")
            c = catalogue.get(name)
            if c is None:
                continue
            shared += 1
            for column, tolerance in (("static_torque_nm", 0.05), ("stall_current_a", 0.05), ("max_torque_nm", 0.05), ("max_current_a", 0.05), ("rotor_inertia_kgm2", 0.06), ("mass_kg", 0.06)):
                # The manual prints the inertia of the 1FK2308-3AB as "1,60,1"; the file leaves it empty.
                if name == "1FK2308-3AB" and column == "rotor_inertia_kgm2":
                    if m[column] != "":
                        errors.append(f"{name}: {column} is readable now, check it")
                    continue
                if not close(need(m, column), need(c, column), tolerance):
                    errors.append(f"{name}: {column} {m[column]} against {c[column]} in motors_s210.csv")
        except ValueError as e:
            errors.append(f"{name}: {e}")
    if shared < shared_least:
        errors.append(f"only {shared} motors in both files")
    return errors


def siemens_1fk2_data_agrees_between_the_s120_manual_and_the_s210_catalogue(tables):
    """1FK2 of the S120 manual: rated power = torque x speed, and the data agree with the S210 catalogue."""
    return _manual_agrees_with_s210(tables, "siemens/motors_1fk2_s120.csv", 33, (), 28, True)


def siemens_1ft2_data_agrees_between_the_s120_manual_and_the_s210_catalogue(tables):
    """1FT2 of the S120 manual: rated power = torque x speed, and the data agree with the S210 catalogue."""
    # For three motors the manual prints a rated speed that contradicts its own rated torque and power and
    # the S210 catalogue (3000 for 4500 r/min, 2000 for 3000 r/min). The file keeps what is printed, so
    # these rows are not checked.
    return _manual_agrees_with_s210(tables, "siemens/motors_1ft2_s120.csv", 36, ("1FT2105-4AH", "1FT2206-4AH", "1FT2208-4AF"), 30, False)


def siemens_1fk2_dynamic_braking_fits_the_motors(tables):
    """1FK2 dynamic braking: every motor exists, the resistor brakes harder and lowers the current."""
    if not have(tables, "siemens/dynamic_braking_1fk2.csv", "siemens/motors_1fk2_s120.csv"):
        return []
    braking, motors, errors = tables["siemens/dynamic_braking_1fk2.csv"], by_part(tables["siemens/motors_1fk2_s120.csv"]), []
    if len(braking) < 33:
        errors.append(f"only {len(braking)} rows")
    for b in braking:
        name = b["part_number"]
        try:
            if name not in motors:
                errors.append(f"{name}: not in motors_1fk2_s120.csv")
            if not need(b, "optimum_resistance_ohm") > 0:
                errors.append(f"{name}: optimum_resistance_ohm")
            # The optimum resistor is the one that brakes harder than the bare short circuit, and a peak is above an average.
            if not need(b, "braking_torque_rms_nm") <= need(b, "braking_torque_rms_with_resistor_nm") <= need(b, "max_braking_torque_nm"):
                errors.append(f"{name}: torques out of order")
            # A resistor in the circuit lowers the current. For the 1FK2306-2AC the manual prints 6.5 A with
            # and 6.1 A without. The file keeps what is printed, so this one row is not checked.
            without, with_ = need(b, "braking_current_rms_a"), need(b, "braking_current_rms_with_resistor_a")
            if name != "1FK2306-2AC" and not 0 < with_ <= without:
                errors.append(f"{name}: currents out of order")
        except ValueError as e:
            errors.append(f"{name}: {e}")
    return errors


def siemens_s120_line_modules_are_plausible(tables):
    """S120 Line Modules: power = DC link voltage x current, bigger ones have more capacitance, chopper power = U^2 / R."""
    if not have(tables, "siemens/line_modules_s120.csv"):
        return []
    modules, errors, seen = tables["siemens/line_modules_s120.csv"], [], set()
    if len(modules) < 13:
        errors.append(f"only {len(modules)} modules")
    errors += [f"{m['part_number']}: infeed_type {m['infeed_type']}" for m in modules if m["infeed_type"] not in ("smart", "active", "basic")]
    for kind in ("smart", "active", "basic"):
        last_power = last_capacitance = 0.0
        for m in (m for m in modules if m["infeed_type"] == kind):
            name = m["part_number"]
            if name in seen:
                errors.append(f"{name} twice")
            seen.add(name)
            try:
                rated, s6, peak = need(m, "rated_power_w"), need(m, "s6_power_w"), need(m, "max_power_w")
                if not rated < s6 < peak:
                    errors.append(f"{name}: powers")
                # Power is DC link voltage times DC link current; the catalogue rounds to whole amperes.
                if not close(need(m, "dc_link_current_600v_a"), rated / 600, 0.04):
                    errors.append(f"{name}: dc_link_current_600v_a")
                if m["dc_link_current_540v_a"] != "" and not close(need(m, "dc_link_current_540v_a"), rated / 540, 0.04):
                    errors.append(f"{name}: dc_link_current_540v_a")
                # The catalogue prints 138 A and 178 A for the 55 kW Smart Line Module, about 510 V at its
                # 71 kW and 91 kW, where every other module has 600 V. The file keeps what is printed, so
                # these two values are not checked.
                misprint = name == "6SL3130-6TE25-5AA3"
                for current, power in (("s6_dc_link_current_a", s6), ("max_dc_link_current_a", peak)):
                    if not (misprint or close(need(m, current), power / 600, 0.04)):
                        errors.append(f"{name}: {current}")
                # A bigger module has more capacitance, and loses one to two percent of what it converts.
                capacitance = need(m, "dc_link_capacitance_f")
                if not (rated > last_power and capacitance > last_capacitance):
                    errors.append(f"{name}: sizes not in rising order")
                last_power, last_capacitance = rated, capacitance
                if not inside(need(m, "power_loss_w") / rated, 0.005, 0.025):
                    errors.append(f"{name}: power_loss_w")
                # Only a Basic Line Module has a braking chopper, and the 100 kW one has none.
                if m["braking_threshold_v"] == "":
                    if kind == "basic" and rated != 100e3:
                        errors.append(f"{name}: no braking chopper")
                    continue
                if kind != "basic":
                    errors.append(f"{name}: braking chopper on a {kind} Line Module")
                # The chopper's peak power is what the smallest resistor takes at the threshold: U^2 / R.
                threshold, braking = need(m, "braking_threshold_v"), need(m, "max_braking_power_w")
                if not close(braking, threshold ** 2 / need(m, "min_braking_resistance_ohm"), 0.03):
                    errors.append(f"{name}: max_braking_power_w against U^2 / R")
                if not (close(braking, 2 * rated, 1e-9) and close(need(m, "continuous_braking_power_w"), 0.25 * rated, 1e-9)):
                    errors.append(f"{name}: braking powers against rated power")
            except ValueError as e:
                errors.append(f"{name}: {e}")
    return errors


def siemens_s120_braking_resistors_fit_their_modules(tables):
    """S120 braking: each resistor's chopper exists, peak power = U^2 / R, duty cycles within the continuous power."""
    if not have(tables, "siemens/braking_s120.csv", "siemens/line_modules_s120.csv"):
        return []
    braking, line_modules, errors, seen = tables["siemens/braking_s120.csv"], by_part(tables["siemens/line_modules_s120.csv"]), [], set()
    by_name = by_part(braking)
    if len(braking) < 5:
        errors.append(f"only {len(braking)} rows")
    for b in braking:
        name = b["part_number"]
        if name in seen:
            errors.append(f"{name} twice")
        seen.add(name)
        try:
            rated, peak = need(b, "continuous_power_w"), need(b, "peak_power_w")
            if not rated < peak:
                errors.append(f"{name}: powers")
            if b["kind"] == "braking_module":
                if not (inside(need(b, "activation_threshold_v"), 700, 800) and need(b, "dc_link_capacitance_f") > 0):
                    errors.append(f"{name}: activation_threshold_v or dc_link_capacitance_f")
                continue
            if b["kind"] != "braking_resistor":
                errors.append(f"{name}: kind {b['kind']}")
                continue
            # The chopper that switches the resistor is in the Braking Module or in a Basic Line Module.
            host = b["for_part_number"]
            if host in by_name:
                threshold, limit = need(by_name[host], "activation_threshold_v"), need(by_name[host], "peak_power_w")
            elif host in line_modules:
                lm = line_modules[host]
                if not need(b, "resistance_ohm") >= need(lm, "min_braking_resistance_ohm"):
                    errors.append(f"{name}: resistance below what {host} allows")
                threshold, limit = need(lm, "braking_threshold_v"), need(lm, "max_braking_power_w")
            else:
                errors.append(f"{name}: no {host}")
                continue
            if not peak <= limit:
                errors.append(f"{name}: more peak power than {host} switches")
            # At the threshold voltage the resistor takes U^2 / R. The 6SN1113-1AA00-0DA0 is printed with
            # 25 kW at 17 ohm, where 770 V give 35 kW. The file keeps what is printed, so it is not checked.
            at_threshold = threshold ** 2 / need(b, "resistance_ohm")
            if name != "6SN1113-1AA00-0DA0" and not close(peak, at_threshold, 0.05):
                errors.append(f"{name}: peak_power_w {peak:g} W against {at_threshold:.0f} W")
            # Over a duty cycle the peak power averages to no more than the continuous power.
            for load, cycle in (("load_duration_s", "cycle_duration_s"), ("load_duration_2_s", "cycle_duration_2_s")):
                if b[load] == "":
                    if b[cycle] != "" or load == "load_duration_s":
                        errors.append(f"{name}: duty cycle {load}")
                    continue
                if not peak * need(b, load) / need(b, cycle) <= 1.01 * rated:
                    errors.append(f"{name}: {load}")
        except ValueError as e:
            errors.append(f"{name}: {e}")
    return errors


def siemens_s120_data_sheets_agree_with_the_catalogue(tables):
    """1FK7/1FT7 data sheets against the catalogue: poles, torque constant, voltage constant, L / R, speeds."""
    if not have(tables, "siemens/motors_s120_electrical.csv", "siemens/motors_s120.csv"):
        return []
    sheets, motors, errors = tables["siemens/motors_s120_electrical.csv"], by_part(tables["siemens/motors_s120.csv"]), []
    if len(sheets) < 160:
        errors.append(f"only {len(sheets)} data sheets")
    for sheet in sheets:
        name = sheet["part_number"]
        motor = motors.get(name)
        if motor is None:
            errors.append(f"{name}: not in motors_s120.csv")
            continue
        try:
            if not sheet["data_sheet_part_number"].startswith(name):
                errors.append(f"{name}: data_sheet_part_number {sheet['data_sheet_part_number']}")
            # The 1FT7 High Dynamic motors (1FT706x-7 and 1FT708x-7) have 6 poles on their data sheets and 5 pole
            # pairs in the catalogue's table, the value of the Compact motors beside them. Both are kept as printed.
            high_dynamic = name.startswith("1FT70") and name[8:9] == "7"
            if not high_dynamic and need(sheet, "poles") != 2 * need(motor, "pole_pairs"):
                errors.append(f"{name}: poles against pole_pairs")
            # The torque constant is static torque over stall current, by Siemens' own definition.
            kt = need(sheet, "torque_constant_nm_a")
            if not close(kt, need(motor, "static_torque_nm") / need(motor, "stall_current_a"), 0.08):
                errors.append(f"{name}: torque_constant_nm_a {kt}")
            # The voltage constant is line to line, RMS; per phase, three times it is the torque constant again,
            # apart from saturation and temperature.
            ke = need(sheet, "voltage_constant_v_s_rad") / math.sqrt(3)
            # The data sheet of the 1FT7044-1AF prints 49 V per 1000 r/min beside a torque constant of 1.79 N m/A,
            # less than half of what fits. Kept as printed and not checked here.
            if name != "1FT7044-1AF7" and not close(3 * ke, kt, 0.15):
                errors.append(f"{name}: voltage_constant_v_s_rad against torque_constant_nm_a")
            # The electrical time constant is inductance over resistance.
            r, l = need(sheet, "phase_resistance_20c_ohm"), need(sheet, "phase_inductance_h")
            if not close(need(sheet, "electrical_time_constant_s"), l / r, 0.15):
                errors.append(f"{name}: electrical_time_constant_s against L / R = {l / r:.3g} s")
            if not need(sheet, "max_torque_nm") > 1.5 * need(motor, "static_torque_nm"):
                errors.append(f"{name}: max_torque_nm")
            if not need(sheet, "max_speed_inverter_rad_s") <= need(sheet, "max_speed_mechanical_rad_s"):
                errors.append(f"{name}: max_speed_inverter_rad_s above max_speed_mechanical_rad_s")
            # Water-cooled motors settle within a few minutes.
            if not inside(need(sheet, "thermal_time_constant_s"), 1 * MIN, 200 * MIN):
                errors.append(f"{name}: thermal_time_constant_s")
        except ValueError as e:
            errors.append(f"{name}: {e}")
    return errors


def _bahr_name(r):
    return f"{r['series']} {r['size']}"


def bahr_rows_are_unique_and_the_gantries_are_there(tables):
    """Bahr: no axis twice, each row's source is its series' sheet, the gantries are all there."""
    if not have(tables, "bahr/belt_axes.csv"):
        return []
    rows, errors, seen = tables["bahr/belt_axes.csv"], [], set()
    if len(rows) < 60:
        errors.append(f"only {len(rows)} rows")
    for r in rows:
        if (r["series"], r["size"]) in seen:
            errors.append(f"{_bahr_name(r)} twice")
        seen.add((r["series"], r["size"]))
        if not r["source"].endswith(f"_{r['series']}"):
            errors.append(f"{_bahr_name(r)}: source {r['source']}")
    count = lambda series, kind: sum(r["series"] == series and r["kind"] == kind for r in rows)
    for series, kind, least in (("ELZI", "xz_gantry", 3), ("ELZU", "xy_gantry", 7), ("ELZ", "axis", 8)):
        if count(series, kind) < least:
            errors.append(f"only {count(series, kind)} {series} {kind} rows")
    return errors


def bahr_belt_and_pulley_agree(tables):
    """Bahr: the belt code agrees with pitch and width, feed per turn = pulley teeth x pitch."""
    if not have(tables, "bahr/belt_axes.csv"):
        return []
    errors = []
    for r in tables["bahr/belt_axes.csv"]:
        name = _bahr_name(r)
        try:
            # "5M25": HTD with 5 mm pitch, 25 mm wide.
            pitch, m, width = r["belt"].partition("M")
            if not m:
                errors.append(f"{name}: belt {r['belt']}")
                continue
            if not close(need(r, "belt_pitch_m"), float(pitch) / 1000, 1e-12):
                errors.append(f"{name}: belt_pitch_m against belt {r['belt']}")
            if not close(need(r, "belt_width_m"), float(width) / 1000, 1e-12):
                errors.append(f"{name}: belt_width_m against belt {r['belt']}")
            # One turn of the pulley feeds one tooth pitch per tooth.
            if not close(need(r, "feed_per_rev_m"), need(r, "pulley_teeth") * need(r, "belt_pitch_m"), 1e-12):
                errors.append(f"{name}: feed_per_rev_m")
        except ValueError as e:
            errors.append(f"{name}: {e}")
    return errors


def bahr_forces_loads_and_weights_are_plausible(tables):
    """Bahr: feed forces against Fx, dynamic loads within static ones, no-load torque, speed and masses plausible."""
    if not have(tables, "bahr/belt_axes.csv"):
        return []
    errors = []
    for r in tables["bahr/belt_axes.csv"]:
        name = _bahr_name(r)
        n = lambda column: need(r, column)
        try:
            if not n("feed_force_peak_n") >= n("feed_force_continuous_n"):
                errors.append(f"{name}: feed_force_peak_n")
            # The belt pull is the load limit along the travel. The sheets round it differently in places
            # (900 N beside 894 N; ELZU 100: 3600 N beside 4000 N), hence the wide tolerance.
            if not close(n("feed_force_continuous_n"), n("fx_static_n"), 0.11):
                errors.append(f"{name}: feed_force_continuous_n against fx_static_n")
            for load in ("fx", "fy", "fz", "mx", "my", "mz"):
                unit = "n" if load.startswith("f") else "nm"
                fixed, moving = f"{load}_static_{unit}", f"{load}_dynamic_{unit}"
                # The X/Z gantry has no Fy.
                if r[fixed] == "" and r[moving] == "" and load == "fy" and r["kind"] == "xz_gantry":
                    continue
                if not 0 < n(moving) <= n(fixed):
                    errors.append(f"{name}: {moving} against {fixed}")
            if not 0 < n("no_load_torque_nm") < 5:
                errors.append(f"{name}: no_load_torque_nm")
            # The no-load torque is small beside what the belt pull needs at the pulley.
            pull_torque = n("feed_force_continuous_n") * n("feed_per_rev_m") / math.tau
            if not n("no_load_torque_nm") < 0.5 * pull_torque:
                errors.append(f"{name}: no_load_torque_nm beside {pull_torque:.1f} N m")
            if not inside(n("max_speed_m_s"), 1, 10):
                errors.append(f"{name}: max_speed_m_s")
            # As the sheet prints it: the base mass is above the mass per 100 mm of stroke.
            if not n("mass_at_zero_stroke_kg") > n("mass_per_stroke_kg_m") / 10 > 0:
                errors.append(f"{name}: mass_at_zero_stroke_kg or mass_per_stroke_kg_m")
        except ValueError as e:
            errors.append(f"{name}: {e}")
    return errors


def bahr_gantries_state_their_moved_masses_and_lengths(tables):
    """Bahr: a gantry's first axis moves more than the second, lengths and no-load torque count fit its kind."""
    if not have(tables, "bahr/belt_axes.csv"):
        return []
    errors = []
    for r in tables["bahr/belt_axes.csv"]:
        name = _bahr_name(r)
        n = lambda column: need(r, column)
        # The second axis of each gantry, the one that rides on the first.
        second = {"xz_gantry": "z", "xy_gantry": "y"}.get(r["kind"])
        if r["kind"] == "axis":
            errors += [f"{name}: {c} on a single axis" for c in ("moved_mass_x_kg", "moved_mass_y_kg", "moved_mass_z_kg", "mass_per_stroke_z_kg_m") if r[c] != ""]
            continue
        if second is None:
            errors.append(f"{name}: kind {r['kind']}")
            continue
        try:
            # The first axis moves more mass than the one riding on it, and the whole gantry weighs more than what moves.
            carried, riding = n("moved_mass_x_kg"), n(f"moved_mass_{second}_kg")
            if not 0 < riding < carried < n("mass_at_zero_stroke_kg"):
                errors.append(f"{name}: moved masses")
            if not n("max_length_x_m") >= n(f"max_length_{second}_m"):
                errors.append(f"{name}: max_length_x_m below max_length_{second}_m")
            # Only the X/Z gantry sheets print "2 x" before the no-load torque.
            if (r["no_load_torque_count"] == "2") != (r["kind"] == "xz_gantry"):
                errors.append(f"{name}: no_load_torque_count")
            if second == "z" and not n("mass_per_stroke_kg_m") > n("mass_per_stroke_z_kg_m"):
                errors.append(f"{name}: the double X guide is not heavier than Z")
        except ValueError as e:
            errors.append(f"{name}: {e}")
    return errors


def every_catalogue_gearbox_has_a_mass_and_bigger_ones_weigh_more(tables):
    """Gearbox masses: every gearbox of the three files has one, a bigger size of a series is heavier, NRK070-04-5 is 1.4 kg."""
    paths = [f"siemens/gearbox_{s}.csv" for s in ("nrb", "nrk", "nlc")]
    if not have(tables, "siemens/gearbox_mass.csv", *paths):
        return []
    masses = tables["siemens/gearbox_mass.csv"]
    errors, index = [], {}
    for m in masses:
        for frame in m["motor_frames"].split(" "):
            index.setdefault((m["size"], frame, m["ratio"]), m["mass_kg"])
    rows = []
    for path in paths:
        for r in tables[path]:
            key = (r["size"], r["motor_frame"], r["ratio"])
            name = "-".join(key)
            mass = num(index.get(key, ""))
            if mass is None:
                errors.append(f"{name}: no mass_kg in gearbox_mass.csv")
                continue
            if not 0.2 < mass < 40:
                errors.append(f"{name}: {mass} kg")
            rows.append((*key, mass))
    groups = {}
    for size, frame, ratio, mass in rows:
        groups.setdefault((size[:3], frame, ratio), []).append((size, mass))
    for (_, frame, ratio), of in groups.items():
        for a in of:
            for b in of:
                if a[0] < b[0] and not a[1] < b[1]:
                    errors.append(f"{a[0]}-{frame}-{ratio}: {a[1]} kg, {b[0]}: {b[1]} kg")
    # Table 7-33 of the 01/2023 manual.
    for key, want in ((("NRK070", "04", "5"), 1.4), (("NRB160", "08", "3"), 17.0)):
        if num(index.get(key, "")) != want:
            errors.append(f"{'-'.join(key)}: mass_kg {index.get(key)}, the manual has {want}")
    return errors


def megadyne_belts_are_plausible(tables):
    """Megadyne belts: EA per width within 25 % of the profile's mean, breaking strength 4 x the pull, mass grows with width, tooth load falls with speed."""
    if not have(tables, "megadyne/belts.csv", "megadyne/belt_tooth_load.csv"):
        return []
    belts, teeth_rows, errors = tables["megadyne/belts.csv"], tables["megadyne/belt_tooth_load.csv"], []
    for profile in ("T5", "AT3", "AT5", "AT10", "MTD3", "MTD5", "MTD8"):
        of = [r for r in belts if r["profile"] == profile]
        try:
            if len(of) < 4:
                errors.append(f"{profile}: only {len(of)} widths")
            if of:
                per_width = [need(r, "max_pull_n") / need(r, "elongation_at_max_pull") / need(r, "width_m") for r in of]
                mean = sum(per_width) / len(per_width)
                for r, e in zip(of, per_width):
                    if not close(e, mean, 0.25):
                        errors.append(f"{profile} {r['width_m']} m: EA per width {e:.4g} N/m against the mean {mean:.4g}")
                    # As printed: at 150 mm T5 and AT10 break at 3.5 and 3.43 times their allowable pull (12750 N against
                    # 3640 N, 105000 N against 30600 N).
                    times = 3.5 if close(need(r, "width_m"), 0.15, 1e-12) else 4.0
                    if not close(need(r, "breaking_strength_n"), times * need(r, "max_pull_n"), 0.1):
                        errors.append(f"{profile} {r['width_m']} m: breaking_strength_n")
                for a, b in zip(of, of[1:]):
                    if not (need(a, "width_m") < need(b, "width_m") and need(a, "mass_kg_m") < need(b, "mass_kg_m")):
                        errors.append(f"{profile} {b['width_m']} m: width or mass_kg_m not rising")
            teeth = [need(r, "specific_tooth_load_n_m") for r in teeth_rows if r["profile"] == profile]
            if len(teeth) != 18 or any(a < b for a, b in zip(teeth, teeth[1:])):
                errors.append(f"{profile}: tooth load not 18 values falling with speed")
        except ValueError as e:
            errors.append(f"{profile}: {e}")
    return errors


def _vendor_tables(tables, kind):
    """Other makers' tables of a kind ("motors_" or "drives_"): path -> rows."""
    return {p: rows for p, rows in tables.items() if not p.startswith("siemens/") and p.split("/")[-1].startswith(kind)}


# Values another maker's document prints that do not fit the others, kept as printed and not checked:
# (table, part_number, line_voltage_v, check) -> why. The same list is in docs/catalogue.md.
VENDOR_MISPRINTS = {
    **{("beckhoff/motors_am8000.csv", m, v, "ke"): "the AM802x print a torque constant 22 to 28 % below sqrt(3) x Ke; Kt fits M0 / I0"
       for m in ("AM8022-0D", "AM8022-0E", "AM8023-0E") for v in ("400", "480")},
    **{("beckhoff/motors_am8000.csv", m, v, "rated-power"): "rated power printed 14 to 25 % off rated torque x rated speed"
       for m, v in (("AM8051-0J-fan", "480"), ("AM8551-0J-fan", "480"), ("AM8052-0N-fan", "400"), ("AM8052-0N-fan", "480"),
                    ("AM8552-0N-fan", "400"), ("AM8552-0N-fan", "480"), ("AM8073-0U-fan", "480"))},
    ("kollmorgen/motors_akm.csv", "AKM64Q", "240", "ke"): "Ke printed 0.64 V/krpm; its Kt of 0.99 N m/A implies about 60",
}


def vendor_motors_are_plausible_and_their_drives_exist(tables):
    """Other makers' motors: overload, torque constant against standstill values and against the voltage constant,
    rated power = torque x speed, rated within standstill and maximum, brake adds inertia and mass, L / R and the thermal
    time constant within what servo motors have, and the paired drive exists at that voltage and carries the current."""
    drives = {}
    for path, rows in _vendor_tables(tables, "drives_").items():
        for d in rows:
            drives[(d["vendor"], d["line_voltage_v"], d["part_number"])] = d
    errors = []
    for path, rows in _vendor_tables(tables, "motors_").items():
        maker = path.split("/")[0]
        for m in rows:
            name, line = m["part_number"], m["line_voltage_v"]
            skip = lambda check: (path, name, line, check) in VENDOR_MISPRINTS  # noqa: E731
            fail = lambda check, text: None if skip(check) else errors.append(f"{path} {name} {line} V: {text}")  # noqa: E731
            try:
                if m["vendor"] != maker:
                    fail("vendor", f"vendor {m['vendor']} in the folder of {maker}")
                # A maker that prints no standstill values has its rated ones checked in their place, and the other way
                # round (as the simulator takes them, `catalog.rs` motor_data).
                rated_t, rated_i = num(m["rated_torque_nm"]), num(m["rated_current_a"])
                printed_rated_i, printed_m0 = rated_i, num(m["static_torque_nm"])
                m0 = num(m["static_torque_nm"]) or rated_t or need(m, "static_torque_nm")
                i0 = num(m["stall_current_a"]) or rated_i or need(m, "stall_current_a")
                rated_t, rated_i = rated_t or m0, rated_i or i0
                if not (need(m, "max_torque_nm") > 1.2 * m0 and need(m, "max_current_a") > 1.2 * i0):
                    fail("overload", "maximum torque or current not above 1.2 times the standstill values")
                if not rated_t <= 1.05 * m0:
                    fail("rated-torque", f"rated torque {rated_t} above standstill {m0}")
                if not need(m, "rated_speed_rad_s") <= 1.001 * need(m, "max_speed_rad_s"):
                    fail("rated-speed", "rated speed above maximum speed")
                kt = num(m["torque_constant_nm_a"])
                # The torque constant is standstill torque over standstill current, apart from rounding, saturation and
                # temperature: Rexroth prints Kt at 20 degC beside M0 and I0 at 100 K, 15 to 17 % apart for its
                # water-cooled MS2N at their high stall currents.
                if kt is not None and printed_m0 is not None and not close(kt, m0 / i0, 0.2):
                    fail("kt", f"torque_constant_nm_a {kt} against {m0} / {i0}")
                ke = num(m["voltage_constant_v_s_rad"])
                # Line-to-line RMS voltage constant: a sine-wave motor's torque constant is sqrt(3) times it.
                if kt is not None and ke is not None and not close(kt, math.sqrt(3) * ke, 0.2):
                    fail("ke", f"torque_constant_nm_a {kt} against sqrt(3) x voltage_constant {ke:.4g}")
                power = num(m["rated_power_w"])
                if power is not None and not close(power, rated_t * need(m, "rated_speed_rad_s"), 0.12):
                    fail("rated-power", f"rated_power_w {power} against torque x speed")
                for a, b in (("rotor_inertia_with_brake_kgm2", "rotor_inertia_kgm2"), ("mass_with_brake_kg", "mass_kg")):
                    if m[a] != "" and m[b] != "" and not need(m, a) >= need(m, b):
                        fail("brake", f"{a} below {b}")
                r, l = num(m["line_resistance_20c_ohm"]), num(m["line_inductance_h"])
                # A servo winding's electrical time constant: a fraction of a millisecond to some tens of them.
                if r and l and not inside(l / r, 1e-4, 0.1):
                    fail("l-over-r", f"L / R = {l / r:.3g} s")
                tau = num(m["thermal_time_constant_s"])
                if tau is not None and not inside(tau, 1 * MIN, 200 * MIN):
                    fail("thermal", f"thermal_time_constant_s {tau}")
                paired = m["paired_drive"]
                if paired:
                    d = drives.get((m["vendor"], line, paired))
                    if d is None:
                        fail("paired", f"paired drive {paired} not in a {m['vendor']} drive table at {line} V")
                    # Against a printed rated current only: SEW pairs by peak torque, smaller drives than the stall current.
                    elif printed_rated_i is not None and not need(d, "rated_output_current_a") >= 0.85 * rated_i:
                        fail("paired", f"{paired} gives {d['rated_output_current_a']} A, the motor needs {rated_i} A")
            except ValueError as e:
                errors.append(f"{path} {name} {line} V: {e}")
    return errors


def vendor_drives_are_plausible(tables):
    """Other makers' drives: peak above rated current, DC link above the line's mean and below the trip level,
    the chopper switching on above the line's peak and below the trip level."""
    errors = []
    for path, rows in _vendor_tables(tables, "drives_").items():
        maker = path.split("/")[0]
        for d in rows:
            name = d["part_number"]
            try:
                line = need(d, "line_voltage_v")
                if d["vendor"] != maker:
                    errors.append(f"{path} {name}: vendor {d['vendor']}")
                # Mitsubishi prints no peak current of its amplifiers.
                if d["max_output_current_a"] != "" and not need(d, "max_output_current_a") >= need(d, "rated_output_current_a"):
                    errors.append(f"{path} {name}: max_output_current_a below rated_output_current_a")
                if not need(d, "axes") >= 1:
                    errors.append(f"{path} {name}: axes")
                bus, trip, on = num(d["dc_link_voltage_v"]), num(d["dc_link_overvoltage_v"]), num(d["braking_threshold_v"])
                # Rectified, between the line's mean and its peak + 10 %; a regulated supply (Rexroth's at 750 V on a
                # 400 V line) lifts it higher, but never to the trip level.
                if bus is not None and not (inside(bus, 1.2 * line, 2 * line) and (trip is None or bus < trip)):
                    errors.append(f"{path} {name}: dc_link_voltage_v {bus} at a {line} V line")
                if on is not None and not on > SQRT_2 * line:
                    errors.append(f"{path} {name}: braking_threshold_v {on} not above the line's peak")
                if on is not None and trip is not None and not on < trip:
                    errors.append(f"{path} {name}: braking_threshold_v {on} not below dc_link_overvoltage_v {trip}")
            except ValueError as e:
                errors.append(f"{path} {name}: {e}")
    return errors


def siemens_s210_operating_instructions_agree_with_the_catalogue_and_the_manual(tables):
    """1FK2 and 1FT2 thermal time constants of the S210 operating instructions: every motor is in the S210 catalogue with
    the same static torque and maximum current, and where an S120 manual has the motor too it gives the same time
    constant."""
    s120 = ("siemens/motors_1fk2_s120.csv", "siemens/motors_1ft2_s120.csv")
    if not have(tables, "siemens/motors_s210_oi.csv", "siemens/motors_s210.csv", *s120):
        return []
    oi, errors = tables["siemens/motors_s210_oi.csv"], []
    catalogue = by_part(tables["siemens/motors_s210.csv"])
    manual = {**by_part(tables[s120[0]]), **by_part(tables[s120[1]])}
    if len(oi) < 70:
        errors.append(f"only {len(oi)} rows")
    for m in oi:
        name = m["part_number"]
        try:
            if not inside(need(m, "thermal_time_constant_s"), 5 * MIN, 120 * MIN):
                errors.append(f"{name}: thermal_time_constant_s")
            c = catalogue.get(name)
            if c is None:
                errors.append(f"{name}: not in motors_s210.csv")
                continue
            for column in ("static_torque_nm", "max_current_a"):
                if not close(need(m, column), need(c, column), 0.01):
                    errors.append(f"{name}: {column} {m[column]} against {c[column]} in motors_s210.csv")
            if name in manual and m["thermal_time_constant_s"] != manual[name]["thermal_time_constant_s"]:
                errors.append(f"{name}: thermal_time_constant_s against the S120 manual")
        except ValueError as e:
            errors.append(f"{name}: {e}")
    return errors


def siemens_s120_dc_link_trip_is_that_of_active_line_modules(tables):
    """S120 DC link trip: every row is an Active Line Module of line_modules_s120.csv, above the DC link's highest
    operating voltage (720 V) and the Braking Module's switch-on threshold."""
    if not have(tables, "siemens/dc_link_trip_s120.csv", "siemens/line_modules_s120.csv", "siemens/braking_s120.csv"):
        return []
    modules, errors = by_part(tables["siemens/line_modules_s120.csv"]), []
    chopper = max((num(r["activation_threshold_v"]) or 0.0) for r in tables["siemens/braking_s120.csv"])
    for r in tables["siemens/dc_link_trip_s120.csv"]:
        name = r["part_number"]
        try:
            if modules.get(name, {}).get("infeed_type") != "active":
                errors.append(f"{name}: not an Active Line Module of line_modules_s120.csv")
            if not need(r, "dc_link_overvoltage_v") > max(720.0, chopper):
                errors.append(f"{name}: dc_link_overvoltage_v")
        except ValueError as e:
            errors.append(f"{name}: {e}")
    return errors


CHECKS = [
    festo_exct_values_agree_with_its_own_geometry,
    siemens_s210_motors_are_plausible_and_their_converters_exist,
    siemens_s210_braking_rows_are_the_s210_converters,
    siemens_s120_motor_modules_are_plausible,
    siemens_s120_motors_are_plausible_and_their_motor_modules_exist,
    siemens_gearbox_rows_are_complete_and_plausible,
    siemens_1fk2_data_agrees_between_the_s120_manual_and_the_s210_catalogue,
    siemens_1ft2_data_agrees_between_the_s120_manual_and_the_s210_catalogue,
    siemens_1fk2_dynamic_braking_fits_the_motors,
    siemens_s210_operating_instructions_agree_with_the_catalogue_and_the_manual,
    siemens_s120_line_modules_are_plausible,
    siemens_s120_dc_link_trip_is_that_of_active_line_modules,
    siemens_s120_braking_resistors_fit_their_modules,
    siemens_s120_data_sheets_agree_with_the_catalogue,
    bahr_rows_are_unique_and_the_gantries_are_there,
    bahr_belt_and_pulley_agree,
    bahr_forces_loads_and_weights_are_plausible,
    bahr_gantries_state_their_moved_masses_and_lengths,
    every_catalogue_gearbox_has_a_mass_and_bigger_ones_weigh_more,
    megadyne_belts_are_plausible,
    vendor_motors_are_plausible_and_their_drives_exist,
    vendor_drives_are_plausible,
]

# The mechanics tables of 14 November 2026 have their checks in checks_mech.py.
from .checks_mech import MECH_CHECKS  # noqa: E402

CHECKS += MECH_CHECKS
