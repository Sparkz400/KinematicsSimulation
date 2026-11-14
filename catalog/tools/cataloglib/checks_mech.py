"""Physics and cross-document checks of the mechanics tables added on 14 November 2026: other makers' gearboxes, ball
screws, guides, linear motors, belts, single belt axes and motor shafts. Each takes the tables (path -> rows) and returns
messages, as the checks of checks.py; checks.py runs them after its own (`MECH_CHECKS`)."""

import math
import re

from .checks import ARCMIN, MIN, close, have, inside, need, num

PI = math.pi


# --- apex

APEX_ORDER = {"apex/gearbox_peii.csv": ["PEII050", "PEII070", "PEII090", "PEII120", "PEII155"],
              "apex/gearbox_af.csv": ["AF042", "AF060", "AF075", "AF100", "AF140", "AF180", "AF220"]}
APEX_RATIOS = {"apex/gearbox_peii.csv": (5, 11), "apex/gearbox_af.csv": (8, 17)}


def apex_gearboxes_agree_with_their_own_physics(tables):
    """Apex PEII and AF: T2B = 1.8 T2N and T2NOT = 3 T2N as printed, rated < max speed, reduced < standard backlash,
    two stages no tighter and no more efficient than one, per-size values one per size and stage, a bigger size is
    stronger and heavier, the ratio count per size and bore as printed, values in a sensible band."""
    errors = []
    for path, order in APEX_ORDER.items():
        if not have(tables, path):
            continue
        rows = tables[path]
        per = {}
        for r in rows:
            name = f"{path}:{r['size']}-{r['motor_frame']}-{r['ratio']}"
            try:
                t2n, t2b, t2not = need(r, "rated_output_torque_nm"), need(r, "max_output_torque_nm"), need(r, "emergency_stop_output_torque_nm")
                if not (t2n < t2b < t2not and close(t2b, 1.8 * t2n, 1e-9) and close(t2not, 3 * t2n, 1e-9)):
                    errors.append(f"{name}: torques {t2n} {t2b} {t2not}")
                if not need(r, "rated_input_speed_rad_s") < need(r, "max_input_speed_rad_s"):
                    errors.append(f"{name}: rated speed not below max speed")
                if not close(need(r, "clamping_hub_diameter_m"), need(r, "motor_frame") / 1000, 1e-9):
                    errors.append(f"{name}: hub diameter is not the motor_frame bore")
                if num(r.get("backlash_reduced_rad", "")) is not None and not need(r, "backlash_reduced_rad") < need(r, "backlash_rad"):
                    errors.append(f"{name}: reduced backlash not below standard")
                if not inside(need(r, "backlash_rad"), 2.9e-4, 4.1e-3):          # 1 to 14 arcmin
                    errors.append(f"{name}: backlash out of band")
                if not inside(need(r, "torsional_stiffness_nm_rad"), 1e3, 1e7):
                    errors.append(f"{name}: stiffness out of band")
                if not inside(need(r, "inertia_at_input_kgm2"), 1e-7, 1e-2):
                    errors.append(f"{name}: inertia out of band")
                if not inside(need(r, "efficiency_full_load"), 0.9, 0.99):
                    errors.append(f"{name}: efficiency out of band")
                # AF180 is printed with F2aB 19,500 N above F2rB 18,000 N, in the AF and the AFR table alike.
                if r["size"] != "AF180" and not need(r, "max_axial_force_n") <= need(r, "max_radial_force_n"):
                    errors.append(f"{name}: axial force above radial force")
                key = (r["size"], int(need(r, "stages")))
                if int(need(r, "stages")) != (1 if need(r, "ratio") <= 10 else 2):
                    errors.append(f"{name}: stages do not match ratio")
                same = tuple(r.get(c, "") for c in ("torsional_stiffness_nm_rad", "max_radial_force_n", "max_axial_force_n",
                                                    "max_tilting_moment_nm", "rated_input_speed_rad_s", "max_input_speed_rad_s",
                                                    "backlash_rad", "backlash_reduced_rad", "efficiency_full_load", "mass_kg",
                                                    "no_load_torque_nm"))
                if per.setdefault(key, same) != same:
                    errors.append(f"{name}: per-size values differ within {key}")
            except ValueError as e:
                errors.append(f"{name}: {e}")
        # one stage tighter and more efficient than two
        for (size, s), v in per.items():
            two = per.get((size, 2))
            if s == 1 and two:
                if not float(v[6]) < float(two[6]) or not float(v[8]) > float(two[8]):
                    errors.append(f"{path}:{size}: 1-stage backlash/efficiency not better than 2-stage")
        # bigger size: stiffer, more radial force, higher peak torque at ratio 10 / 100, heavier
        for s in (1, 2):
            last = None
            for size in order:
                if (size, s) not in per:
                    continue
                v = per[(size, s)]
                ratio = "10" if s == 1 else "100"
                t = max(float(r["rated_output_torque_nm"]) for r in rows if r["size"] == size and r["ratio"] == ratio)
                now = (float(v[0]), float(v[1]), t, float(v[9]) if v[9] else None)
                if last and not (now[0] >= last[0] and now[1] > last[1] and now[2] > last[2] and (now[3] is None or now[3] > last[3])):
                    errors.append(f"{path}:{size} stage {s}: not stronger/heavier than the size before")
                last = now
        # ratio count per size and bore as printed; inertia does not fall with a bigger bore (PEII)
        counts, inertia = {}, {}
        for r in rows:
            k = (r["size"], r["motor_frame"], r["stages"])
            counts[k] = counts.get(k, 0) + 1
            inertia.setdefault((r["size"], r["stages"], r["ratio"]), []).append((float(r["motor_frame"]), float(r["inertia_at_input_kgm2"])))
        for (size, frame, s), n in counts.items():
            if n != APEX_RATIOS[path][int(s) - 1]:
                errors.append(f"{path}:{size}-{frame} stage {s}: {n} ratios")
        for k, pts in inertia.items():
            js = [j for _, j in sorted(pts)]
            if js != sorted(js):
                errors.append(f"{path}:{k}: inertia falls with a bigger bore")
    return errors


if __name__ == "__main__":
    import csv
    from pathlib import Path
    root = Path(r"D:\Entwicklung\KinematicsSimulation\.claude\worktrees\fix-netlist-nodes-badf62\catalog")
    tables = {p: list(csv.DictReader(open(root / p, encoding="utf-8"))) for p in APEX_ORDER}
    print("as written:", apex_gearboxes_agree_with_their_own_physics(tables))
    tables["apex/gearbox_af.csv"][3]["max_output_torque_nm"] = "99"       # broken on purpose
    tables["apex/gearbox_peii.csv"][0]["mass_kg"] = "1"
    print("broken:", apex_gearboxes_agree_with_their_own_physics(tables))


# --- festo

SINGLE_AXIS_TABLES = ["festo/belt_axes_egc.csv", "festo/belt_axes_elga.csv", "rexroth/belt_axes_ckr.csv"]


def single_belt_axes_agree_with_their_geometry(tables):
    """Festo EGC/ELGA and Rexroth CKR single belt axes: feed per turn = pi x pulley diameter = teeth x pitch; the
    drive torque limit and the feed force limit fit one pulley; a bigger size is heavier and stronger; values in a
    sensible band; the sizes as printed."""
    errors = []
    expected = {"festo/belt_axes_egc.csv": {"EGC-TB-KF": ["50", "70", "80", "120", "185"]},
                "festo/belt_axes_elga.csv": {"ELGA-TB-KF": ["70", "80", "120", "150"], "ELGA-TB-G": ["70", "80", "120"]},
                "rexroth/belt_axes_ckr.csv": {"CKR": ["070", "070L", "090", "090L", "110", "110L", "145", "145L",
                                                      "200", "200L", "280"]}}
    for path in SINGLE_AXIS_TABLES:
        if not have(tables, path):
            continue
        rows = tables[path]
        for series, sizes in expected[path].items():
            got = [r["size"] for r in rows if r["series"] == series]
            if got != sizes:
                errors.append(f"{path}: {series} sizes {got}, expected {sizes}")
        previous = {}
        for r in rows:
            name = f"{r['series']}-{r['size']}"
            try:
                d, pitch, feed = need(r, "pulley_effective_diameter_m"), need(r, "belt_pitch_m"), need(r, "feed_per_rev_m")
                teeth = need(r, "pulley_teeth")
                if not close(feed, PI * d, 2e-3):
                    errors.append(f"{name}: feed_per_rev_m {feed} != pi x diameter {PI * d}")
                if not close(feed, teeth * pitch, 1e-9):
                    errors.append(f"{name}: feed_per_rev_m != teeth x pitch")
                if not inside(teeth, 15, 50):
                    errors.append(f"{name}: pulley_teeth {teeth}")
                # The pulley torque limit and the feed force limit belong to the same pulley: F d/2 within 1/3 .. 3 x M.
                force, torque = need(r, "feed_force_peak_n"), need(r, "max_pulley_torque_nm")
                if not inside(force * d / 2 / torque, 0.3, 3.0):
                    errors.append(f"{name}: feed force x radius {force * d / 2} vs pulley torque {torque}")
                if not inside(need(r, "max_speed_m_s"), 1, 10) or not inside(need(r, "max_acceleration_m_s2"), 10, 100):
                    errors.append(f"{name}: speed or acceleration out of band")
                moved, base = need(r, "moved_mass_x_kg"), need(r, "mass_at_zero_stroke_kg")
                if not moved < base:
                    errors.append(f"{name}: moved mass {moved} >= mass at zero stroke {base}")
                if not inside(need(r, "mass_per_stroke_kg_m"), 0.5, 100):
                    errors.append(f"{name}: mass_per_stroke_kg_m")
                if not need(r, "max_no_load_torque_nm" if "max_no_load_torque_nm" in r else "no_load_torque_nm") < 0.2 * torque:
                    errors.append(f"{name}: no-load torque not well below the drive torque")
                if not need(r, "fz_dynamic_n") >= need(r, "fy_dynamic_n") * 0.99 and r["series"] != "CKR":
                    errors.append(f"{name}: Fz < Fy")
                elongation = num(r.get("belt_elongation", ""))
                if elongation is not None and not inside(elongation, 1e-4, 1e-2):
                    errors.append(f"{name}: belt_elongation {elongation}")
                # Within a series (and carriage length), a bigger size is stronger and heavier.
                key = (r["series"], r["size"].endswith("L"))
                if key in previous:
                    p = previous[key]
                    for c in ("feed_force_peak_n", "max_pulley_torque_nm", "mass_at_zero_stroke_kg", "feed_per_rev_m",
                              "mx_dynamic_nm"):
                        if not need(r, c) >= need(p, c):
                            errors.append(f"{name}: {c} smaller than the size before")
                previous[key] = r
            except ValueError as e:
                errors.append(f"{name}: {e}")
    return errors


# --- gates
def gates_belts_are_plausible(tables):
    """Gates Synchro-Power LINEAR: breaking force about 4 x the open-ended allowable pull, strain at that pull in a
    band, mass and stiffness rise with width, Z min x pitch / pi near the printed D min, one tooth load per profile
    that agrees with every width's allowable effective force."""
    if not have(tables, "gates/belts.csv", "gates/belt_tooth_load.csv"):
        return []
    belts, teeth, errors = tables["gates/belts.csv"], tables["gates/belt_tooth_load.csv"], []
    # As printed: HTD8 25 mm (17100 N, the AT10 value) and HTD8 100 mm (58400 N) break at 4.57 and 3.47 x their pull.
    misprints = {("HTD8", 0.025), ("HTD8", 0.1)}
    for profile in ("T5", "T10", "AT5", "AT10", "HTD5", "HTD8"):
        of = [r for r in belts if r["profile"] == profile]
        try:
            if len(of) < 6:
                errors.append(f"{profile}: only {len(of)} widths")
            ratios = sorted(need(r, "breaking_strength_n") / need(r, "max_pull_n") for r in of)
            median = ratios[len(ratios) // 2]
            if not inside(median, 3.5, 4.6):
                errors.append(f"{profile}: breaking / pull {median:.3g}")
            for r in of:
                w = need(r, "width_m")
                if (profile, w) not in misprints and not close(need(r, "breaking_strength_n") / need(r, "max_pull_n"), median, 0.02):
                    errors.append(f"{profile} {w} m: breaking_strength_n against max_pull_n")
                strain = need(r, "max_pull_n") / (need(r, "stiffness_ea_per_width_n_m") * w)
                if not inside(strain, 0.002, 0.006):
                    errors.append(f"{profile} {w} m: strain {strain:.3g} at max_pull_n")
                # D min of the steel cord is the pitch circle of Z min teeth, rounded up (T5: 10 x 5 mm / pi = 15.9 -> 16 mm).
                pitch_d = need(r, "min_pulley_teeth") * need(r, "pitch_m") / math.pi
                if not inside(need(r, "min_pulley_diameter_m"), pitch_d * 0.95, pitch_d * 1.15):
                    errors.append(f"{profile} {w} m: min_pulley_diameter_m against min_pulley_teeth")
                if not inside(need(r, "thickness_m"), 0.3 * need(r, "pitch_m"), 0.8 * need(r, "pitch_m")):
                    errors.append(f"{profile} {w} m: thickness_m")
            for a, b in zip(of, of[1:]):
                if not (need(a, "width_m") < need(b, "width_m") and need(a, "mass_kg_m") <= need(b, "mass_kg_m")
                        and need(a, "stiffness_ea_per_width_n_m") * need(a, "width_m") < need(b, "stiffness_ea_per_width_n_m") * need(b, "width_m")):
                    errors.append(f"{profile} {b['width_m']} m: width, mass_kg_m or stiffness not rising")
            tooth = [need(t, "specific_tooth_load_n_m") for t in teeth if t["profile"] == profile]
            if len(tooth) != 1 or not inside(tooth[0], 1500, 10000):
                errors.append(f"{profile}: tooth load {tooth}")
        except ValueError as e:
            errors.append(f"{profile}: {e}")
    return errors


def megadyne_min_pulley_teeth_is_plausible(tables):
    """Megadyne: one Z min and thickness per profile, Z min 10 to 20 teeth, thickness between 0.3 and 0.8 pitch."""
    if not have(tables, "megadyne/belts.csv"):
        return []
    errors = []
    for r in tables["megadyne/belts.csv"]:
        name = f"{r['profile']} {r['width_m']} m"
        try:
            same = {(o["min_pulley_teeth"], o["thickness_m"]) for o in tables["megadyne/belts.csv"] if o["profile"] == r["profile"]}
            if len(same) != 1:
                errors.append(f"{name}: min_pulley_teeth or thickness_m differ within the profile")
            if not inside(need(r, "min_pulley_teeth"), 10, 20):
                errors.append(f"{name}: min_pulley_teeth")
            if not inside(need(r, "thickness_m"), 0.3 * need(r, "pitch_m"), 0.8 * need(r, "pitch_m")):
                errors.append(f"{name}: thickness_m")
        except ValueError as e:
            errors.append(f"{name}: {e}")
    return errors


# --- harmonicdrive
def harmonicdrive_strain_wave_gears_are_plausible(tables):
    """Harmonic Drive CSF/CSG/SHG: torque limits in order, K1 <= K2 <= K3 and T1 < T2, T1 < repeated peak, a bigger size is
    rated higher and weighs more, per-size values the same on every ratio, the ratio count per size as printed, the
    CSG and SHG (same gearing) agree except the two values the SHG table prints differently."""
    counts = {
        "csf": {"8": 3, "11": 3, "14": 4, "17": 5, "20": 6, "25": 6, "32": 6, "40": 5, "45": 5, "50": 5, "58": 5,
                "65": 5, "80": 5, "90": 5, "100": 5},
        "csg": {"14": 3, "17": 4, "20": 5, "25": 5, "32": 5, "40": 5, "45": 5, "50": 4, "58": 4, "65": 4},
    }
    counts["shg"] = counts["csg"]
    errors = []
    for series, expected in counts.items():
        path = f"harmonicdrive/gearbox_{series}.csv"
        if not have(tables, path):
            continue
        rows = tables[path]
        prefix = series.upper()
        seen = {}
        for r in rows:
            seen.setdefault(r["size"][len(prefix):], []).append(r)
        if {s: len(v) for s, v in seen.items()} != expected:
            errors.append(f"{path}: ratios per size {[(s, len(v)) for s, v in seen.items()]}")
        last = None
        for size in sorted(seen, key=float):
            group = seen[size]
            name = f"{prefix}{size}"
            try:
                for column in ("motor_frame", "clamping_hub_diameter_m", "rated_input_speed_rad_s",
                               "max_input_speed_rad_s", "inertia_at_input_kgm2", "mass_kg", "stiffness_t1_nm",
                               "stiffness_t2_nm"):
                    if len({r[column] for r in group}) != 1:
                        errors.append(f"{name}: {column} differs between ratios")
                g = group[-1]
                hub = num(g["clamping_hub_diameter_m"])
                if hub is not None and not hub >= float(g["motor_frame"]) / 1000:
                    errors.append(f"{name}: max bore below the standard bore")
                if not need(g, "rated_input_speed_rad_s") < need(g, "max_input_speed_rad_s"):
                    errors.append(f"{name}: average input speed not below the maximum")
                if not inside(need(g, "inertia_at_input_kgm2"), 1e-7, 1e-1):
                    errors.append(f"{name}: inertia_at_input_kgm2 out of band")
                for r in group:
                    where = f"{name} ratio {r['ratio']}"
                    rated, peak, stop = (need(r, c) for c in ("rated_output_torque_nm", "max_output_torque_nm",
                                                             "emergency_stop_output_torque_nm"))
                    if not rated < peak < stop:
                        errors.append(f"{where}: not rated < repeated peak < momentary peak")
                    k1, k2, k3 = (need(r, f"stiffness_k{i}_nm_rad") for i in (1, 2, 3))
                    if not (k1 <= k2 <= k3 and need(r, "torsional_stiffness_nm_rad") == k3):
                        errors.append(f"{where}: K1 <= K2 <= K3 = torsional_stiffness")
                    # T2 may lie above the repeated peak (CSF32 ratio 30: 108 against 100 N m, as printed).
                    if not need(r, "stiffness_t1_nm") < min(need(r, "stiffness_t2_nm"), peak):
                        errors.append(f"{where}: not T1 < T2 and T1 < repeated peak")
                    # The twist at T2 (theta2 of the document) lies between 0.5 and 5 mrad (printed: 0.6 to 4.0) for every size and ratio.
                    twist = need(r, "stiffness_t1_nm") / k1 + (need(r, "stiffness_t2_nm") - need(r, "stiffness_t1_nm")) / k2
                    if not inside(twist, 5e-4, 5e-3):
                        errors.append(f"{where}: twist at T2 {twist:.2e} rad")
                    if not inside(need(r, "hysteresis_rad"), 2.9e-4, 8.7e-4):
                        errors.append(f"{where}: hysteresis_rad")
                now = (need(g, "rated_output_torque_nm"), need(g, "mass_kg"), need(g, "inertia_at_input_kgm2"),
                       need(g, "stiffness_t2_nm"))
                if last and not all(n > l for n, l in zip(now, last)):
                    errors.append(f"{name}: rated torque (top ratio), mass, inertia, T2 not above the smaller size")
                last = now
            except ValueError as e:
                errors.append(f"{name}: {e}")
    if have(tables, "harmonicdrive/gearbox_csg.csv", "harmonicdrive/gearbox_shg.csv"):
        # The SHG is the CSG's gearing with a hollow shaft: its rating table prints the CSG's numbers, except
        # SHG32's grease average input speed (3000 against 3500 r/min) and SHG65's inertia (46.5 against 46.8).
        exceptions = {("32", "rated_input_speed_rad_s"), ("65", "inertia_at_input_kgm2")}
        shg = {(r["size"][3:], r["ratio"]): r for r in tables["harmonicdrive/gearbox_shg.csv"]}
        for c in tables["harmonicdrive/gearbox_csg.csv"]:
            s = shg.get((c["size"][3:], c["ratio"]))
            if s is None:
                errors.append(f"{c['size']} ratio {c['ratio']}: no SHG")
                continue
            for column in ("rated_output_torque_nm", "max_output_torque_nm", "emergency_stop_output_torque_nm",
                           "rated_input_speed_rad_s", "max_input_speed_rad_s", "inertia_at_input_kgm2",
                           "stiffness_k1_nm_rad", "stiffness_k2_nm_rad", "stiffness_k3_nm_rad", "hysteresis_rad"):
                if (c["size"][3:], column) not in exceptions and num(c[column]) != num(s[column]):
                    errors.append(f"{c['size']} ratio {c['ratio']}: {column} differs from the SHG")
    return errors


# --- hiwin
def hiwin_screws_and_guides_agree_with_their_own_geometry(tables):
    """HIWIN FDW, FDC, FSH screws and HG/EG guides: what the dimension tables tie together."""
    errors = []
    if have(tables, "hiwin/ball_screws.csv"):
        rows = tables["hiwin/ball_screws.csv"]
        dn = {"FDW": 70000, "FSH": 70000, "FDC": 220000}  # M33 general rule; Super S "up to 220,000" (page 6)
        code = {}
        for r in rows:
            p = r["part_number"]
            m = re.fullmatch(r"(\d+)-([\d.]+)([A-Z])(\d)-([A-Z]+)(-.*)?", p)
            if not m or m[5] != r["series"]:
                errors.append(f"hiwin {p}: not <model>-<series>[-suffix]")
                continue
            code[p] = m
            d, lead, rd, ball = (need(r, c) for c in ("nominal_diameter_m", "lead_m", "root_diameter_m", "ball_diameter_m"))
            c, c0, k = need(r, "dynamic_load_rating_n"), need(r, "static_load_rating_n"), need(r, "axial_stiffness_n_m")
            if not close(d, float(m[1]) / 1000, 1e-9) or not close(lead, float(m[2]) / 1000, 1e-9):
                errors.append(f"hiwin {p}: diameter or lead does not match the model code")
            # root diameter about nominal minus one ball (M37: dr = Dm - Db, Dm a little above d)
            if not inside(d - rd, 0.6 * ball, 1.3 * ball):
                errors.append(f"hiwin {p}: d - root diameter {d - rd:.4g} m not near the ball {ball} m")
            if not inside(c0 / c, 1.5, 5.0):  # 4.4 for 50-5K5-FDC: small balls on a large screw
                errors.append(f"hiwin {p}: C0/C = {c0 / c:.2f} outside 1.5..5")
            if not inside(k, 1.5e8, 4e9):
                errors.append(f"hiwin {p}: stiffness {k:.3g} N/m outside 0.15..4 kN/um")
            if num(r["max_dn"]) != dn.get(r["series"]):
                errors.append(f"hiwin {p}: max_dn {r['max_dn']} is not the catalogue's {dn.get(r['series'])}")
        # same nut type and thread, more ball circuits -> higher C, C0 and K (e.g. 32-8B1 < 32-8B2 < 32-8B3);
        # a longer nut too, except FSH, whose S4 nuts are as long as the S2 (four starts in one nut)
        thread = {}
        for r in rows:
            if r["part_number"] in code:
                m = code[r["part_number"]]
                thread.setdefault((m[5], m[1], m[2], m[3], r["ball_diameter_m"]), []).append(r)
        for (series, *_), group in thread.items():
            group.sort(key=lambda r: int(code[r["part_number"]][4]))
            for a, b in zip(group, group[1:]):
                if code[a["part_number"]][4] == code[b["part_number"]][4]:
                    continue  # the same model printed twice (another nut flange)
                cols = ["dynamic_load_rating_n", "static_load_rating_n", "axial_stiffness_n_m"]
                for col in cols + (["nut_length_m"] if series != "FSH" else []):
                    if not need(a, col) < need(b, col):
                        errors.append(f"hiwin {b['part_number']}: {col} not above {a['part_number']}'s")
    if have(tables, "hiwin/ball_screw_mountings.csv"):
        m = {r["mounting"]: r for r in tables["hiwin/ball_screw_mountings.csv"]}
        # the critical speed factors go with the clamped beam's (lambda L)^2: 22.37, 15.42, 9.87, 3.52
        lam = {"fixed-fixed": 4.730, "fixed-supported": 3.927, "supported-supported": 3.1416, "fixed-free": 1.875}
        for name, ll in lam.items():
            ratio = need(m[name], "critical_speed_factor") / need(m["fixed-fixed"], "critical_speed_factor")
            if not close(ratio, (ll / 4.730) ** 2, 0.02):
                errors.append(f"hiwin mounting {name}: critical speed factor ratio {ratio:.3f} not beam theory's")
            # Euler: buckling factors 1 : 1/2 : 1/4 : 1/16 of fixed-fixed
        for name, share in {"fixed-supported": 0.5, "supported-supported": 0.25, "fixed-free": 0.0625}.items():
            if not close(need(m[name], "buckling_factor"), share * need(m["fixed-fixed"], "buckling_factor"), 1e-6):
                errors.append(f"hiwin mounting {name}: buckling factor not {share} of fixed-fixed")
    if have(tables, "hiwin/linear_guides.csv"):
        rows = tables["hiwin/linear_guides.csv"]
        for r in rows:
            p = r["part_number"]
            c, c0 = need(r, "dynamic_load_rating_n"), need(r, "static_load_rating_n")
            if not inside(c0 / c, 1.2, 2.2):
                errors.append(f"hiwin {p}: C0/C = {c0 / c:.2f} outside 1.2..2.2")
            if need(r, "dynamic_load_rating_distance_m") != 50000:
                errors.append(f"hiwin {p}: C of a ball guide is for 50 km")
            if not need(r, "static_moment_y_nm") == need(r, "static_moment_z_nm"):
                errors.append(f"hiwin {p}: MP != MY (four-row 45 degree design prints them equal)")
            if not inside(need(r, "rail_width_m"), 0.8 * float(r["size"]) / 1000, 1.05 * float(r["size"]) / 1000):
                errors.append(f"hiwin {p}: rail width {r['rail_width_m']} m does not fit size {r['size']}")
        # bigger size, same block: higher C, C0, MR and heavier block and rail
        series = {}
        for r in rows:
            series.setdefault(r["carriage_type"], []).append(r)
        for group in series.values():
            group.sort(key=lambda r: int(r["size"]))
            for a, b in zip(group, group[1:]):
                for col in ("dynamic_load_rating_n", "static_load_rating_n", "static_moment_x_nm",
                            "carriage_mass_kg", "rail_mass_kg_m", "carriage_length_m"):
                    if not need(a, col) < need(b, col):
                        errors.append(f"hiwin {b['part_number']}: {col} not above {a['part_number']}'s")
        # H and W blocks of one size share rail, C, C0 and moments
        by = {}
        for r in rows:
            by.setdefault((r["series"], r["size"]), []).append(r)
        for group in by.values():
            for col in ("rail_width_m", "dynamic_load_rating_n", "static_load_rating_n", "static_moment_x_nm",
                        "rail_mass_kg_m", "seal_drag_n"):
                if len({g[col] for g in group}) != 1:
                    errors.append(f"hiwin {group[0]['series']}{group[0]['size']}: {col} differs between blocks")
    return errors


# --- linear-motors-siemens-beckhoff

SIEMENS_1FN3 = "siemens/linear_motors_1fn3.csv"
BECKHOFF_AL8000 = "beckhoff/linear_motors_al8000.csv"
LENGTH_ORDER = "123456789ABCDEF"


def _linear_motor_rows_are_plausible(rows, path, ke_factor, ke_tol, km_from_r, loss_band):
    """Peak above continuous, kF = factor x kE, kM and the power loss from R and I, the speeds, the attraction
    force and the force per coil mass in a band, and within a family (size, cooling, winding) a longer coil has
    more force, more mass and more length."""
    errors, families = [], {}
    for r in rows:
        p = f"{path} {r['part_number']}"
        try:
            fmax, fn, imax, i_n = (need(r, c) for c in ("peak_force_n", "continuous_force_n", "peak_current_a",
                                                         "continuous_current_a"))
            kf, ke, res = need(r, "force_constant_n_a"), need(r, "back_emf_constant_v_s_m"), need(r, "phase_resistance_20c_ohm")
            km, loss, fa, mass = (need(r, c) for c in ("motor_constant_n_sqrt_w", "continuous_power_loss_w",
                                                       "attraction_force_n", "coil_mass_kg"))
            v, length, pitch = need(r, "max_speed_m_s"), need(r, "coil_length_m"), need(r, "pole_pitch_m")
            need(r, "phase_inductance_h"), need(r, "magnet_track_mass_kg_m"), need(r, "line_voltage_v")
        except ValueError as e:
            errors.append(f"{p}: {e}")
            continue
        if not (fmax > fn and imax > i_n):
            errors.append(f"{p}: peak {fmax} N / {imax} A not above continuous {fn} N / {i_n} A")
        if not inside(fn / i_n / kf, 0.85, 1.15):
            errors.append(f"{p}: F/I {fn / i_n:.1f} vs kF {kf}")
        if not close(kf, ke_factor * ke, ke_tol):
            errors.append(f"{p}: kF {kf} vs {ke_factor:.3f} kE {ke_factor * ke:.1f}")
        expected_km = kf / math.sqrt(3 * res) if km_from_r else fn / math.sqrt(loss)
        if not close(km, expected_km, 0.05):
            errors.append(f"{p}: kM {km} vs {expected_km:.2f}")
        if not inside(loss / (3 * res * i_n ** 2), *loss_band):
            errors.append(f"{p}: power loss {loss} W vs 3 R I^2 {3 * res * i_n ** 2:.0f} W")
        if r["max_speed_at_peak_force_m_s"] and need(r, "max_speed_at_peak_force_m_s") > v:
            errors.append(f"{p}: speed at peak force above the speed at rated force")
        if not (inside(v, 0.3, 20) and inside(fa / fn, 1.5, 8) and inside(fn / mass, 20, 200)
                and inside(pitch, 0.02, 0.05)):
            errors.append(f"{p}: speed {v}, attraction/force {fa / fn:.2f}, N/kg {fn / mass:.0f} or pole pitch {pitch} out of band")
        name = r["part_number"]
        # 1FN3SSS-L?WW..: size, length digit, letter; AL80uv-wx: width u, length v, cooling w, winding x
        if name.startswith("1FN3"):
            fam, n = (name[:7], name[9], name[10:13]), name[8]
        else:
            fam, n = (name[4], name[7], name[8]), name[5]
        families.setdefault(fam, []).append((LENGTH_ORDER.index(n), fn, mass, length, name))
    for fam, members in families.items():
        members.sort()
        for a, b in zip(members, members[1:]):
            if a[0] < b[0] and not (a[1] < b[1] and a[2] < b[2] and a[3] < b[3]):
                errors.append(f"{path}: {b[4]} is longer than {a[4]} but not stronger, heavier and longer")
    return errors


def siemens_1fn3_linear_motors_are_plausible(tables):
    if not have(tables, SIEMENS_1FN3):
        return []
    # kE phase to neutral: kF = 3 kE; kM,20 = kF / sqrt(3 RSTR,20); PV,N = 3 R(120 degC) IN^2, R(120) = 1.39 R20
    return _linear_motor_rows_are_plausible(tables[SIEMENS_1FN3], SIEMENS_1FN3, 3.0, 0.03, True, (1.3, 1.5))


def beckhoff_al8000_linear_motors_are_plausible(tables):
    if not have(tables, BECKHOFF_AL8000):
        return []
    # kE phase to phase: kF = sqrt(3) kE; kM = sqrt(printed km^2) = Fc / sqrt(PL)
    return _linear_motor_rows_are_plausible(tables[BECKHOFF_AL8000], BECKHOFF_AL8000, math.sqrt(3), 0.12, False, (1.0, 1.6))


if __name__ == "__main__":
    import csv
    root = r"D:\Entwicklung\KinematicsSimulation\.claude\worktrees\fix-netlist-nodes-badf62\catalog"
    tables = {p: list(csv.DictReader(open(f"{root}/{p}", encoding="utf-8"))) for p in (SIEMENS_1FN3, BECKHOFF_AL8000)}
    print(siemens_1fn3_linear_motors_are_plausible(tables))
    print(beckhoff_al8000_linear_motors_are_plausible(tables))
    tables[SIEMENS_1FN3][5]["continuous_force_n"] = "20"
    tables[BECKHOFF_AL8000][3]["force_constant_n_a"] = "7.5"
    print(siemens_1fn3_linear_motors_are_plausible(tables))
    print(beckhoff_al8000_linear_motors_are_plausible(tables))


# --- motor-shafts
SHAFT_TABLES = ["siemens/motor_shafts.csv", "beckhoff/motor_shafts.csv", "sew/motor_shafts.csv", "yaskawa/motor_shafts.csv"]

def motor_shafts_fit_their_flange(tables):
    """Motor shaft/flange tables: shaft < spigot < bolt circle, the bolt circle fits on the flange square
    (between its edge and its diagonal), a shaft end 1.5 to 3.5 diameters long, and a bigger spigot never comes
    with a thinner shaft within one series."""
    errors = []
    for path in SHAFT_TABLES:
        if not have(tables, path):
            continue
        rows, by_series = tables[path], {}
        for r in rows:
            name = f"{path} {r['part_number_prefix']}"
            try:
                d, e, n = need(r, "shaft_diameter_m"), need(r, "shaft_length_m"), need(r, "centering_diameter_m")
                m, ab = need(r, "bolt_circle_diameter_m"), num(r["flange_square_m"])
            except ValueError as err:
                errors.append(f"{name}: {err}"); continue
            if not d < n < m:
                errors.append(f"{name}: not shaft {d} < spigot {n} < bolt circle {m}")
            if ab is not None and not (n < ab and 0.95 * ab < m < ab * math.sqrt(2)):
                errors.append(f"{name}: bolt circle {m} / spigot {n} do not fit the flange square {ab}")
            if not 1.5 * d <= e <= 3.5 * d:
                errors.append(f"{name}: shaft length {e} is not 1.5 to 3.5 x the diameter {d}")
            by_series.setdefault(r["series"], []).append((n, d, name))
        for series, sizes in by_series.items():
            sizes.sort()
            errors += [f"{b[2]}: spigot {b[0]} > {a[0]} with a thinner shaft than {a[2]}"
                       for a, b in zip(sizes, sizes[1:]) if b[1] < a[1]]
    return errors


# --- neugart
def neugart_planetary_gearboxes_are_plausible(tables):
    """Neugart PLE / PLN / PLFE (05.2026): T2D <= T2max <= T2Stop, n1D <= n1max, the ratio count per size as printed,
    backlash a whole arcmin that grows with the stages, efficiency falls with the stages, bigger sizes carry more."""
    counts = {"neugart/gearbox_ple.csv": {"PLE040": 24, "PLE060": 24, "PLE080": 24, "PLE120": 24, "PLE160": 12},
              "neugart/gearbox_pln.csv": {f"PLN{s}": 15 for s in ("070", "090", "115", "142", "190")},
              "neugart/gearbox_plfe.csv": {f"PLFE{s}": 16 for s in ("055", "064", "090", "110")}}
    errors = []
    for path, want in counts.items():
        if not have(tables, path):
            continue
        rows, per_size = tables[path], {}
        for r in rows:
            name = f"{r['size']}-{r['motor_frame']}-{r['ratio']}"
            try:
                per_size.setdefault(r["size"], []).append(r)
                t_d, t_max, t_stop = (need(r, c) for c in ("rated_output_torque_nm", "max_output_torque_nm",
                                                           "emergency_stop_output_torque_nm"))
                if not (0 < t_d <= t_max <= t_stop <= 3.5 * t_max):
                    errors.append(f"{name}: T2D {t_d}, T2max {t_max}, T2Stop {t_stop}")
                n_d, n_max = need(r, "rated_input_speed_rad_s"), need(r, "max_input_speed_rad_s")
                if not inside(n_d, 500 * PI / 30, n_max) or not inside(n_max, 5000 * PI / 30, 20000 * PI / 30):
                    errors.append(f"{name}: n1D {n_d * 30 / PI:.0f} r/min, n1max {n_max * 30 / PI:.0f} r/min")
                if abs(need(r, "clamping_hub_diameter_m") * 1000 - need(r, "motor_frame")) > 1e-9:
                    errors.append(f"{name}: clamping hub {r['clamping_hub_diameter_m']} m vs frame {r['motor_frame']}")
                stages, ratio = int(need(r, "stages")), need(r, "ratio")
                arcmin = need(r, "backlash_rad") / ARCMIN
                if abs(arcmin - round(arcmin)) > 1e-9 or not 1 <= arcmin <= 25:
                    errors.append(f"{name}: backlash {arcmin} arcmin")
                reduced = num(r["backlash_reduced_rad"])
                if reduced is not None and not reduced < need(r, "backlash_rad"):
                    errors.append(f"{name}: reduced backlash not below standard")
                eff = need(r, "efficiency_full_load")
                if not inside(eff, {1: 0.97, 2: 0.94, 3: 0.9}[stages], {1: 0.99, 2: 0.98, 3: 0.95}[stages]):
                    errors.append(f"{name}: efficiency {eff} for {stages} stages")
                # a ratio of one stage is 3..10; two stages 9..100; three 60..512 (as printed)
                if not inside(ratio, *{1: (3, 10), 2: (9, 100), 3: (60, 512)}[stages]):
                    errors.append(f"{name}: ratio {ratio} for {stages} stages")
                # range estimates: lower stiffness end, upper inertia end, per size and stage
                c2t, j1 = need(r, "torsional_stiffness_nm_rad"), need(r, "inertia_at_input_kgm2")
                if not inside(c2t / (1 / ARCMIN), 0.5, 250) or not inside(j1, 1e-7, 1e-2):
                    errors.append(f"{name}: C2t {c2t * ARCMIN:.2f} N m/arcmin, J1 {j1} kg m2")
                # the input inertia reflected to the output against the stiffness: natural frequency above 5 Hz
                if not j1 * ratio ** 2 < c2t / (2 * PI * 5) ** 2:
                    errors.append(f"{name}: J1 i^2 {j1 * ratio ** 2} vs C2t {c2t}: natural frequency below 5 Hz")
                mass, idle = need(r, "mass_kg"), num(r["no_load_torque_nm"])
                if mass is not None and not 0.2 < mass < 60:
                    errors.append(f"{name}: mass {mass} kg")
                if (idle is None) != (num(r["no_load_torque_speed_rad_s"]) is None) or (
                        idle is not None and not (0 < idle < 25 and abs(need(r, "no_load_torque_speed_rad_s") * MIN / (2 * PI) - 3000) < 1e-6)):
                    errors.append(f"{name}: idle torque {idle} at {r['no_load_torque_speed_rad_s']}")
            except ValueError as e:
                errors.append(f"{name}: {e}")
        if {s: len(v) for s, v in per_size.items()} != want:
            errors.append(f"{path}: rows per size {({s: len(v) for s, v in per_size.items()})}, want {want}")
        sizes = sorted(per_size, key=lambda s: int(s[-3:]))
        for a, b in zip(sizes, sizes[1:]):
            ra, rb = per_size[a], per_size[b]
            if not max(need(r, "max_output_torque_nm") for r in ra) < max(need(r, "max_output_torque_nm") for r in rb):
                errors.append(f"{a} -> {b}: top T2max does not rise")
            if not need(ra[0], "clamping_hub_diameter_m") < need(rb[0], "clamping_hub_diameter_m"):
                errors.append(f"{a} -> {b}: reference bore does not grow")
            if not need(ra[0], "max_input_speed_rad_s") >= need(rb[0], "max_input_speed_rad_s"):
                errors.append(f"{a} -> {b}: n1max rises")
            if not need(ra[0], "max_axial_force_n") < need(rb[0], "max_axial_force_n"):
                errors.append(f"{a} -> {b}: Fa max does not rise")
            for col in ("torsional_stiffness_nm_rad", "inertia_at_input_kgm2"):
                for stage in (1, 2, 3):
                    va = {need(r, col) for r in ra if r["stages"] == str(stage)}
                    vb = {need(r, col) for r in rb if r["stages"] == str(stage)}
                    if va and vb and not max(va) < min(vb):
                        errors.append(f"{a} -> {b}: {stage}-stage {col} does not rise")
            for stage in (1, 2, 3):
                ma = {num(r["mass_kg"]) for r in ra if r["stages"] == str(stage)} - {None}
                mb = {num(r["mass_kg"]) for r in rb if r["stages"] == str(stage)} - {None}
                if ma and mb and not max(ma) < min(mb):
                    errors.append(f"{a} -> {b}: {stage}-stage mass {ma} vs {mb}")
        for size, rs in per_size.items():
            by_stage = {}
            for r in rs:
                by_stage.setdefault(int(r["stages"]), set()).add((r["backlash_rad"], r["efficiency_full_load"]))
            if any(len(v) != 1 for v in by_stage.values()):
                errors.append(f"{size}: backlash or efficiency varies inside a stage")
            j_by_stage = [{r["inertia_at_input_kgm2"] for r in rs if r["stages"] == str(p)} for p in sorted(by_stage)]
            if any(len(j) != 1 for j in j_by_stage) or any(
                    float(next(iter(x))) < float(next(iter(y))) for x, y in zip(j_by_stage, j_by_stage[1:])):
                errors.append(f"{size}: input inertia not one per stage or rising with the stages")
            seq = [next(iter(by_stage[p])) for p in sorted(by_stage)]
            if any(float(x[0]) >= float(y[0]) or float(x[1]) <= float(y[1]) for x, y in zip(seq, seq[1:])):
                errors.append(f"{size}: backlash must grow and efficiency fall with the stages")
    return errors


# --- wittenstein
def wittenstein_gearboxes_are_plausible(tables):
    """SP+ and TP+ MF: keys unique, torques and speeds in order, values in sensible bands, one value per table where the
    document prints one, the ratios per size as printed, and a bigger size is stronger, stiffer and heavier."""
    errors = []
    expect = {  # path -> {size: (number of 1-stage ratios, number of 2-stage ratios)} as printed
        "wittenstein/gearbox_sp.csv": {"SP+060MF": (6, 11), "SP+075MF": (6, 11), "SP+100MF": (6, 11), "SP+140MF": (6, 11),
                                       "SP+180MF": (6, 11), "SP+210MF": (4, 9), "SP+240MF": (4, 9)},
        "wittenstein/gearbox_tp.csv": {"TP+004MF": (5, 15), "TP+010MF": (5, 15), "TP+025MF": (5, 15), "TP+050MF": (5, 15),
                                       "TP+110MF": (5, 15), "TP+300MF": (3, 10), "TP+500MF": (3, 10)},
    }
    for path, sizes in expect.items():
        if not have(tables, path):
            continue
        rows, seen, per = tables[path], set(), {}
        for r in rows:
            name = f"{r['size']}-{r['motor_frame']}-{r['ratio']}"
            try:
                key = (r["size"], r["motor_frame"], r["ratio"])
                if key in seen:
                    errors.append(f"{name} twice")
                seen.add(key)
                if "-" in r["size"] or r["size"] not in sizes:
                    errors.append(f"{name}: size not of {path}")
                stages, ratio = need(r, "stages"), need(r, "ratio")
                if not (stages == 1 and 3 <= ratio <= 10 or stages == 2 and 16 <= ratio <= 100):
                    errors.append(f"{name}: {stages:g} stages for ratio {ratio:g}")
                rated, peak, emergency = (need(r, c) for c in ("rated_output_torque_nm", "max_output_torque_nm", "emergency_stop_output_torque_nm"))
                if not 0 < rated <= peak < emergency:
                    errors.append(f"{name}: torques not rated <= max < emergency stop")
                if not 0 < need(r, "rated_input_speed_rad_s") < need(r, "max_input_speed_rad_s"):
                    errors.append(f"{name}: n1N not below n1Max")
                if not inside(need(r, "max_input_speed_rad_s"), 2000 * PI / 30, 10000 * PI / 30):
                    errors.append(f"{name}: max_input_speed")
                if not 0 < need(r, "backlash_reduced_rad") < need(r, "backlash_rad") <= 6 * ARCMIN:
                    errors.append(f"{name}: backlash")
                if not inside(need(r, "efficiency_full_load"), 0.9, 0.98):
                    errors.append(f"{name}: efficiency_full_load")
                # The no-load torque at the input is a loss: below the rated torque referred to the input.
                if not 0 < need(r, "no_load_torque_nm") < rated / ratio:
                    errors.append(f"{name}: no_load_torque_nm")
                hub = need(r, "clamping_hub_diameter_m")
                if not (inside(hub, 0.011, 0.060) and close(hub * 1000, float(r["motor_frame"]), 1e-9)):
                    errors.append(f"{name}: clamping hub")
                if not inside(need(r, "inertia_at_input_kgm2"), 1e-6, 3e-2):
                    errors.append(f"{name}: inertia_at_input_kgm2")
                # Stiffness per output torque: Ct21 / T2B between 100 and 2000 per rad for these gearboxes.
                if not inside(need(r, "torsional_stiffness_nm_rad") / peak, 100, 2000):
                    errors.append(f"{name}: torsional_stiffness_nm_rad")
                if num(r["max_radial_force_n"]) is not None and not need(r, "max_radial_force_n") > need(r, "max_axial_force_n") * 0.5:
                    errors.append(f"{name}: radial force")
                g = per.setdefault((r["size"], r["stages"]), {"ratios": set(), "one": {}})
                g["ratios"].add(r["ratio"])
                # One value per table: these are printed once and must be the same in every row of the table.
                for c in ("backlash_rad", "backlash_reduced_rad", "max_axial_force_n", "max_radial_force_n",
                          "max_tilting_moment_nm", "efficiency_full_load", "no_load_torque_speed_rad_s"):
                    if g["one"].setdefault(c, r[c]) != r[c]:
                        errors.append(f"{name}: {c} differs within its table")
                # The same size, stages and ratio share every value but the inertia across hubs.
                same = g.setdefault("by_ratio", {}).setdefault(r["ratio"], r)
                for c in ("rated_output_torque_nm", "max_output_torque_nm", "mass_kg", "torsional_stiffness_nm_rad"):
                    if same[c] != r[c]:
                        errors.append(f"{name}: {c} differs between hubs")
            except ValueError as e:
                errors.append(f"{name}: {e}")
        for size, counts in sizes.items():
            for stages, n in zip(("1", "2"), counts):
                got = len(per.get((size, stages), {"ratios": ()})["ratios"])
                if got != n:
                    errors.append(f"{path}: {size} {stages}-stage has {got} ratios, the document {n}")
        # A bigger size: higher emergency stop torque, tilting moment and mass (same stages).
        order = list(sizes)
        for stages in ("1", "2"):
            prev = None
            for size in order:
                g = per.get((size, stages))
                if not g:
                    continue
                rs = list(g["by_ratio"].values())
                cur = (max(need(r, "emergency_stop_output_torque_nm") for r in rs), need(rs[0], "max_tilting_moment_nm"),
                       min(need(r, "mass_kg") for r in rs))
                if prev and not all(c > p for c, p in zip(cur, prev[1])):
                    errors.append(f"{path}: {size} {stages}-stage not stronger and heavier than {prev[0]}")
                prev = (size, cur)
    return errors


MECH_CHECKS = [
    wittenstein_gearboxes_are_plausible,
    neugart_planetary_gearboxes_are_plausible,
    apex_gearboxes_agree_with_their_own_physics,
    harmonicdrive_strain_wave_gears_are_plausible,
    hiwin_screws_and_guides_agree_with_their_own_geometry,
    siemens_1fn3_linear_motors_are_plausible,
    beckhoff_al8000_linear_motors_are_plausible,
    gates_belts_are_plausible,
    megadyne_min_pulley_teeth_is_plausible,
    single_belt_axes_agree_with_their_geometry,
    motor_shafts_fit_their_flange,
]
