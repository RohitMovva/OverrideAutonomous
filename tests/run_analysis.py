#!/usr/bin/env python3
"""Full path-following analysis report over all logs in tests/logs/.

Usage:  python3 tests/run_analysis.py [logfile ...]
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from log_analysis import metrics
from log_analysis.parser import parse_log, trajectory_runs

LOG_DIR = Path(__file__).parent / "logs"
DEFAULT_LOGS = ["square.txt", "init.txt", "pikes.log", "erie.txt",
                "state.txt", "league_night_2.txt", "default.log"]


def fmt(x, spec=".2f"):
    try:
        return format(x, spec)
    except (TypeError, ValueError):
        return str(x)


def analyze_file(path: Path):
    sessions = parse_log(path)
    runs = trajectory_runs(sessions)
    n_ticks = sum(len(r.ticks) for r in runs)
    print(f"\n{'='*78}\n{path.name}: {len(sessions)} sessions, "
          f"{len(runs)} trajectory runs, {n_ticks} control ticks\n{'='*78}")
    if not runs:
        return None

    agg = {
        "cross_rms": [], "cross_max": [], "final_err": [], "head_p95": [],
        "vel_rms": [], "overrun_frac": [], "worst_stall": [], "sat": [],
        "nan": 0, "spikes": 0, "creep": 0, "unexplained_jumps": 0,
        "reset_jump_max": 0.0, "mid_run_stalls": 0, "zero_dt": 0,
        "lag": [],
    }

    for i, run in enumerate(runs):
        te = metrics.tracking_errors(run)
        tm = metrics.loop_timing(run)
        vt = metrics.velocity_tracking(run)
        nr = metrics.nan_report(run)
        st = metrics.saturation(run)
        pj = metrics.pose_jumps(run)
        vs = metrics.velocity_spikes(run)
        mv = metrics.min_voltage_substitution(run)
        ws = metrics.warning_summary(run)
        lag = metrics.velocity_lag_ticks(run)

        ct = te["cross_track"]
        hd = te["heading"]
        if ct.get("n"):
            agg["cross_rms"].append(ct["rms"])
            agg["cross_max"].append(ct["max"])
            agg["head_p95"].append(hd["p95"])
        if te["final_position_error_in"] == te["final_position_error_in"]:
            agg["final_err"].append(te["final_position_error_in"])
        for side in ("left_abs", "right_abs"):
            if vt[side].get("n"):
                agg["vel_rms"].append(vt[side]["rms"])
        if tm.get("n"):
            agg["overrun_frac"].append(tm["overrun_fraction"])
            agg["worst_stall"].append(tm["worst_stall_ms"])
        if st.get("n"):
            agg["sat"].append(max(st["left_fraction"], st["right_fraction"]))
        agg["nan"] += sum(nr.values())
        agg["spikes"] += vs["count"]
        agg["creep"] += mv["creep_count"]
        agg["unexplained_jumps"] += pj["unexplained_jump_count"]
        agg["reset_jump_max"] = max(agg["reset_jump_max"], pj["reset_jump_max_in"])
        agg["mid_run_stalls"] += ws["mid_run_stall_count"]
        agg["zero_dt"] += ws["zero_dt_count"]
        if lag is not None:
            agg["lag"].append(lag)

        if len(runs) <= 8:  # per-run detail for small files
            print(f"\n-- run {i}: {len(run.ticks)} ticks, route_size={run.route_size}, "
                  f"initial={run.initial_pose}")
            if ct.get("n"):
                print(f"   cross-track |err|: rms={ct['rms']:.2f} max={ct['max']:.2f} in | "
                      f"heading |err|: p95={hd['p95']:.3f} max={hd['max']:.3f} rad")
                print(f"   final error: {te['final_position_error_in']:.2f} in, "
                      f"{te['final_heading_error_rad']:.3f} rad")
            if tm.get("n"):
                print(f"   loop period: mean={tm['mean']*1000:.1f}ms p95={tm['p95']*1000:.1f}ms "
                      f"worst={tm['worst_stall_ms']:.0f}ms overruns={tm['overrun_count']} "
                      f"({100*tm['overrun_fraction']:.1f}%) slip={tm['cumulative_slip_ms']:.0f}ms")
            if vt["left_abs"].get("n"):
                print(f"   vel tracking |err|: L rms={vt['left_abs']['rms']:.1f} "
                      f"R rms={vt['right_abs']['rms']:.1f} in/s, lag={lag} ticks")
            if nr:
                print(f"   NaN/inf: {nr}")
            if st.get("n"):
                print(f"   saturation: L={100*st['left_fraction']:.1f}% R={100*st['right_fraction']:.1f}%")
            if vs["count"]:
                print(f"   impossible velocities: {vs['count']} (worst {vs['worst']:.0f} in/s)")
            if pj["reset_jump_count"] or pj["unexplained_jump_count"]:
                print(f"   pose jumps: {pj['reset_jump_count']} at resets "
                      f"(max {pj['reset_jump_max_in']:.1f} in), "
                      f"{pj['unexplained_jump_count']} unexplained")
            if mv["creep_count"]:
                print(f"   creep-at-stop ticks: {mv['creep_count']}")
            if ws["invalid_deltatime_count"]:
                print(f"   invalid deltaTime warnings: {ws['invalid_deltatime_count']} "
                      f"(zero-dt {ws['zero_dt_count']}, mid-run stalls {ws['mid_run_stall_count']}, "
                      f"worst {ws['worst_stall_s']*1000:.0f} ms)")
            mr = metrics.motion_regimes(run)
            if mr["turn"]["n"] >= 50 and mr["straight"]["n"] >= 50:
                print(f"   regime gain act/sp: straight={mr['straight']['gain']:.2f} "
                      f"arc={fmt(mr['arc']['gain'])} turn={mr['turn']['gain']:.2f} "
                      f"(turn/straight {fmt(mr['turn_to_straight_ratio'])})")

    def m(xs):
        return sum(xs) / len(xs) if xs else float("nan")

    print(f"\nFILE SUMMARY ({path.name})")
    print(f"  cross-track RMS (mean over runs): {m(agg['cross_rms']):.2f} in "
          f"(worst run max {max(agg['cross_max'], default=float('nan')):.2f} in)")
    print(f"  final position error: mean {m(agg['final_err']):.2f} in, "
          f"worst {max(agg['final_err'], default=float('nan')):.2f} in")
    print(f"  heading err p95 (mean over runs): {m(agg['head_p95']):.3f} rad")
    print(f"  wheel-vel tracking RMS (mean): {m(agg['vel_rms']):.1f} in/s, "
          f"median lag {sorted(agg['lag'])[len(agg['lag'])//2] if agg['lag'] else '?'} ticks")
    print(f"  loop overrun fraction (mean): {100*m(agg['overrun_frac']):.1f}%, "
          f"worst stall {max(agg['worst_stall'], default=float('nan')):.0f} ms")
    print(f"  saturation fraction (worst side, mean): {100*m(agg['sat']):.1f}%")
    print(f"  NaN/inf values: {agg['nan']} | impossible velocities: {agg['spikes']} | "
          f"creep-at-stop ticks: {agg['creep']}")
    print(f"  pose jumps: {agg['unexplained_jumps']} unexplained, "
          f"largest reset correction {agg['reset_jump_max']:.1f} in")
    print(f"  odometry: {agg['zero_dt']} zero-dt ticks, {agg['mid_run_stalls']} mid-run stalls")

    # Pooled gain / feasibility analyses (regimes are sparse per run)
    mr = metrics.motion_regimes(runs)
    ss = metrics.steady_state_gain(runs)
    sf = metrics.setpoint_feasibility(runs)
    sc = metrics.saturation_cost(runs)
    if mr["straight"]["n"]:
        sd_s, sd_t = mr["straight"].get("heading_err_sd_rad"), mr["turn"].get("heading_err_sd_rad")
        print(f"  wheel gain act/sp by regime: straight={fmt(mr['straight']['gain'], '.3f')} "
              f"(n={mr['straight']['n']}), arc={fmt(mr['arc']['gain'], '.3f')} "
              f"(n={mr['arc']['n']}), turn={fmt(mr['turn']['gain'], '.3f')} "
              f"(n={mr['turn']['n']}, turn/straight {fmt(mr['turn_to_straight_ratio'])})")
        if sd_s is not None or sd_t is not None:
            print(f"  heading err spread: straight={fmt(math.degrees(sd_s) if sd_s else None, '.1f')} deg, "
                  f"in-place turn={fmt(math.degrees(sd_t) if sd_t else None, '.1f')} deg")
    if ss["left"]["n"] or ss["right"]["n"]:
        print(f"  steady-state gain: L={fmt(ss['left']['gain'], '.3f')} (n={ss['left']['n']}) "
              f"R={fmt(ss['right']['gain'], '.3f')} (n={ss['right']['n']}), "
              f"asymmetry {fmt(100 * ss['asymmetry'] if ss['asymmetry'] is not None else None, '.1f')}%")
    if sf["n"]:
        print(f"  infeasible wheel setpoints (>{sf['vmax_in_s']:.0f} in/s): "
              f"{100 * sf['infeasible_fraction']:.2f}% "
              f"(worst commanded {sf['max_setpoint_in_s']:.0f} in/s)")
    if sc["saturated"].get("n"):
        print(f"  |vel err| saturated vs not: {sc['saturated']['mean']:.1f} vs "
              f"{sc['unsaturated']['mean']:.1f} in/s "
              f"({fmt(sc['mean_ratio'])}x, {sc['saturated']['n']} saturated wheel-ticks)")
    return agg


def main():
    names = sys.argv[1:] or DEFAULT_LOGS
    for name in names:
        path = LOG_DIR / name if not Path(name).exists() else Path(name)
        if not path.exists():
            print(f"skip {name}: not found")
            continue
        analyze_file(path)


if __name__ == "__main__":
    main()
