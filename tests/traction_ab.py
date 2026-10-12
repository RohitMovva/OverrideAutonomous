#!/usr/bin/env python3
"""Double vs single traction A/B over the path-following suite runs.

    tests/.venv/bin/python tests/traction_ab.py                       # every suite / traction_ab run in tests/logs
    tests/.venv/bin/python tests/traction_ab.py --untagged-as 2       # count logs without the tag as double traction
    tests/.venv/bin/python tests/traction_ab.py --since-boot 1001     # only boots from 1001 on (MMDDYYYY-<boot>.txt)
    tests/.venv/bin/python tests/traction_ab.py 10092026-1001.txt     # just these logs
    tests/.venv/bin/python tests/traction_ab.py --a 10092026-1001.txt --b 10102026-1010.txt
    tests/.venv/bin/python tests/traction_ab.py --exclude 10092026-1001.txt:6

Works run by run: each run's route is the "trajectory '<folder/name>' loaded" line before
it (Run.route), so one boot can hold any number of routes. A route counts when its folder
is suite/ or traction_ab/ (VexAutonomousPlanner/suite); logs copied in as
suite_<route>-<n>.txt from before Run.route existed still work by file name.

Runs are grouped by the "tractionWheels=" field on the firmware's "Build config" line
(Config::TRACTION_WHEELS_PER_SIDE, from 2026-10-09). Logs without it are "untagged" unless
--untagged-as says which configuration they were, or --a / --b name the groups outright.

Runs listed in tests/traction_ab_excludes.txt (LOG:RUN, RUN the 0-based index among the
log's trajectory runs, then a reason) are left out, as are any given with --exclude: a run
that hit something says nothing about traction.

Spin section (spin_w1..w4, spin_max): the turn-cost fit (extra voltage per wheel =
ks + kv * |w|, compare Config::KS_ANGULAR / KV_ANGULAR), effective track width, top spin
rate and acceleration, tracking-wheel offsets, the creep RAMSETE commands while spinning,
and how far the pivot wanders. Route section: error, saturation and duration per route.
Run 3 of each route per configuration on a charged battery, after fixing anything
one-sided (a dragging drive side biases every number here).
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from statistics import mean

sys.path.insert(0, str(Path(__file__).parent))

from log_analysis import metrics  # noqa: E402
from log_analysis.parser import Run, parse_log, trajectory_runs  # noqa: E402
from log_analysis.robot_config import (  # noqa: E402
    KS_ANGULAR, KV_ANGULAR, LATERAL_WHEEL_OFFSET, VERTICAL_WHEEL_OFFSET)

LOG_DIR = Path(__file__).parent / "logs"
EXCLUDES_FILE = Path(__file__).parent / "traction_ab_excludes.txt"
SUITE_FOLDERS = ("suite", "traction_ab")
SUITE_NAME = re.compile(r"^suite_(.+)-\d+\.txt$")
BOOT_NAME = re.compile(r"^\d{8}-(\d+)\.txt$")
ROUTE_LINE = re.compile(rb"trajectory '(?:suite|traction_ab)/")


@dataclass
class Entry:
    log: str
    index: int
    route: str
    run: Run

    @property
    def key(self) -> str:
        return f"{self.log}:{self.index}"


def fmt(x, spec=".2f"):
    return format(x, spec) if isinstance(x, (int, float)) and math.isfinite(x) else "-"


def avg(rows, key):
    vals = [r[key] for r in rows if isinstance(r.get(key), (int, float)) and math.isfinite(r[key])]
    return mean(vals) if vals else math.nan


def read_excludes(path: Path = EXCLUDES_FILE) -> dict[str, str]:
    """LOG:RUN -> reason, from the checked-in exclusion list."""
    out = {}
    if path.exists():
        for line in path.read_text().splitlines():
            line = line.split("#", 1)[0].strip()
            if line:
                key, _, reason = line.partition(" ")
                out[key] = reason.strip()
    return out


def suite_route(run: Run, log: str) -> str | None:
    """The suite route a run followed, without its folder, or None if not a suite run."""
    if run.route:
        folder, _, name = run.route.rpartition("/")
        return name if folder in SUITE_FOLDERS else None
    m = SUITE_NAME.match(log)
    return m.group(1) if m else None


def candidate_logs(since_boot: int | None = None) -> list[str]:
    """Logs in LOG_DIR that hold a suite / traction_ab run (cheap byte scan first). With
    since_boot, only imported boots (MMDDYYYY-<boot>.txt) numbered since_boot or later."""
    names = []
    for p in sorted(LOG_DIR.glob("*.txt")):
        if since_boot is not None:
            m = BOOT_NAME.match(p.name)
            if not m or int(m.group(1)) < since_boot:
                continue
        if SUITE_NAME.match(p.name) or ROUTE_LINE.search(p.read_bytes()):
            names.append(p.name)
    return names


def load(names, excludes: dict[str, str]) -> list[Entry]:
    entries = []
    for name in names:
        path = LOG_DIR / name
        if not path.exists():
            print(f"skipping {name}: not in {LOG_DIR}")
            continue
        for i, run in enumerate(trajectory_runs(parse_log(path))):
            route = suite_route(run, name)
            if route is None:
                continue
            e = Entry(name, i, route, run)
            if e.key in excludes:
                print(f"excluded {e.key} ({route}): {excludes[e.key] or 'listed'}")
                continue
            entries.append(e)
    return entries


def group_entries(args, excludes) -> dict[str, list[Entry]]:
    if args.a or args.b:
        return {"A": load(args.a or [], excludes), "B": load(args.b or [], excludes)}
    groups: dict[str, list[Entry]] = defaultdict(list)
    for e in load(args.logs or candidate_logs(args.since_boot), excludes):
        n = metrics.traction_wheels(e.run) or args.untagged_as
        groups[f"{n} traction/side" if n else "untagged"].append(e)
    return dict(sorted(groups.items()))


def spin_section(groups):
    print("SPINS (spin_w1..w4, spin_max)")
    print(f"  config: KS_ANGULAR={KS_ANGULAR:g} KV_ANGULAR={KV_ANGULAR:g} "
          f"LATERAL_WHEEL_OFFSET={LATERAL_WHEEL_OFFSET:+g} VERTICAL_WHEEL_OFFSET={VERTICAL_WHEEL_OFFSET:+g}")
    for label, entries in groups.items():
        spins = [e for e in entries if e.route.startswith("spin_")]
        if not spins:
            continue
        print(f"\n  [{label}]")
        print(f"    {'run':22s} {'route':9s} {'plan':>5s} {'rate':>5s} {'deliv':>5s} {'scrubL':>6s} {'scrubR':>6s} "
              f"{'trackW':>6s} {'top':>5s} {'accel':>5s} {'latOff':>6s} {'vertOff':>7s} {'ccw-cw':>6s} "
              f"{'creep':>6s} {'slip/rad':>8s} {'drift/rev':>9s}")
        rows, fit_runs, top_rows = [], [], []
        for e in sorted(spins, key=lambda e: (e.route, e.key)):
            sc, off, cr = metrics.spin_scrub(e.run), metrics.tracking_wheel_offsets(e.run), metrics.spin_creep(e.run)
            row = {**sc, **off, **cr}
            rows.append(row)
            if e.route == "spin_max":
                top_rows.append(row)
            elif sc.get("saturated_fraction", 1.0) <= 0.3:
                fit_runs.append(e.run)
            gap = off.get("vertical_ccw_in", math.nan) - off.get("vertical_cw_in", math.nan)
            print(f"    {e.key:22s} {e.route:9s} {fmt(sc.get('plan_rate_mean')):>5s} {fmt(sc.get('rate_mean')):>5s} "
                  f"{fmt(sc.get('delivered_ratio')):>5s} {fmt(sc.get('scrub_v_left'), '.1f'):>6s} "
                  f"{fmt(sc.get('scrub_v_right'), '.1f'):>6s} {fmt(sc.get('effective_track_width_in')):>6s} "
                  f"{fmt(sc.get('top_rate')):>5s} {fmt(sc.get('top_accel'), '.1f'):>5s} "
                  f"{fmt(off.get('lateral_offset_in')):>6s} {fmt(off.get('vertical_offset_in')):>7s} "
                  f"{fmt(gap):>6s} {fmt(cr.get('creep_in_per_rad')):>6s} "
                  f"{fmt(off.get('lateral_residual_in_per_rad'), '.3f'):>8s} "
                  f"{fmt(off.get('pivot_drift_in_per_rev')):>9s}")
        fit = metrics.fit_spin_scrub(fit_runs) if fit_runs else {"n": 0}
        if "ks" in fit:
            print(f"    turn cost fit: ks={fit['ks']:.1f} kv={fit['kv']:.1f} (r2 {fit['r2']:.2f}, {fit['n']} ticks, "
                  f"{fit['rate_min']:.1f}-{fit['rate_max']:.1f} rad/s) | left ks={fit['ks_left']:.1f} "
                  f"kv={fit['kv_left']:.1f} | right ks={fit['ks_right']:.1f} kv={fit['kv_right']:.1f}")
        else:
            print(f"    turn cost fit: not enough spin rates yet ({fit.get('n', 0)} plateau ticks; "
                  f"needs two or more of spin_w1..w4)")
        print(f"    means: track width {fmt(avg(rows, 'effective_track_width_in'))} in, "
              f"lateral offset {fmt(avg(rows, 'lateral_offset_in'))} in, vertical offset "
              f"{fmt(avg(rows, 'vertical_offset_in'))} in, creep {fmt(avg(rows, 'creep_in_per_rad'))} in/rad, "
              f"pivot drift {fmt(avg(rows, 'pivot_drift_in_per_rev'))} in/rev")
        if top_rows:
            print(f"    spin_max: top rate {fmt(avg(top_rows, 'top_rate'))} rad/s, "
                  f"top spin-up accel {fmt(avg(top_rows, 'top_accel'), '.1f')} rad/s^2")


def route_section(groups):
    print("\nROUTES (mean over runs)")
    routes = sorted({e.route for entries in groups.values() for e in entries if not e.route.startswith("spin_")})
    print(f"  {'route':20s} {'group':18s} {'runs':>4s} {'dur s':>6s} {'final in':>8s} "
          f"{'xtrack rms':>10s} {'head p95 deg':>12s} {'sat L/R %':>10s}")
    for route in routes:
        for label, entries in groups.items():
            rows = []
            for e in entries:
                if e.route != route:
                    continue
                te, sat = metrics.tracking_errors(e.run), metrics.saturation(e.run)
                ticks = [tk.t for tk in e.run.ticks]
                rows.append({
                    "dur": ticks[-1] - ticks[0] if len(ticks) > 1 else math.nan,
                    "final": te.get("final_position_error_in", math.nan),
                    "xtrack": te.get("cross_track", {}).get("rms", math.nan),
                    "head": math.degrees(te.get("heading", {}).get("p95", math.nan)),
                    "satl": 100 * sat.get("left_fraction", math.nan),
                    "satr": 100 * sat.get("right_fraction", math.nan),
                })
            if rows:
                print(f"  {route:20s} {label:18s} {len(rows):4d} {fmt(avg(rows, 'dur')):>6s} "
                      f"{fmt(avg(rows, 'final')):>8s} {fmt(avg(rows, 'xtrack')):>10s} "
                      f"{fmt(avg(rows, 'head'), '.1f'):>12s} "
                      f"{fmt(avg(rows, 'satl'), '.0f') + '/' + fmt(avg(rows, 'satr'), '.0f'):>10s}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="*", help="log names in tests/logs (default: every suite / traction_ab run)")
    ap.add_argument("--a", nargs="*", help="logs for configuration A (overrides tag grouping)")
    ap.add_argument("--b", nargs="*", help="logs for configuration B")
    ap.add_argument("--since-boot", type=int, help="only logs MMDDYYYY-<boot>.txt with boot >= this")
    ap.add_argument("--untagged-as", type=int, help="traction wheels per side for logs without the tag")
    ap.add_argument("--exclude", nargs="*", default=[], help="LOG:RUN entries to leave out, besides the excludes file")
    args = ap.parse_args()
    excludes = read_excludes()
    excludes.update({k: "--exclude" for k in args.exclude})
    groups = group_entries(args, excludes)
    if not any(groups.values()):
        print(f"no suite runs found in {LOG_DIR}")
        return 1
    for label, entries in groups.items():
        print(f"{label}: {len(entries)} run(s) from {len({e.log for e in entries})} log(s)")
    print()
    spin_section(groups)
    route_section(groups)
    return 0


if __name__ == "__main__":
    sys.exit(main())
