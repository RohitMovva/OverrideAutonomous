#!/usr/bin/env python3
"""Standardized visualization CLI for robot logs.

Replaces the legacy scripts in tests/log_analyis/:

    error_visualizer.py                → visualize.py tracking <log>
    theta_grapher.py                   → visualize.py theta <log>
    velocity_grapher.py                → visualize.py outputs <log>
    velocity_tuner.py                  → visualize.py wheels <log> [--integral]
    drivetrain_readings_visualizer.py  → visualize.py encoders <log>
    robot_animator.py / test.py        → visualize.py animate <log> --show
    goal_animator.py                   → visualize.py animate-mp <log> --show
    velo_checker.py                    → visualize.py animate-wheels <log> --show
    (new)                              → visualize.py field | dashboard | all <log>

Examples:
    python3 tests/visualize.py dashboard state.txt --run all
    python3 tests/visualize.py tracking square.txt --show
    python3 tests/visualize.py animate erie.txt --run 3 --gif
    python3 tests/visualize.py all square.txt

Static plots save PNGs to tests/output/<log>/ by default; --show opens
interactive windows instead. Animations are interactive by default
(require --show or --gif).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))

STATIC = ["tracking", "theta", "outputs", "wheels", "encoders", "field", "dashboard"]
POOLED = ["gains"]   # per-file figures pooling all selected runs
ANIMS = ["animate", "animate-mp", "animate-wheels"]


def resolve_log(name: str) -> Path:
    p = Path(name)
    if p.exists():
        return p
    p = HERE / "logs" / name
    if p.exists():
        return p
    sys.exit(f"error: log file '{name}' not found (looked in cwd and tests/logs/)")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=STATIC + POOLED + ANIMS + ["all"],
                    help="plot type ('all' = every static plot)")
    ap.add_argument("log", help="log file name (resolved against tests/logs/) or path")
    ap.add_argument("--session", type=int, default=None,
                    help="session index within the log (default: all sessions)")
    ap.add_argument("--run", default="0",
                    help="run index among selected runs, or 'all' (default: 0)")
    ap.add_argument("--out", default=str(HERE / "output"),
                    help="output directory for PNGs (default: tests/output)")
    ap.add_argument("--show", action="store_true",
                    help="open interactive windows instead of saving PNGs")
    ap.add_argument("--gif", action="store_true",
                    help="save animations as GIF (requires pillow)")
    ap.add_argument("--integral", action="store_true",
                    help="wheels: plot cumulative distance instead of velocity")
    ap.add_argument("--source", choices=["wheel_setpoints", "wheel_actuals"],
                    default="wheel_setpoints",
                    help="animate-wheels: which wheel velocities to integrate")
    ap.add_argument("--track-width", type=float, default=14.0,
                    help="animate-wheels: track width for diff-drive kinematics")
    ap.add_argument("--step", type=int, default=2,
                    help="animation frame stride (higher = faster playback)")
    args = ap.parse_args()

    import matplotlib
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from log_analysis import plots
    from log_analysis.animate import ANIMATIONS
    from log_analysis.parser import parse_log, select_runs

    log_path = resolve_log(args.log)
    sessions = parse_log(log_path)
    try:
        runs = select_runs(sessions, args.session, args.run)
    except IndexError as e:
        sys.exit(f"error: {e}")
    if not runs:
        sys.exit("error: no trajectory runs with enough control ticks in selection")

    out_dir = Path(args.out) / log_path.stem
    if not args.show:
        out_dir.mkdir(parents=True, exist_ok=True)

    if args.command in ANIMS:
        if not (args.show or args.gif):
            sys.exit("animations are interactive: pass --show (window) or --gif (file)")
        for run in runs:
            builder = ANIMATIONS[args.command]
            kwargs = {"step": args.step}
            if args.command == "animate-wheels":
                kwargs.update(source=args.source, track_width=args.track_width)
            try:
                fig, anim = builder(run, **kwargs)
            except ValueError as e:
                print(f"skip {run.label}: {e}")
                continue
            if args.gif:
                try:
                    from matplotlib.animation import PillowWriter
                except ImportError:
                    sys.exit("GIF export requires pillow: pip install pillow")
                path = out_dir / f"{run.label}_{args.command}.gif"
                out_dir.mkdir(parents=True, exist_ok=True)
                anim.save(path, writer=PillowWriter(fps=int(1000 / 25)))
                print(path)
                plt.close(fig)
            else:
                plt.show()
        return

    # Pooled figures: one per selection, not one per run
    pooled = POOLED if args.command == "all" else \
        [c for c in POOLED if c == args.command]
    for cmd in pooled:
        label = runs[0].label if len(runs) == 1 else "pooled"
        fig = plots.POOLED_FIGURES[cmd](runs, label=label)
        if args.show:
            plt.show()
        else:
            path = out_dir / f"{label}_{cmd}.png"
            fig.savefig(path, dpi=130, bbox_inches="tight")
            print(path)
        plt.close(fig)
    if args.command in POOLED:
        return

    commands = STATIC if args.command == "all" else [args.command]
    for run in runs:
        for cmd in commands:
            fn = plots.FIGURES[cmd]
            if cmd == "wheels":
                fig = fn(run, integral=args.integral)
            else:
                fig = fn(run)
            if args.show:
                try:  # optional hover tooltips (was velocity_tuner.py's mplcursors)
                    import mplcursors
                    mplcursors.cursor(fig.axes, hover=True)
                except ImportError:
                    pass
                plt.show()
            else:
                suffix = "_integral" if (cmd == "wheels" and args.integral) else ""
                path = out_dir / f"{run.label}_{cmd}{suffix}.png"
                fig.savefig(path, dpi=130, bbox_inches="tight")
                print(path)
            plt.close(fig)


if __name__ == "__main__":
    main()
