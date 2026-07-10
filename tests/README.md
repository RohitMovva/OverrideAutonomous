# Path-following test & analysis suite

Standardized system for analyzing the robot logs in `tests/logs/`:
invariant tests (pytest), a text report, and visualizations — all built on
one shared parser. Replaces the former `tests/log_analyis/` scripts,
`tests/test.py`, and `tests/reformater.py`.

See [`ISSUES.md`](ISSUES.md) for the full catalog of system issues these
tests found, with measurements, code locations, and the recommended fix
order.

## Setup

```bash
pip install -r tests/requirements.txt
# optional extras: pip install pillow mplcursors
```

## Layout

- `log_analysis/parser.py` — parses raw logs into sessions → trajectory runs → 10 ms control ticks (Pose/Goal/wheel velocities/voltages/feedforward/encoder positions/…)
- `log_analysis/metrics.py` — tracking error (incl. signed along/cross-track bias), loop timing, velocity tracking, NaN detection, saturation, pose jumps, velocity spikes, creep-at-stop, MP-vs-RAMSETE divergence, dead-reckoning, effective track width, reverse-driving, encoder disagreement, distance-integral drift, regime-split wheel gain (straight/arc/in-place-turn), steady-state gain & left/right asymmetry, setpoint feasibility, saturation cost
- `log_analysis/plots.py` / `log_analysis/animate.py` — figures and field animations
- `test_path_following.py` — pytest invariants, parametrized over every log file; each assertion message names the responsible source location
- `run_analysis.py` — per-run / per-file text report
- `visualize.py` — CLI for all plots and animations

## Usage

```bash
pytest tests/test_path_following.py -v            # invariant checks
python3 tests/run_analysis.py                     # text report, all logs
python3 tests/run_analysis.py state.txt           # single file

python3 tests/visualize.py dashboard state.txt --run all   # one PNG per run
python3 tests/visualize.py gains state.txt --run all       # pooled gain/feasibility view
python3 tests/visualize.py all square.txt                  # every static plot
python3 tests/visualize.py tracking erie.txt --run 3 --show
python3 tests/visualize.py animate square.txt --show       # or --gif
python3 tests/visualize.py animate-wheels square.txt --show --track-width 12.7
```

Static plots save to `tests/output/<log>/<session+run>_<plot>.png` by
default; `--show` opens interactive windows (with hover tooltips if
`mplcursors` is installed). Animations are interactive (`--show`) or saved
as GIF (`--gif`, needs `pillow`). Select data with `--session N` and
`--run N|all`. Log names are resolved against `tests/logs/`.

## Migration from the old scripts

| Old script | Replacement |
|---|---|
| `log_analyis/error_visualizer.py` | `visualize.py tracking <log>` |
| `log_analyis/theta_grapher.py` | `visualize.py theta <log>` |
| `log_analyis/velocity_grapher.py` | `visualize.py outputs <log>` |
| `log_analyis/velocity_tuner.py` | `visualize.py wheels <log>` (`--integral` for distance view) |
| `log_analyis/drivetrain_readings_visualizer.py` | `visualize.py encoders <log>` |
| `log_analyis/robot_animator.py`, `test.py` | `visualize.py animate <log> --show` |
| `log_analyis/goal_animator.py` | `visualize.py animate-mp <log> --show` |
| `log_analyis/velo_checker.py` | `visualize.py animate-wheels <log> --show` |
| `reformater.py` | obsolete — the parser reads logs directly |
| (new) | `visualize.py field`, `visualize.py dashboard`, `visualize.py gains`, `visualize.py all` |

Unlike the old scripts, nothing is hardcoded to one log file, all commands
understand multi-session logs, and every capability is also available as a
metric (`log_analysis/metrics.py`) with a matching pytest invariant.

`logs/splitter.py` (splits a raw log into per-boot session files) is kept
as a standalone preprocessing utility; the parser handles multi-session
files natively, so splitting is optional.

## Interpreting failures

Test failures are evidence, not noise: thresholds encode what a healthy
path follower should do, and each failure message points at the code
responsible (e.g. NaN feedforward → `trajectory.cpp:32`, pose teleports →
`distance_reset_odometry.cpp`, track-width mismatch → `config.hpp:13`).
Fix the robot code, re-log, and the tests measure the improvement.

The gain/feasibility invariants added after the 2026-07 competition-log
analysis encode its headline findings:

- `test_turning_gain_matches_straight_gain` — in-place turns deliver only
  ~71–79% of commanded wheel speed vs 95–98% straight (turning scrub) →
  the shared kS/kV/kA feedforward needs separate angular constants and/or
  gyro-rate heading feedback
- `test_steady_state_velocity_gain_near_unity` — settled cruise runs 2–5%
  under setpoint → KV mis-calibration (`config.hpp:18`)
- `test_drivetrain_sides_symmetric` — left/right gain mismatch is a
  hardware health alarm (right side ran ~5% weak all of state)
- `test_wheel_setpoints_feasible` / `test_saturation_cost_bounded` —
  infeasible commanded speeds (state routes hit ~200 in/s vs ~74 max) make
  saturated ticks effectively open-loop (15–26 in/s error vs 2–3 normally);
  fix the route generator limits and add wheel-speed desaturation

All four are visualized in one figure: `visualize.py gains <log> --run all`
(regime scatter, steady-state gain histograms, setpoint feasibility,
saturation cost). It pools the selected runs, since in-place turns are
sparse within any single run.
