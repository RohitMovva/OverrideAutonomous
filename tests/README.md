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
- `log_analysis/metrics.py` — tracking error (incl. signed along/cross-track bias), loop timing, velocity tracking, NaN detection, saturation, pose jumps, velocity spikes, creep-at-stop, MP-vs-RAMSETE divergence, dead-reckoning, effective track width, reverse-driving, per-motor encoder divergence (as a fraction of travel — the
  logged positions are absolute and untared, so raw spread is meaningless),
  distance-integral drift, regime-split wheel gain (straight/arc/in-place-turn), steady-state gain & left/right asymmetry, setpoint feasibility, saturation cost
- `log_analysis/plots.py` / `log_analysis/animate.py` — figures and field animations
- `log_analysis/health_parser.py` / `log_analysis/health_plots.py` — drive motor health stream (per-motor temperature, current, power, efficiency, over-temp) and the coupled-partner comparison that finds a bad motor
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

python3 tests/visualize.py lift 07222026.txt --run all     # lift/chainbar debug dashboard
pytest tests/test_lift.py -v                               # lift log-format checks

python3 tests/visualize.py health 08012026-10.txt          # drive motor health dashboard
pytest tests/test_health.py -v                             # per-motor health invariants
```

### Lift / chainbar debug stream

The lift emits its own `LIFTDBG` / `LIFTCMD` / `LIFTEVT` lines from
`Lift::log_debug_state` (throttled ~20 Hz, opcontrol only — the lift is not
driven during trajectory following). `visualize.py lift <log>` draws a
target-vs-actual + commanded-voltage dashboard per boot (with command and
respool/startup-transition markers) for tuning the position PIDs, the chainbar
gravity feed-forward, and the respool stall threshold. Here `--run N` selects
the Nth boot that actually contains lift samples (`all` for every one).
`test_lift.py` round-trips the log format and asserts lift invariants (finite
values, `err == target-actual`, commands within ±12000 mV, respool only drives
up) over any log carrying lift data; logs without it are skipped. Parsing/plots
live in `log_analysis/lift_parser.py` and `log_analysis/lift_plots.py`.

### Drive motor health stream

`Robot::logDriveHealth` (`src/hardware/robot.cpp`) emits one `DRIVEHEALTH` line
per motor every `Robot::DRIVE_HEALTH_LOG_MS`, from **both** the trajectory loop
and the opcontrol heartbeat — motors keep heating while the robot is driven back
to the start, so the wear story only reads correctly across a whole boot. Here
`--run N` therefore selects the Nth boot carrying health samples, not a
trajectory run.

    DRIVEHEALTH auton L0 port=-18 tempC=35 curMA=810 W=5.2 eff=71 ovt=0

The diagnosis rests on one fact: **the two motors on a side are mechanically
coupled**, so they turn at the same speed under the same command. The healthy
one is therefore a built-in control for the sick one and no baseline run is
needed — which is why `health_dashboard` overlays partners in the same axes
(a column per side, a row per metric) instead of plotting sides against each
other. Read the fields together; each alone is ambiguous:

| field | what it isolates |
|---|---|
| `curMA` | the load that motor is actually carrying |
| `W` | with current, recovers the volts it really saw (`P/I`) — a motor not *receiving* its command (connector, wire, winding) reads low here while its partner is normal |
| `eff` | mechanical out over electrical in: the motor that gets its power and burns it as heat (bearings, drag). Only meaningful at matched speed, which the coupled pair gives for free |
| `tempC` / `ovt` | the V5 derates torque in steps as it heats, and nothing else in the log says so |

`partner_imbalance()` reports two numbers per side, deliberately separated:
`current_ratio` measures load sharing over samples where **both** motors are
engaged, and `cutout_frac` counts samples where one sits at ~0 mA while its
partner pulls. Pooling them hides the interesting case — in `08012026-10` the
right pair pools to x1.94, which reads as "two badly matched motors", but splits
into x1.13 while engaged (as well matched as the left pair's x1.02) plus port 15
contributing nothing in 32% of loaded samples, all of them at stall. One motor
cutting out under peak electrical stress is a different fault, and a different
repair, from two motors sharing badly.

Logs captured before the per-motor split (`08012026-9` is the only one) parse
too: the legacy per-side `L tempC=a,b curMA=a,b ovt=a,b` form expands to the
same per-motor model, with `W=`/`eff=` optional and ports resolved from the
`MotorGroup`s in `src/main.cpp` (`robot_config.DRIVE_PORTS`).

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
| (new) | `visualize.py health` — per-motor drive health / bad-motor detection |

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

The drive-health invariants in `test_health.py` are alarms in the same spirit —
`test_no_motor_cuts_out_under_load` currently fails on `08012026-9` and `-10`,
naming port 15 as contributing nothing in 32–38% of loaded samples. That failure
is the finding; it clears when the motor does.

All four are visualized in one figure: `visualize.py gains <log> --run all`
(regime scatter, steady-state gain histograms, setpoint feasibility,
saturation cost). It pools the selected runs, since in-place turns are
sparse within any single run.
