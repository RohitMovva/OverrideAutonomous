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

## Getting logs off the card

The brain writes **one file per boot** to `/usd/logs/boot_NNNN.txt`
(`src/utilities/logger.cpp`; the V5 has no clock, so boots are numbered,
continuing from the highest number on the card). With the card auto-mounted
in WSL at `/mnt/sdcard` (see `VexAutonomousPlanner/tools/sdcard/`):

```bash
python3 tests/import_logs.py            # copy every new boot into tests/logs/
python3 tests/import_logs.py --open     # ...and open the new files in VS Code
python3 tests/import_logs.py --archive  # ...and move them to logs/imported/ on the card
python3 tests/import_logs.py --list     # what is on the card, and what is already in
```

Files are named `<MMDDYYYY>-<boot>.txt` (import date + boot number) and each
one is summarized as it lands (program type, route, trajectory runs,
duration, error/warning counts, which debug streams it carries). Re-running
is safe: already-imported boots are skipped by content hash
(`tests/logs/.imported.json`). Old-style single-file dumps
(`robot_log_*.txt`) are split per boot on import as `<MMDDYYYY>-legacy<N>.txt`,
so `logs/splitter.py` is no longer needed.

A boot cut short by power loss can leave a damaged FAT cluster chain behind,
and the card then refuses to serve that file at all (`OSError: [Errno 5]`,
with `FAT-fs ... invalid cluster chain` in `dmesg`). The import skips such a
file, names it on stderr with how many bytes are still readable, and exits 1
— the healthy boots around it are imported as usual. Salvage the readable
part with `dd if=<file> of=<dest> bs=512 conv=noerror,sync`, then repair the
card (`sdcard eject`, `fsck.vfat -a` the device, or `chkdsk /f` on Windows).
The same FAT error also flips the card to read-only, so re-insert it before
the next export.

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

python3 tests/visualize.py lift 07222026.txt --run all     # lift debug dashboard
pytest tests/test_lift.py -v                               # lift log-format checks

python3 tests/visualize.py health 08012026-10.txt          # drive motor health dashboard
pytest tests/test_health.py -v                             # per-motor health invariants

python3 tests/visualize.py walls 09302026-648.txt          # wall-reset sensors along the path
```

### Wall-reset sensor stream

`DistanceResetOdometry::update()` logs every tick (firmware from 2026-09-30):

    DISTREAD front=<mm> back=<mm> left=<mm> right=<mm>

Raw readings: `-1` no sensor on that side, `9999` no object, `2147483647` device
missing. The parser attaches them to the tick as `ControlTick.distance_mm`.

- `metrics.wall_estimate()` is `DistanceResetOdometry::reset`'s geometry offline
  (sensor offsets parsed from `src/main.cpp`, wall position from `config.hpp`).
- `metrics.wall_samples()` gives, per tick and per sensor squarely facing a wall,
  the correction a reset there would apply: steady bands are odometry drift, one
  sensor jumping alone is its beam clipping a field element.
- `visualize.py walls` plots those for X and Y; `run_analysis.py` lists them at
  every planned stop.
- `test_offline_wall_math_matches_firmware` checks the offline geometry against every
  correction the firmware logged (0.25 in). If it fails, resync `wall_estimate` with
  the firmware or the offsets in `main.cpp`.

Reset motion gates: a reset on a point that starts a `wait_lift` hold waits until the
robot is still (`DISTANCE_RESET_STILL_*`), logging `Distance reset (<side>) waits for the
robot to be still in the hold` and then `... <n> ms into the hold`. Any other reset is
tried at once, gated on the error the measured motion would add
(`DISTANCE_RESET_MAX_MOTION_ERROR`), so a side sensor can reset while driving along a wall.

### Tracking wheel stream

`Odometry::update()` logs every tick (firmware from 2026-10-08):

    TRACKWHEEL vert=<in> lat=<in> dvert=<in> dlat=<in> ddrive=<in> vok=<0|1> lok=<0|1>

`vert`/`lat` are the absolute tracking-wheel positions (nan when the sensor is
unreadable), `dvert`/`dlat` this tick's deltas (nan when not used), `ddrive` the
drive-encoder forward delta, and `vok`/`lok` whether odometry used each wheel. When a
wheel is unusable the vertical one falls back to the drive encoders and the lateral one
to "no sideslip", with one WARNING per outage. The parser attaches the line to the tick
as `ControlTick.tracking_wheel`.

- `metrics.tracking_wheel_agreement()` compares the vertical wheel against the drive
  encoders: the gap is drive-wheel slip (or an uncalibrated diameter).
- `test_tracking_wheels_read_every_tick` fails on any fallback tick,
  `test_vertical_tracking_wheel_direction` on a reversed vertical sensor, and
  `test_drive_slip_against_tracking_wheel` when drive and wheel disagree by more than
  5% of travel, and `test_pose_follows_vertical_wheel` when the pose's forward travel
  falls more than 5% short of what the wheel rolled (`wheel_abs_net_in` vs
  `pose_forward_in`): odometry rejected real wheel deltas, as in boot 989 (copied in as
  `suite_cruise_8ft-1.txt`), where the 20 ms sensor data rate made every fresh reading a
  two-tick jump. `tests/test_tracking_wheels.py` checks the parser and metric on
  synthetic logs; `tests/cpp/test_tracking_arc.cpp` checks the arc math.

### Spin tests and the traction A/B

The suite's spin routes (`spin_w1`..`spin_w4` at 1-4 rad/s, `spin_max` planned past the
robot's limit) spin 720° each way. Three metrics read them:

- `metrics.tracking_wheel_offsets(run)` fits the tracking wheels' offsets from the pivot
  (`Config::LATERAL_WHEEL_OFFSET`, `VERTICAL_WHEEL_OFFSET`) against the IMU, per spin
  direction too, plus the lateral wheel's sideways scrub per radian and how far the pivot
  wanders per revolution.
- `metrics.spin_scrub(run)` gives the voltage each wheel needs beyond the straight-line
  feedforward on the spin plateau, the effective track width, the delivered vs planned
  rate, and the top spin rate and spin-up acceleration.
- `metrics.fit_spin_scrub(runs)` regresses that extra voltage on |w| across two or more
  rates. Its intercept and slope estimate `KS_ANGULAR` and `KV_ANGULAR`, per side as well.

- `metrics.spin_creep(run)` is the forward speed RAMSETE commands per rad/s while the
  plan spins in place. It should be zero. A wrong `LATERAL_WHEEL_OFFSET` makes it positive:
  boot 1001 (3.0 in configured, 2.37 in measured) crept 0.72 in per radian.

Invariants: `test_tracking_wheel_offsets_match_config` (within 0.25 in),
`test_spin_pivot_same_both_ways` (CCW vs CW within 0.3 in),
`test_spin_commands_no_creep` (under 0.2 in per radian) and
`test_spin_delivers_planned_rate` (within 10% on unsaturated plateaus). Synthetic checks
are in `tests/test_spin_metrics.py`. `test_path_following.py` picks up every imported boot
that ran a `traction_ab/` route, as well as the `suite_*` copies.

`tests/traction_ab.py` works run by run. Each run's route comes from the "trajectory
'<folder/name>' loaded" line before it (`Run.route`), so one boot can hold many routes. It
prints the spin metrics, creep included, and per-route error, saturation and duration for
every `suite/` or `traction_ab/` run, grouped by the `tractionWheels=` field on the Build
config line (`Config::TRACTION_WHEELS_PER_SIDE`). Options:
`--untagged-as N` for logs from before the tag, `--since-boot N` to start at a boot,
positional log names to restrict, and `--a ... --b ...` for explicit groups. Runs listed
in `tests/traction_ab_excludes.txt` (`LOG:RUN reason`, e.g. a run that hit something) or
given with `--exclude LOG:RUN` are left out. Run three of each route per configuration on
a charged battery.

### Lift debug stream

The lift emits one `LIFTDBG` line per 10 ms `Lift::update()` from
`Lift::log_debug_state` (`src/hardware/lift.cpp`), in autonomous and opcontrol
alike — the lift state machine runs alongside trajectory following, not inside
it. The line carries mode/state, which subsystem owns the PTO (`pto=`) and
whether the shift has settled (`own=`), the tower and loader sensors, and
target-vs-actual-vs-command for the winch, the angler and the clamp:

```
LIFTDBG mode=MATCHLOAD state=ML_DESCEND pto=LIFT own=1 home=0 tower=9999 loader=9999 \
  | WINCH tgt=500.0 act=204.7 err=295.3 mv=12000 \
  | ANG tgt=91.0 set=88.4 act=-6.2 err=97.2 mv=12000 \
  | CLAMP tgt=CLEAR act=19.8 err=0.2 done=1
```

`WINCH tgt` only means anything in `HEIGHT_MOVE`; elsewhere it is whatever the
last `raise_to()` asked for. `WINCH mv` is in the `Lift` class's convention
(positive = up) so it agrees in sign with the `err` beside it, and it is what
`Lift::drive()` was asked for rather than what survived the PTO check — so
`mv != 0` with `own=0` is a command the motors never saw. `ANG set` is the
ramped setpoint the PID tracks, `ANG tgt` the mode's target — a long gap
between `tgt` and `act` means a claw swing is being blocked. `tower=` is the
score macro's tower sensor and `loader=` the matchload detection sensor; 9999
means nothing in view. Logs from before the loader sensor was added have no
`loader=` field, and their `tower=` did both jobs.

The claw was a roller for logs from 2026-09-25 and 09-26; those end the line
with `| ROLLER on=1 mv=12000 [rpm=-35 ma=1840 w=4.6] [cyc=1]` instead of
`CLAMP`. From 2026-09-27 the clamp is back, with the targets `CLOSED`,
`CLEAR`, `MATCHLOAD` and `OPEN`. It is now a piston, so the section reads
`| CLAMP tgt=CLOSED pis=1 done=1`: `pis=1` while the solenoid is closed
(`CLOSED` and `CLEAR` close it, `MATCHLOAD` and `OPEN` open it) and `done=1`
once `CLAMP_ACTUATE_MS` has passed since it switched. Motor-clamp logs carry
`act=`/`err=` in deg in place of `pis=`. A mode change into or out of `LOADING` moves the clamp first (`Lift claw
CLEAR at .. -> swinging`) and only then swings the angler, so `ANG tgt` holds
still until that event.

The lift's prose lines (`Lift matchload macro: ...`, `Lift clamp -> CLEAR
(MATCHLOAD)`) are collected as events and mark the cycle boundaries.

`visualize.py lift <log>` draws a per-boot dashboard: target-vs-actual plus
commanded voltage for each actuator, a state/PTO/tower context row, and event
markers. `--run N` selects the Nth boot that contains lift samples (`all` for
every one). `test_lift.py` round-trips the log format and asserts lift
invariants over any log carrying lift data — finite values, `err ==
target-actual`, commands within ±12000 mV, the `MAX_LOWER_MV` lowering cap, no
upward drive at the height cap, no winch command while the PTO is not ours, and
**no full-voltage stretch without motion** (the boot-0161 stall). Logs without
lift data are skipped. Parsing and plots live in `log_analysis/lift_parser.py`
and `log_analysis/lift_plots.py`.

### Roller mech stream

`RollerMech::log_debug_state` (`src/hardware/roller_mech.cpp`) emits one
`ROLLERDBG` line per 10 ms tick while the shared PTO motors run in INTAKE
mode, whether a roller cycle or the intake is driving them. The roller mech is
geared onto the intake, so both cases turn the same motors and roller:

```
ROLLERDBG src=ROLLER state=SEEKING cmd=12000 \
  | PTO rpm=180,179,181 mv=11800,11750,11820 ma=900,950,880 \
  | AXLE pos=312.5 dps=455
```

`src` is who drives the motors (`ROLLER` or `INTAKE`), `cmd` the last PTO
command. `PTO` is what the three motors measure: rpm, applied mV and mA. `AXLE`
is the roller rotation sensor: travel in deg and its own deg/s reading.

`test_roller.py` round-trips the format and, on logs that carry the stream,
checks that a full-voltage roller cycle actually shows near-full voltage at the
motors, and that it reaches at least 85% of the intake's steady motor speed.
Both compare ticks at least 200 ms into a continuous run, so spin-up does not
count. These checks were added after roller cycles were seen spinning slower
than intaking; the cause was the idle intake braking the motors every tick.
Parsing and the `steady_speeds()` metric live in
`log_analysis/roller_parser.py`.

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

`logs/splitter.py` (splits a raw multi-boot dump into per-boot session
files) is kept for old dumps only; the firmware now writes one file per boot
and `import_logs.py` splits legacy dumps on import. The parser handles
multi-session files natively either way.

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
