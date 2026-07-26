# Suggested Changes — OverrideAutonomous

Prioritized list from a full-stack review (firmware + VexAutonomousPlanner) on 2026-07-15.
Companion file: `VexAutonomousPlanner/SUGGESTED_CHANGES.md`.

Note: the smart-port overlaps in `main.cpp` (IMU vs. lift rotation on 20, lift motors vs. distance
sensor vs. side encoder on 5, drive vs. distance on 3) are known rebuild artifacts — re-verify port
assignments once the rebuild is complete, then delete this note.

---

## 1. Bug fixes (do first)

- [ ] **Min-voltage "creep" bug** (`src/hardware/robot.cpp:205-209`). When `m_min_voltage != 0`,
  the code assigns the *wheel velocity in in/s* (cast to int) directly as a ±127 motor command
  instead of clamping the computed voltage to the minimum. Your own test flags it
  (`test_no_creep_when_stopped`, `tests/test_path_following.py:224-238`).
- [ ] **Time-index the trajectory follower.** `followTrajectory` advances exactly one waypoint per
  loop iteration (`robot.cpp:369-396`), so any tick overrun (SD logging runs inside the 10 ms
  budget) permanently desyncs the robot from the trajectory clock. Index the waypoint by
  `elapsed_ms / Config::DT` instead — a slow tick then skips ahead rather than lagging forever.
  The loop-period test (`test_path_following.py:69-94`) exists precisely because of this fragility.
- [ ] **IMU calibration wait.** `initialize()` calls `imu_sensor.reset()` then `pros::delay(5)`
  before use (`main.cpp:75, 98`); calibration takes ~2 s. The baseline-reseed guard in odometry
  papers over it — block on `imu_sensor.is_calibrating()` instead.
- [ ] **`joystickCurve` division by zero** (`main.cpp:180`) — divides by `std::abs(x)` with no
  zero guard. Currently unused (opcontrol is raw tank), but a NaN landmine if ever adopted.
  Guard or delete.

## 2. Control quality

- [ ] **Add a small kI (with anti-windup) to the drivetrain velocity loop.** Currently pure
  P + feedforward (`KI=0, KD=0` in `config.hpp`), so any steady-state feedforward miscalibration
  goes uncorrected — which is exactly what `test_steady_state_velocity_gain_near_unity` and
  `test_distance_integral_drift_bounded` are probing for.
- [ ] **Move blocking SD logging out of the 10 ms control tick** (`robot.cpp:145-146, 178-179, 210`).
  Buffer log lines in memory and flush from a lower-priority task so a slow SD write can't blow
  the loop budget (compounds with the index-per-loop issue above).
- [ ] **Reconcile track width.** `Config::WHEEL_BASE_WIDTH = 13.5` with a comment admitting the
  effective value is ~15; the planner uses 13.5 too. After the rebuild, run
  `tuneTrackWidth()`, pick one number, and update both repos
  (`test_effective_track_width_consistent` guards this).
- [ ] **Document the unit conventions** (ideally in a `CLAUDE.md` / `UNITS.md`): trajectory files
  are inches + radians; RAMSETE internals are meters (via `RAMSETE_SCALE_FACTOR = 0.0254`,
  `config.hpp:80`); drivetrain kS/kV/kA are in ±127 "motor units" while the lift uses true
  millivolts (`move_voltage`). This map currently lives only in heads.

## 3. Planner↔firmware contract (coordinate with VexAutonomousPlanner)

- [ ] **Validate a versioned trajectory header at load.** Have the planner emit
  `# v2 track_width=13.5 wheel_diam=2.65 max_v=55 hash=<route-hash>` and extend
  `Trajectory::loadFromFile` (`src/navigation/trajectory.cpp:20-59`) to parse it and warn (or
  refuse) on constant mismatch. Turns silent wrong-constants runs into loud failures.
- [ ] **Shared action-slot spec.** ActionPoint slots (`actions[0..9]`) are defined only implicitly
  by `Robot::processAction` (`robot.cpp:222-308`) — intake state, distance resets, min-voltage
  hold, color sort, etc. Write the table down once and mirror constants in both repos so a
  planner-side reorder can't silently rewire autonomous behavior.
- [ ] **Log the route hash.** The `Build config:` log line (`robot.cpp:319-330`) already ties logs
  to firmware constants; add the trajectory file's hash so every recorded log traces to the exact
  route version that ran.

## 4. Dead code / hygiene

- [ ] **Delete or branch-park dead code:**
  - `MCL` (`src/navigation/mcl.cpp`, 315 lines) — fully written, never instantiated, sensor-update
    stubs at `:126, :135`. If it's a future plan, move it to a branch.
  - `Trajectory::loadFromVector` (commented out, `trajectory.cpp:63-130`) and the empty
    `routes.hpp`.
  - `arcade_drive` (~100 lines, `main.cpp:201`) — opcontrol uses raw tank.
  - Unused `calculate_global_error`/`calculate_local_error` (`ramsete_controller.cpp:12-37`) —
    `calculate()` re-implements the transform inline — and the computed-but-never-read
    `is_low_speed` (`:89`).
  - `goal_to_matchloader`/`low_goal` trajectories loaded but never followed; hardcoded
    `program_type = "autonomous"` / `skills=false` dead branches.
  - Root-level `test.py` (2-line throwaway) and `test.txt` (stale sample route).
- [ ] **Fix stale docs so the test suite stays trustworthy:**
  - `tests/README.md` references an `ISSUES.md` that doesn't exist — create it or drop the link.
  - Pytest docstrings describe already-fixed bugs: the odometry per-tick re-tare (fixed at
    `odometry.cpp:186-198`) and the unguarded accel finite-difference (now guarded,
    `trajectory.cpp:32-42`). Update the docstrings.
- [ ] **Untrack `compile_commands.json`** (88 KB, checked in despite being gitignored).

## 5. Capability ideas

- [ ] **Run the C++ unit tests in CI.** `tests/cpp/` already host-compiles the real controller
  sources with stubbed `pros::millis()` — a tiny GitHub Actions workflow running those four
  binaries plus the pytest invariant suite (against committed sample logs) would catch
  regressions before they reach the field.
- [ ] **Enable the lateral tracking wheel** — declared but disabled (`lateralPos = 0`,
  `odometry.cpp:98, 164`). With drive-encoder-only translation, lateral scrub during turns is
  invisible to odometry; worth revisiting after the rebuild if the hardware returns.
- [ ] **Post-run summary on the brain screen** — final cross-track/heading error and max loop
  overrun after each auton run, so you get instant field-side feedback without pulling the SD
  card into the pytest suite.
