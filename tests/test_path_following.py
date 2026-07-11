"""Path-following health checks over real robot logs in tests/logs/.

Each test encodes an invariant a healthy path follower should satisfy.
A FAILURE is evidence of a specific weakness in the control stack; the
assertion message names the code location responsible.

Run:  pytest tests/test_path_following.py -v
"""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import pytest

from log_analysis import metrics
from log_analysis.parser import parse_log, trajectory_runs

LOG_DIR = Path(__file__).parent / "logs"
LOG_FILES = ["square.txt", "init.txt", "pikes.log", "erie.txt",
             "state.txt", "league_night_2.txt", "default.log",
             "initial_tuning.txt"]

_cache: dict[str, list] = {}


def get_runs(name: str):
    if name not in _cache:
        path = LOG_DIR / name
        if not path.exists():
            pytest.skip(f"{name} not present")
        _cache[name] = trajectory_runs(parse_log(path))
    runs = _cache[name]
    if not runs:
        pytest.skip(f"{name}: no trajectory runs with control ticks")
    return runs


@pytest.fixture(params=LOG_FILES, ids=lambda n: n)
def log_runs(request):
    return request.param, get_runs(request.param)


# ------------------------------------------------------------------ NaN

def test_no_nan_in_control_pipeline(log_runs):
    """NaNs must never reach the controller.

    Trajectory::loadFromFile divides by (tp.time - past_time) with no
    zero guard, so the first waypoint (t=0) gets NaN accelerations, which
    flow through Robot::processTrajectory into
    DrivetrainController::calculateFeedforward and poison the voltage cast.
    """
    name, runs = log_runs
    total: dict[str, int] = {}
    for run in runs:
        for field_name, count in metrics.nan_report(run).items():
            total[field_name] = total.get(field_name, 0) + count
    assert not total, (
        f"{name}: NaN/inf values reached the control pipeline: {total}. "
        "Root cause: unguarded division in Trajectory::loadFromFile "
        "(src/navigation/trajectory.cpp:32-33)")


# ------------------------------------------------------------------ timing

def test_loop_period_holds_10ms(log_runs):
    """Control loop must hold Config::DT (10 ms) within 50% tolerance.

    Overruns break trajectory tracking because followTrajectory advances
    exactly one waypoint per iteration with no time-based indexing
    (src/hardware/robot.cpp:347-375): a late loop = robot permanently
    behind the trajectory time base.
    """
    name, runs = log_runs
    worst = 0.0
    total_overruns = 0
    total_periods = 0
    for run in runs:
        tm = metrics.loop_timing(run)
        if tm.get("n", 0) == 0:
            continue
        worst = max(worst, tm["worst_stall_ms"])
        total_overruns += tm["overrun_count"]
        total_periods += tm["n"]
    assert total_periods > 0, f"{name}: no timing data"
    frac = total_overruns / total_periods
    assert worst <= 15.0 and frac <= 0.01, (
        f"{name}: loop overruns {total_overruns}/{total_periods} "
        f"({100*frac:.2f}%), worst period {worst:.1f} ms (nominal 10 ms). "
        "The one-waypoint-per-iteration loop in Robot::followTrajectory "
        "cannot compensate for lost iterations.")


def test_no_mid_run_odometry_stalls(log_runs):
    """Odometry::update must never see deltaTime > 100 ms mid-run.

    When it does, it silently substitutes 10 ms (src/navigation/odometry.cpp:40-44),
    which multiplies measured wheel velocity by real_dt/10ms and corrupts
    the velocity feedback for that tick.
    """
    name, runs = log_runs
    stalls = 0
    worst = 0.0
    for run in runs:
        ws = metrics.warning_summary(run)
        stalls += ws["mid_run_stall_count"]
        worst = max(worst, ws["worst_stall_s"])
    assert stalls == 0, (
        f"{name}: {stalls} mid-run 'Invalid deltaTime' warnings "
        f"(worst {worst*1000:.0f} ms). Odometry velocity was scaled by up to "
        f"{worst/0.010:.0f}x on those ticks.")


# ------------------------------------------------------------------ tracking

def test_cross_track_error_bounded(log_runs):
    """RMS cross-track error should stay under 1 inch, max under 4 inches."""
    name, runs = log_runs
    failures = []
    for i, run in enumerate(runs):
        te = metrics.tracking_errors(run)
        ct = te["cross_track"]
        if ct.get("n", 0) == 0:
            continue
        if ct["rms"] > 1.0 or ct["max"] > 4.0:
            failures.append(f"run{i}: rms={ct['rms']:.2f}in max={ct['max']:.2f}in")
    assert not failures, f"{name}: cross-track error out of bounds: {failures}"


def test_final_pose_error_bounded(log_runs):
    """Position error at the last waypoint should be < 2 in and < 0.15 rad."""
    name, runs = log_runs
    failures = []
    for i, run in enumerate(runs):
        te = metrics.tracking_errors(run)
        fe, fh = te["final_position_error_in"], te["final_heading_error_rad"]
        if math.isnan(fe):
            continue
        if fe > 2.0 or fh > 0.15:
            failures.append(f"run{i}: pos={fe:.2f}in heading={fh:.3f}rad")
    assert not failures, f"{name}: final pose error too large: {failures}"


def test_heading_error_bounded(log_runs):
    """Heading error should stay under ~11 deg (0.2 rad) at p95.

    Heading is the weakest estimate: Odometry::update re-tares the IMU
    every 10 ms (src/navigation/odometry.cpp:145) and integrates deltas,
    discarding the IMU's absolute reference and accumulating quantization
    drift without bound.
    """
    name, runs = log_runs
    failures = []
    for i, run in enumerate(runs):
        te = metrics.tracking_errors(run)
        h = te["heading"]
        if h.get("n", 0) == 0:
            continue
        if h["p95"] > 0.20:
            failures.append(f"run{i}: p95={h['p95']:.3f}rad max={h['max']:.3f}rad")
    assert not failures, f"{name}: heading tracking error too large: {failures}"


# ------------------------------------------------------------------ velocity

def test_wheel_velocity_tracking(log_runs):
    """Measured wheel velocity should track setpoint within 6 in/s RMS."""
    name, runs = log_runs
    failures = []
    for i, run in enumerate(runs):
        vt = metrics.velocity_tracking(run)
        for side in ("left_abs", "right_abs"):
            st = vt[side]
            if st.get("n", 0) == 0:
                continue
            if st["rms"] > 6.0:
                failures.append(f"run{i} {side}: rms={st['rms']:.1f}in/s")
    assert not failures, (
        f"{name}: wheel velocity tracking poor: {failures}. "
        "DrivetrainController has kI=0/kD=0 and feedforward gains that may "
        "not match the drivetrain.")


def test_no_impossible_measured_velocities(log_runs):
    """Measured wheel velocity must never exceed the drivetrain's physical max.

    Spikes indicate the Odometry deltaTime fallback dividing a long
    encoder delta by the nominal 10 ms.
    """
    name, runs = log_runs
    total = 0
    worst = 0.0
    for run in runs:
        vs = metrics.velocity_spikes(run)
        total += vs["count"]
        worst = max(worst, vs["worst"])
    assert total == 0, (
        f"{name}: {total} physically impossible measured velocities "
        f"(worst {worst:.0f} in/s vs ~57 in/s max). These feed straight into "
        "the drivetrain PID as feedback.")


# ------------------------------------------------------------------ actuation

def test_voltage_saturation_low(log_runs):
    """Motors should saturate (+/-127) on < 15% of ticks; more means the
    trajectory or RAMSETE demands more than the drivetrain can deliver and
    tracking degrades open-loop."""
    name, runs = log_runs
    failures = []
    for i, run in enumerate(runs):
        st = metrics.saturation(run)
        if st.get("n", 0) < 50:
            continue
        if st["left_fraction"] > 0.15 or st["right_fraction"] > 0.15:
            failures.append(
                f"run{i}: L={100*st['left_fraction']:.0f}% R={100*st['right_fraction']:.0f}%")
    assert not failures, f"{name}: heavy voltage saturation: {failures}"


def test_no_creep_when_stopped(log_runs):
    """When the trajectory commands ~0 velocity the voltage must be 0.

    Robot::processTrajectory replaces voltage==0 with m_min_voltage
    (src/hardware/robot.cpp:198-205), so the robot creeps during
    intentional stops.
    """
    name, runs = log_runs
    total_creep = 0
    for run in runs:
        mv = metrics.min_voltage_substitution(run)
        total_creep += mv["creep_count"]
    assert total_creep == 0, (
        f"{name}: {total_creep} ticks with ~0 velocity setpoint but nonzero "
        "voltage (min-voltage substitution / PID bias).")


# ------------------------------------------------------------------ estimator

def test_no_unexplained_pose_jumps(log_runs):
    """Pose must be continuous except at commanded distance-sensor resets."""
    name, runs = log_runs
    total = 0
    examples = []
    for i, run in enumerate(runs):
        pj = metrics.pose_jumps(run)
        total += pj["unexplained_jump_count"]
        examples += [f"run{i}@t={t:.2f}s d={d:.1f}in" for t, d in pj["unexplained_jumps"][:3]]
    assert total == 0, (
        f"{name}: {total} pose discontinuities >2in not near a DISTANCE RESET "
        f"(e.g. {examples[:5]})")


# ------------------------------------------------------------------ kinematics

def test_effective_track_width_consistent(log_runs):
    """Measured wheel differential should imply the configured track width
    (Config::WHEEL_BASE_WIDTH, config.hpp:13).

    Estimated by regressing (v_r - v_l) on IMU rotation rate per tick, using
    *measured* wheel velocities -- so this tests the kinematic constant alone
    and is unaffected by whether the drivetrain hit its setpoints. A real
    mismatch mis-scales every RamseteController::calculate_wheel_velocities
    conversion.

    Only estimates with r_squared >= 0.8 are trusted; a poorly-conditioned run
    (little turning) must not be allowed to indict the constant.

    Judged against the track width the log's own "Build config:" line says the
    firmware ran with -- NOT against today's config.hpp. Logs recorded before
    that line existed are skipped: the robot has been rebuilt since, so a
    mismatch would only be telling us the geometry changed, which is not a bug.
    Keep the Build config line in the firmware; it is what makes old logs
    interpretable.
    """
    name, runs = log_runs
    nominal = next((metrics.declared_track_width(r) for r in runs
                    if metrics.declared_track_width(r)), None)
    if nominal is None:
        pytest.skip(f"{name}: no 'Build config:' line — the constants this log "
                    "ran with are unknown, so config.hpp cannot be judged by it")
    estimates = []
    for run in runs:
        tw = metrics.effective_track_width(run)
        if tw.get("estimate_in") and tw.get("r_squared", 0) >= 0.8:
            estimates.append(tw["estimate_in"])
    if not estimates:
        pytest.skip(f"{name}: no run with a well-conditioned track-width fit")
    median = sorted(estimates)[len(estimates) // 2]
    assert abs(median - nominal) / nominal <= 0.10, (
        f"{name}: effective track width median {median:.1f} in "
        f"(runs: {[f'{e:.1f}' for e in sorted(estimates)]}) vs the "
        f"{nominal:.1f} in this build used. Config::WHEEL_BASE_WIDTH is wrong; "
        "angular velocity commands are systematically mis-scaled.")


def test_dead_reckoned_actual_wheels_match_goal(log_runs):
    """Integrating the MEASURED wheel velocities over the first 3 s should
    reproduce the goal path within 4 in RMS.

    Measured wheels + configured track width = the kinematic chain alone. This
    is the version of the check that genuinely tests kinematics: if the wheels
    actually turned this much and the robot still isn't where the goal says, the
    geometry constants are wrong. Compounding drift is why only the first 300
    ticks are integrated.

    Requires the log to declare its build config: this integration divides by a
    track width, so on a log from an unknown build it would just be measuring
    our guess at the constant rather than the robot.
    """
    name, runs = log_runs
    if not any(metrics.declared_track_width(r) for r in runs):
        pytest.skip(f"{name}: no 'Build config:' line — the track width this log "
                    "ran with is unknown, so dead reckoning cannot be attributed")
    failures = []
    for i, run in enumerate(runs):
        short = replace(run, ticks=run.ticks[:300])
        dr = metrics.dead_reckon(short, source="wheel_actuals")
        if not dr.get("n") or dr["error"].get("n", 0) < 100:
            continue
        if dr["error"]["rms"] > 4.0:
            failures.append(f"run{i}: rms={dr['error']['rms']:.1f}in "
                            f"end={dr['endpoint_drift_in']:.1f}in")
    assert not failures, (
        f"{name}: the path the wheels actually drove diverges from the goal "
        f"path: {failures}. Kinematic constants (track width, wheel diameter) "
        "are inconsistent with the trajectory generator.")


def test_commanded_wheel_path_is_delivered(log_runs):
    """Integrating the COMMANDED wheel velocities should also reproduce the
    goal path within 6 in RMS.

    Same integration as above but seeded with setpoints, so it fails when the
    drivetrain does not deliver what RAMSETE asked for -- saturation, feedforward
    error, a weak side. Read it *together* with the measured-wheel test:

      measured OK + commanded BAD  -> delivery problem (voltage/feedforward)
      measured BAD + commanded OK  -> kinematics problem (track width)
      both BAD                     -> could be either; check rotation_deficit

    The old suite had only this half and called it a kinematics test, which sent
    tuning after track width when the real fault was voltage saturation.

    Requires a declared build config for the same reason as the measured-wheel
    version: RAMSETE produced these setpoints by dividing by *its* track width,
    so replaying them through a different one proves nothing.
    """
    name, runs = log_runs
    if not any(metrics.declared_track_width(r) for r in runs):
        pytest.skip(f"{name}: no 'Build config:' line — the track width this log "
                    "ran with is unknown, so dead reckoning cannot be attributed")
    failures = []
    for i, run in enumerate(runs):
        short = replace(run, ticks=run.ticks[:300])
        dr = metrics.dead_reckon(short, source="wheel_setpoints")
        if not dr.get("n") or dr["error"].get("n", 0) < 100:
            continue
        if dr["error"]["rms"] > 6.0:
            failures.append(f"run{i}: rms={dr['error']['rms']:.1f}in "
                            f"end={dr['endpoint_drift_in']:.1f}in")
    assert not failures, (
        f"{name}: the wheels did not deliver the commanded path: {failures}. "
        "Compare with test_dead_reckoned_actual_wheels_match_goal to tell "
        "delivery failure apart from bad kinematics.")


# ------------------------------------------------------------------ rotation

def test_commanded_rotation_is_delivered(log_runs):
    """The wheels should deliver >= 90% of the commanded wheel differential.

    Rotation is driven by the *difference* of two large wheel speeds, so it is
    the first thing to break when a wheel cannot hit its setpoint: a few in/s of
    shortfall on the outside wheel is a large fraction of the differential. The
    result is systematic under-rotation -- the robot runs wide on every turn in
    the same direction, which shows up as a one-sided cross-track error.

    Asserted on the integrated wheel differential (actual vs commanded), which
    never divides by a track width and so is valid even for logs whose build
    constants are unknown. Where the log does declare its build config, the
    message also reports the resulting rotation in radians and what share of the
    gap is wheel-tracking error rather than bad geometry.
    """
    name, runs = log_runs
    failures = []
    for i, run in enumerate(runs):
        rd = metrics.rotation_deficit(run)
        if not rd.get("n") or abs(rd["commanded_differential_in"]) < 10.0:
            continue
        frac = rd["differential_delivered_fraction"]
        if frac < 0.90:
            detail = ""
            if metrics.declared_track_width(run):
                detail = (f"; rotation {rd['achieved_rotation_rad']:+.2f} of "
                          f"{rd['commanded_rotation_rad']:+.2f} rad, "
                          f"{100*rd['explained_fraction']:.0f}% of the gap is "
                          "wheel-velocity tracking error (not track width)")
            failures.append(
                f"run{i}: wheels delivered {100*frac:.0f}% of the commanded "
                f"turn differential{detail}")
    assert not failures, f"{name}: under-rotation: {failures}"


def test_cross_track_error_not_one_sided(log_runs):
    """Cross-track error must not sit consistently on one side of the path.

    Magnitude alone (test_cross_track_error_bounded) cannot tell noise from
    bias. If |mean| is most of the RMS, the robot is always on the same side --
    a systematic kinematic or delivery fault, not tracking noise. On a route
    that mostly turns one way, that is the signature of under-rotation.
    """
    name, runs = log_runs
    failures = []
    for i, run in enumerate(runs):
        te = metrics.tracking_errors(run)
        ct = te["cross_track"]
        if ct.get("n", 0) < 100:
            continue
        bias = te["cross_track_bias_in"]
        if math.isnan(bias):
            continue
        # bias dominating RMS => one-sided. Allow small biases outright.
        if abs(bias) > 0.75 and abs(bias) / ct["rms"] > 0.70:
            failures.append(f"run{i}: bias={bias:+.2f}in of rms={ct['rms']:.2f}in "
                            f"({100*abs(bias)/ct['rms']:.0f}% one-sided)")
    assert not failures, (
        f"{name}: cross-track error is systematically one-sided: {failures}. "
        "The robot is always off the path in the same direction — check "
        "test_commanded_rotation_is_delivered for under-rotation.")


def test_no_unintended_reverse_driving(log_runs):
    """The robot should not move opposite its commanded direction on more
    than 2% of moving ticks (wheel slip, collisions, or sign errors)."""
    name, runs = log_runs
    failures = []
    for i, run in enumerate(runs):
        rv = metrics.reverse_driving(run)
        if rv["n"] < 100:
            continue
        if rv["unintended_fraction"] > 0.02:
            failures.append(f"run{i}: {100*rv['unintended_fraction']:.1f}% "
                            f"({rv['unintended_count']}/{rv['n']})")
    assert not failures, f"{name}: unintended reverse driving: {failures}"


def test_encoder_consistency(log_runs):
    """Same-side motor encoders should agree; a p95 spread above ~12 ticks
    per 10 ms means one motor slips or reads wrong, and the odometry average
    (Odometry::getAveragePosition) silently absorbs the error."""
    name, runs = log_runs
    checked = False
    failures = []
    for i, run in enumerate(runs):
        ed = metrics.encoder_disagreement(run)
        for side in ("left", "right"):
            st = ed[side]
            if st.get("n", 0) < 100:
                continue
            checked = True
            if st["p95"] > 12.0:
                failures.append(f"run{i} {side}: p95={st['p95']:.0f} max={st['max']:.0f} ticks")
    if not checked:
        pytest.skip(f"{name}: no Left/Right Positions lines in this log")
    assert not failures, f"{name}: same-side encoder disagreement: {failures}"


def test_distance_integral_drift_bounded(log_runs):
    """Net delivered distance should stay within 12% of commanded distance
    per side (∫actual dt vs ∫setpoint dt). Persistent drift means the
    feedforward (kS/kV in DrivetrainController) is mis-calibrated with no
    integral term (kI=0) to absorb it."""
    name, runs = log_runs
    failures = []
    for i, run in enumerate(runs):
        di = metrics.distance_integral_drift(run)
        if not di.get("n"):
            continue
        for side in ("left", "right"):
            travel = di[f"{side}_travel_in"]
            if travel < 100:
                continue
            rel = abs(di[f"{side}_drift_in"]) / travel
            if rel > 0.12:
                failures.append(f"run{i} {side}: {100*rel:.0f}% "
                                f"({di[f'{side}_drift_in']:+.1f}in of {travel:.0f}in)")
    assert not failures, f"{name}: cumulative distance drift: {failures}"


def test_mp_ramsete_agreement(log_runs):
    """RAMSETE output should stay close to the MP reference when tracking is
    healthy (rms divergence < 2 in/s linear, < 0.5 rad/s angular). Sustained
    divergence = the controller constantly fighting the pose estimate.
    Requires MP/Ramsete Output lines (square/init-style logs)."""
    name, runs = log_runs
    checked = False
    failures = []
    for i, run in enumerate(runs):
        d = metrics.mp_ramsete_divergence(run)
        if d["linear"].get("n", 0) < 100:
            continue
        checked = True
        if d["linear"]["rms"] > 2.0 or d["angular"]["rms"] > 0.5:
            failures.append(f"run{i}: v_rms={d['linear']['rms']:.2f}in/s "
                            f"w_rms={d['angular']['rms']:.2f}rad/s")
    if not checked:
        pytest.skip(f"{name}: no MP/Ramsete Output lines in this log")
    assert not failures, f"{name}: RAMSETE diverges from MP reference: {failures}"


# ------------------------------------------------- feedforward model / gains

def test_turning_gain_matches_straight_gain(log_runs):
    """Delivered/commanded wheel speed during in-place turns should be
    within 15% of the straight-line value.

    A larger gap is turning scrub the shared feedforward cannot represent:
    kS/kV/kA (config.hpp:17-19) are fitted to straight driving and applied
    per wheel, so turns are systematically under-driven even with P feedback
    helping. Fix: separate angular feedforward constants superposed in wheel
    space, and/or heading-rate feedback from the IMU gyro.
    """
    name, runs = log_runs
    mr = metrics.motion_regimes(runs)
    if mr["turn"]["n"] < 100 or mr["straight"]["n"] < 200:
        pytest.skip(f"{name}: not enough in-place-turn data")
    ratio = mr["turn_to_straight_ratio"]
    sd_t = mr["turn"].get("heading_err_sd_rad")
    sd_s = mr["straight"].get("heading_err_sd_rad")
    assert ratio >= 0.85, (
        f"{name}: wheel gain {mr['turn']['gain']:.2f} turning in place vs "
        f"{mr['straight']['gain']:.2f} straight (ratio {ratio:.2f}) — the "
        f"drivetrain delivers only {100 * mr['turn']['gain']:.0f}% of the "
        "commanded wheel speed while turning. Heading-error spread: "
        f"{math.degrees(sd_t) if sd_t else float('nan'):.1f} deg turning vs "
        f"{math.degrees(sd_s) if sd_s else float('nan'):.1f} deg straight.")


def test_steady_state_velocity_gain_near_unity(log_runs):
    """On settled, feasible cruise stretches, measured wheel speed should
    match the setpoint within 3%.

    The residual after feedback settles isolates feedforward calibration:
    gain < 0.97 means Config::KV under-delivers and kP carries the
    difference (with kI = 0 nothing removes it). Raise kV by ~1/gain.
    """
    name, runs = log_runs
    ss = metrics.steady_state_gain(runs)
    if ss["left"]["n"] < 100 and ss["right"]["n"] < 100:
        pytest.skip(f"{name}: no settled cruise stretches")
    failures = []
    for side in ("left", "right"):
        st = ss[side]
        if st["n"] >= 100 and not 0.97 <= st["gain"] <= 1.03:
            failures.append(f"{side}: gain={st['gain']:.3f} (n={st['n']})")
    assert not failures, (
        f"{name}: steady-state velocity gain off unity: {failures}. "
        "KV (config.hpp:18) is mis-calibrated for the drivetrain.")


def test_drivetrain_sides_symmetric(log_runs):
    """Left and right steady-state gains should agree within 3%.

    A larger gap is mechanical (failing motor, friction), not tuning — the
    state-competition logs showed the right side at 0.92 vs left 0.97 all
    event with nobody noticing. Worth a pre-match open-loop health check.
    """
    name, runs = log_runs
    ss = metrics.steady_state_gain(runs)
    # 25, not 100: a short (~3 s) run yields only a few dozen settled ticks on
    # the inside wheel, so a threshold of 100 skipped this check entirely on
    # exactly the logs where a weak side matters. A median over 25 is stable.
    if ss["left"]["n"] < 25 or ss["right"]["n"] < 25:
        pytest.skip(f"{name}: not enough settled data on both sides")
    assert ss["asymmetry"] <= 0.03, (
        f"{name}: side asymmetry {100 * ss['asymmetry']:.1f}% "
        f"(L={ss['left']['gain']:.3f} n={ss['left']['n']}, "
        f"R={ss['right']['gain']:.3f} n={ss['right']['n']}).")


def test_wheel_setpoints_feasible(log_runs):
    """Commanded wheel speeds must stay within Config::MAX_WHEEL_VELOCITY -- what
    the drivetrain can actually hold under load (~85% of free speed).

    The bar is deliberately below free speed. A setpoint the motors can only reach
    by pinning the voltage at ±127 leaves the feedback term zero authority exactly
    when the robot is off-path and needs it, and a saturated wheel is open-loop.
    (The old threshold was a hardcoded 74 in/s -- above this drivetrain's no-load
    ceiling -- so nothing could ever fail it.)

    RamseteController::calculate_wheel_velocities now desaturates to this budget,
    so a log recorded after that change should not be able to fail this test. If it
    does, either the desaturation is not running (check the maxWheelVel field in the
    log's "Build config:" line) or the budget is set above what the motors deliver.
    Pre-desaturation logs fail here by design -- that is the bug being fixed.
    """
    name, runs = log_runs
    sf = metrics.setpoint_feasibility(runs)
    if sf["n"] < 200:
        pytest.skip(f"{name}: too little setpoint data")
    assert sf["infeasible_fraction"] <= 0.005, (
        f"{name}: {100 * sf['infeasible_fraction']:.1f}% of wheel setpoints "
        f"exceed the usable {sf['vmax_in_s']:.0f} in/s (worst "
        f"{sf['max_setpoint_in_s']:.0f} in/s; free speed "
        f"{sf['free_speed_in_s']:.0f} in/s). Wheel-space desaturation in "
        "RamseteController::calculate_wheel_velocities should make this impossible.")


def test_saturation_cost_bounded(log_runs):
    """Mean |velocity error| on saturated ticks should stay under 8 in/s.

    Saturated ticks are effectively open-loop; a large gap vs unsaturated
    ticks means the controller regularly demands more than the drivetrain
    can deliver (see test_wheel_setpoints_feasible) instead of saturating
    only during brief transients.
    """
    name, runs = log_runs
    sc = metrics.saturation_cost(runs)
    if sc["saturated"].get("n", 0) < 30:
        pytest.skip(f"{name}: too few saturated ticks to compare")
    assert sc["saturated"]["mean"] <= 8.0, (
        f"{name}: mean |vel err| {sc['saturated']['mean']:.1f} in/s while "
        f"saturated vs {sc['unsaturated'].get('mean', float('nan')):.1f} "
        f"unsaturated ({sc['mean_ratio']:.1f}x).")


def test_distance_resets_are_small_corrections(log_runs):
    """A distance-sensor reset should be a small correction (< 6 in).

    Large jumps mean odometry had already drifted badly, and the jump
    itself destabilizes RAMSETE (sudden huge error -> aggressive response).
    """
    name, runs = log_runs
    worst = 0.0
    big = []
    for i, run in enumerate(runs):
        pj = metrics.pose_jumps(run)
        if pj["reset_jump_max_in"] > worst:
            worst = pj["reset_jump_max_in"]
        big += [f"run{i}@t={t:.2f}s d={d:.1f}in" for t, d in pj["reset_jumps"] if d > 6.0]
    assert not big, (
        f"{name}: distance resets moved the pose estimate by up to "
        f"{worst:.1f} in: {big[:5]}. Odometry drift between resets is large.")
