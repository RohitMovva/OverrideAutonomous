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
from log_analysis.robot_config import LATERAL_WHEEL_OFFSET, VERTICAL_WHEEL_OFFSET
from log_analysis.parser import parse_log, trajectory_runs

LOG_DIR = Path(__file__).parent / "logs"
LOG_FILES = ["09122026-3.txt", "square.txt", "init.txt", "pikes.log", "erie.txt",
             "state.txt", "league_night_2.txt", "default.log",
             "initial_tuning.txt", "07132026.txt", "07132026-2.txt", "07132026-3.txt", "07132026-4.txt", "07132026-5.txt", "07132026-6.txt", "07132026-7.txt", "07132026-8.txt"]
# Logs from the path-following test suite (VexAutonomousPlanner/suite/manifest.yaml) are
# copied in as suite_<route>-<n>.txt and picked up automatically.
LOG_FILES += sorted(p.name for p in LOG_DIR.glob("suite_*.txt"))
# Boots that ran routes from the planner's traction_ab/ folder (the traction A/B and the
# tracking-wheel offset spins), picked up without copying them to suite_ names.
LOG_FILES += sorted(p.name for p in LOG_DIR.glob("[01]*.txt")
                    if b"trajectory 'traction_ab/" in p.read_bytes())

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

    Estimated by regressing (v_r - v_l) on IMU rotation rate per tick.
    This tests wheel/IMU agreement, which depends on geometry, encoder accuracy,
    slip, and sensor timing. A mismatch requires investigation before tuning.

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
        f"{nominal:.1f} in this build used. Wheel motion and IMU yaw disagree; "
        "check encoder agreement, slip, and sensor timing before calibrating "
        "Config::WHEEL_BASE_WIDTH.")


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
    expected effect under ideal kinematics is under-rotation. IMU yaw must be
    checked independently: slip or sensor errors can reverse that conclusion.

    Asserted on the integrated wheel differential (actual vs commanded), which
    never divides by a track width and so is valid even for logs whose build
    constants are unknown. Where the log does declare its build config, the
    message also compares the observed yaw gap with the wheel-tracking
    contribution. Opposite signs rule out wheel shortfall as the sole cause.
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
                          f"wheel-tracking contribution {rd['tracking_loss_rad']:+.2f} rad "
                          f"versus observed gap {rd['deficit_rad']:+.2f} rad "
                          "(opposite signs mean wheel shortfall alone cannot explain yaw)")
            failures.append(
                f"run{i}: wheels delivered {100*frac:.0f}% of the commanded "
                f"turn differential{detail}")
    assert not failures, f"{name}: wheel differential shortfall: {failures}"


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
    """Motors on one side are geared to the same wheels, so their encoders
    cannot legitimately travel different distances. Net divergence above 1% of
    that side's travel means one motor slips or reads wrong, and
    Odometry::getAveragePosition averages the bad reading into
    forwardDisplacement.

    Asserted on the *fraction* of travel, not raw ticks: 'Left/Right Positions'
    are absolute untared counts, so raw spread scales with route length and is
    not comparable between a 95 in straight and a 230 in curve route. Healthy
    sides measure 0.00-0.07% across the whole corpus; a bad one measures 2-7%.
    """
    name, runs = log_runs
    checked = False
    failures = []
    for i, run in enumerate(runs):
        ed = metrics.encoder_disagreement(run)
        for side in ("left", "right"):
            st = ed[side]
            # <2 live encoders is the expected shape once a motor has been cut
            # out of the average -- nothing to cross-check, so nothing to assert.
            if st.get("n", 0) < 100 or st.get("motors", 0) < 2:
                continue
            # Short/slow runs accumulate too little travel for the ratio to mean
            # anything (square.txt moves ~11 in total).
            if st["travel_in"] < 40.0:
                continue
            checked = True
            if st["slip_fraction"] > 0.01:
                failures.append(
                    f"run{i} {side}: {100*st['slip_fraction']:.1f}% of travel "
                    f"({st['net_divergence_ticks']:.0f} ticks = "
                    f"{st['net_divergence_in']:.1f} in over {st['travel_in']:.0f} in, "
                    f"per-motor net {[round(x) for x in st['per_motor_net_ticks']]})")
    if not checked:
        pytest.skip(f"{name}: no usable multi-motor Left/Right Positions lines")
    assert not failures, (
        f"{name}: same-side encoder divergence (motors on one side are geared "
        f"together and cannot really travel different distances): {failures}")


def test_encoder_columns_are_integral(log_runs):
    """Encoder counts are whole numbers. Fractional values in the logged
    'Left/Right Positions' columns are memory garbage, not readings.

    Odometry::update logs get_position_all()[0], [1] and [2] but the drivetrain
    has two motors per side (main.cpp), so [2] is an out-of-bounds read on a
    2-element vector. It prints 0.0 most ticks and occasionally whatever float
    was adjacent (-1984.052930, 25.331436). Log only the elements that exist --
    iterate the vector instead of indexing a fixed three.
    """
    name, runs = log_runs
    checked = False
    failures = []
    for i, run in enumerate(runs):
        ed = metrics.encoder_disagreement(run)
        for side in ("left", "right"):
            st = ed[side]
            if "corrupt_samples" not in st:
                continue
            checked = True
            if st["corrupt_samples"] > 0:
                failures.append(f"run{i} {side}: {st['corrupt_samples']} non-integral samples")
    if not checked:
        pytest.skip(f"{name}: no Left/Right Positions lines in this log")
    assert not failures, (
        f"{name}: non-integral encoder samples -- out-of-bounds read at "
        f"odometry.cpp:96-99: {failures}")


def test_tracking_wheels_read_every_tick(log_runs):
    """Both tracking wheels must be usable on every tick of a run.

    When one is not (sensor unplugged, PROS_ERR, or a delta no drivetrain can make),
    Odometry::update silently substitutes the drive encoders for the vertical wheel or
    "no sideslip" for the lateral one, and the pose quietly loses what the wheels were
    added for. The first tick of each outage also logs a WARNING naming the wheel.
    """
    name, runs = log_runs
    checked = False
    failures = []
    for i, run in enumerate(runs):
        tw = metrics.tracking_wheel_agreement(run)
        if not tw["ticks"]:
            continue
        checked = True
        if tw["vertical_fallback"] or tw["lateral_fallback"]:
            failures.append(f"run{i}: vertical {tw['vertical_fallback']}, lateral "
                            f"{tw['lateral_fallback']} of {tw['ticks']} ticks")
    if not checked:
        pytest.skip(f"{name}: no TRACKWHEEL lines (pre-tracking-wheel firmware)")
    assert not failures, (
        f"{name}: odometry fell back from the tracking wheels (check the rotation "
        f"sensor ports in main.cpp and the wiring): {failures}")


def test_vertical_tracking_wheel_direction(log_runs):
    """The vertical tracking wheel must count the same way as the drive.

    A reversed rotation sensor (wrong sign on its port in main.cpp) makes the pose run
    backwards while the robot drives forwards. Its per-tick delta then anti-correlates
    with the drive-encoder delta, which no amount of slip can cause.
    """
    name, runs = log_runs
    checked = False
    failures = []
    for i, run in enumerate(runs):
        tw = metrics.tracking_wheel_agreement(run)
        if tw.get("n", 0) < 100 or tw["wheel_travel_in"] < 10.0:
            continue
        checked = True
        if tw["direction_dot"] < 0:
            failures.append(f"run{i}: wheel net {tw['wheel_net_in']:+.1f} in vs "
                            f"drive net {tw['drive_net_in']:+.1f} in")
    if not checked:
        pytest.skip(f"{name}: no usable TRACKWHEEL lines")
    assert not failures, (
        f"{name}: vertical tracking wheel counts opposite to the drive -- negate "
        f"vertical_encoder's port in main.cpp: {failures}")


def test_drive_slip_against_tracking_wheel(log_runs):
    """Drive encoders should agree with the vertical tracking wheel within 5% of travel.

    The tracking wheel is unpowered and on the centre line, so it measures what the
    chassis travelled. A larger gap is drive-wheel slip, or a diameter that has not
    been calibrated: push-test Config::VERTICAL_TRACKING_WHEEL_DIAMETER (and check
    Config::WHEEL_DIAMETER) before reading it as slip. Odometry no longer integrates
    the drive encoders, so a failure here is a control (feedforward/velocity) concern,
    not a pose error.
    """
    name, runs = log_runs
    checked = False
    failures = []
    for i, run in enumerate(runs):
        tw = metrics.tracking_wheel_agreement(run)
        if tw.get("n", 0) < 100 or tw["wheel_travel_in"] < 40.0:
            continue
        checked = True
        if tw["slip_fraction"] > 0.05:
            failures.append(f"run{i}: {100*tw['slip_fraction']:.1f}% (drive "
                            f"{tw['drive_net_in']:+.1f} in vs wheel {tw['wheel_net_in']:+.1f} in "
                            f"over {tw['wheel_travel_in']:.0f} in)")
    if not checked:
        pytest.skip(f"{name}: no TRACKWHEEL runs with enough travel")
    assert not failures, f"{name}: drive encoders vs vertical tracking wheel: {failures}"


def test_pose_follows_vertical_wheel(log_runs):
    """Odometry's forward travel should match what the vertical wheel rolled within 5%.

    The wheel's absolute reading counts every inch it rolled, including deltas
    Odometry::update rejected as implausible. When the gate drops real travel the pose
    falls behind, RAMSETE chases the goal, and the robot overshoots on the field: boot
    989 integrated 68.8 in of 114.9 in rolled and drove 116 in of a 96 in route. Check
    the tracking-wheel gate (Config::TRACKING_WHEEL_MAX_STALE_TICKS, odometry.cpp) and
    the rotation sensors' data rate (main.cpp initialize).
    """
    name, runs = log_runs
    checked = False
    failures = []
    for i, run in enumerate(runs):
        tw = metrics.tracking_wheel_agreement(run)
        wheel = tw.get("wheel_abs_net_in", 0.0)
        if abs(wheel) < 40.0:
            continue
        checked = True
        gap = abs(tw["pose_forward_in"] - wheel) / abs(wheel)
        if gap > 0.05:
            failures.append(f"run{i}: pose {tw['pose_forward_in']:+.1f} in vs wheel "
                            f"{wheel:+.1f} in ({100*gap:.0f}%)")
    if not checked:
        pytest.skip(f"{name}: no TRACKWHEEL runs with 40 in of vertical travel")
    assert not failures, f"{name}: odometry lost vertical-wheel travel: {failures}"


def test_tracking_wheel_offsets_match_config(log_runs):
    """Spins must show the tracking wheels where config.hpp says they sit, within 0.25 in.

    In a pure spin each wheel rolls only by its offset from the pivot times the angle
    turned (metrics.tracking_wheel_offsets), so the spin routes (suite spin_w*) measure
    Config::LATERAL_WHEEL_OFFSET and VERTICAL_WHEEL_OFFSET directly. A mismatch makes
    every turn leak into the pose as fake translation. The pivot is where the drive
    actually turns about, so the offsets move when the traction wheels do: re-measure
    after a traction change, and move the distance-sensor offsets in main.cpp with them.
    """
    name, runs = log_runs
    checked = False
    failures = []
    for i, run in enumerate(runs):
        off = metrics.tracking_wheel_offsets(run)
        if "lateral_offset_in" not in off or off["radians"] < 4 * math.pi:
            continue
        checked = True
        for key, declared, const in (("lateral_offset_in", LATERAL_WHEEL_OFFSET, "LATERAL_WHEEL_OFFSET"),
                                     ("vertical_offset_in", VERTICAL_WHEEL_OFFSET, "VERTICAL_WHEEL_OFFSET")):
            if abs(off[key] - declared) > 0.25:
                failures.append(f"run{i}: measured {const} {off[key]:+.2f} in vs config {declared:+.2f}")
    if not checked:
        pytest.skip(f"{name}: no tracking-wheel spin with 2 revolutions")
    assert not failures, f"{name}: tracking-wheel offsets (include/config.hpp): {failures}"


def test_spin_pivot_same_both_ways(log_runs):
    """The offsets fitted from CCW and CW spins should agree within 0.3 in.

    If they do not, the robot is not turning about a fixed point: scrub walks the pivot
    one way per direction, and no single offset in config.hpp can be right for both.
    Compare double vs single traction on this (tests/traction_ab.py).
    """
    name, runs = log_runs
    checked = False
    failures = []
    for i, run in enumerate(runs):
        off = metrics.tracking_wheel_offsets(run)
        if "lateral_ccw_in" not in off or off["radians"] < 4 * math.pi:
            continue
        checked = True
        for axis in ("lateral", "vertical"):
            gap = off[f"{axis}_ccw_in"] - off[f"{axis}_cw_in"]
            if abs(gap) > 0.3:
                failures.append(f"run{i} {axis}: CCW {off[f'{axis}_ccw_in']:+.2f} vs CW "
                                f"{off[f'{axis}_cw_in']:+.2f} in")
    if not checked:
        pytest.skip(f"{name}: no tracking-wheel spin with 2 revolutions")
    assert not failures, f"{name}: spin pivot differs by direction: {failures}"


def test_spin_commands_no_creep(log_runs):
    """RAMSETE should command no forward speed while the plan spins in place.

    It does when the pose believes the robot moves sideways during the turn, which a
    wrong Config::LATERAL_WHEEL_OFFSET produces (metrics.spin_creep): the robot then
    walks forward or back through every spin, and through every turn on a route.
    """
    name, runs = log_runs
    checked = False
    failures = []
    for i, run in enumerate(runs):
        cr = metrics.spin_creep(run)
        if not cr:
            continue
        checked = True
        if abs(cr["creep_in_per_rad"]) > 0.2:
            failures.append(f"run{i}: {cr['creep_in_per_rad']:+.2f} in per radian")
    if not checked:
        pytest.skip(f"{name}: no spin in place")
    assert not failures, (f"{name}: RAMSETE drives during spins -- check LATERAL_WHEEL_OFFSET "
                          f"(include/config.hpp) against metrics.tracking_wheel_offsets: {failures}")


def test_spin_delivers_planned_rate(log_runs):
    """On the plateau of an unsaturated spin the robot should turn within 10% of plan.

    The turn feedforward (Config::KS_ANGULAR / KV_ANGULAR, DrivetrainController) has to
    pay the scrub; when it falls short the spin lags the plan and the follower starts
    the next segment behind in heading. spin_max is planned past the robot's limit on
    purpose and is skipped by its saturation.
    """
    name, runs = log_runs
    checked = False
    failures = []
    for i, run in enumerate(runs):
        sc = metrics.spin_scrub(run)
        if "delivered_ratio" not in sc or sc["plateau_n"] < 50 or sc["saturated_fraction"] > 0.3:
            continue
        checked = True
        if abs(sc["delivered_ratio"] - 1.0) > 0.10:
            failures.append(f"run{i}: {sc['rate_mean']:.2f} rad/s for {sc['plan_rate_mean']:.2f} planned "
                            f"(extra turn voltage {sc['scrub_v']:.1f})")
    if not checked:
        pytest.skip(f"{name}: no unsaturated spin plateau")
    assert not failures, f"{name}: spin rate vs plan: {failures}"


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


def _distread_logs() -> list[str]:
    """Imported logs carrying the per-tick DISTREAD line (firmware from 2026-09-30 on).

    Only <MMDDYYYY>-<boot>.txt names dated 2026-09-30 or later are opened, and only
    their first 256 KB: the line is logged from the first control tick.
    """
    out = []
    for p in sorted(LOG_DIR.glob("*-*.txt")):
        stem = p.stem.split("-")[0]
        if len(stem) != 8 or not stem.isdigit():
            continue
        if (stem[4:], stem[:2], stem[2:4]) < ("2026", "09", "30"):
            continue
        with open(p, "rb") as fh:
            if b"DISTREAD" in fh.read(256 * 1024):
                out.append(p.name)
    return out


@pytest.mark.parametrize("name", _distread_logs() or ["(no DISTREAD logs)"])
def test_offline_wall_math_matches_firmware(name):
    """metrics.wall_estimate must reproduce every correction the firmware logged.

    The walls plot and the stop report only mean anything if the offline geometry
    (sensor offsets parsed from main.cpp, wall position from config.hpp, the beam /
    wall snapping) is the firmware's. A mismatch means one side changed: re-sync
    wall_estimate with src/navigation/distance_reset_odometry.cpp, or the offsets.
    Needs the per-tick DISTREAD line to tell which sensor made each reset.
    """
    runs = get_runs(name)
    rows = [(i, a) for i, run in enumerate(runs) for a in metrics.reset_agreement(run)]
    if not rows:
        pytest.skip(f"{name}: no resets with DISTREAD data")
    bad = [f"run{i}@t={a['t'] % 60:.2f}s {a['sensor']} {a['axis']}: logged {a['logged_in']:+.2f} "
           f"offline {a['offline_in']:+.2f}" for i, a in rows if abs(a["diff_in"]) > 0.25]
    assert not bad, f"{name}: offline wall math disagrees with the firmware: {bad[:5]}"
