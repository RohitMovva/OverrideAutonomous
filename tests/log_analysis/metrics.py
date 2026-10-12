"""Path-following quality metrics computed from parsed robot logs.

Each metric targets a specific suspected weakness in the control stack:

- tracking_error_stats   -> how well RAMSETE + drivetrain PID track the goal
- loop_timing_stats      -> control-loop period jitter/stalls (Robot::followTrajectory)
- velocity_tracking_stats-> wheel-velocity setpoint vs measured (DrivetrainController)
- nan_report             -> NaN/inf propagation (Trajectory::loadFromFile accel div-by-zero)
- saturation_stats       -> voltage clipping at +/-127
- pose_jump_stats        -> pose teleports (DistanceResetOdometry mid-path resets)
- velocity_spike_stats   -> impossible measured velocities (Odometry dt fallback)
- min_voltage_stats      -> zero-voltage -> m_min_voltage substitution (Robot::processTrajectory)
- motion_regimes         -> wheel gain split by straight/arc/in-place-turn
                            (shared vs angular feedforward constants)
- steady_state_gain      -> settled measured/commanded ratio per side
                            (kV calibration + left/right mechanical asymmetry)
- setpoint_feasibility   -> commanded wheel speeds beyond what the drivetrain
                            can deliver under load (route limits + desaturation)
- saturation_cost        -> velocity error on saturated vs unsaturated ticks
- effective_track_width  -> track width regressed from wheel differential vs IMU
- rotation_deficit       -> commanded vs achieved rotation, and how much of any
                            gap is wheel-velocity tracking error rather than a
                            wrong track width

Robot constants come from log_analysis.robot_config, which parses
include/config.hpp. Do not hardcode them here -- the previous copies (track
width 14.0, a 74 in/s speed ceiling) had drifted from the firmware and made the
suite both raise false alarms and miss real faults.

motion_regimes / steady_state_gain / setpoint_feasibility / saturation_cost
accept a single Run or a list of Runs; pass a whole file's runs to pool
sparse regimes (in-place turns are rare within any single run).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from statistics import mean, median, pstdev

from .parser import Run
from .robot_config import (
    DT_NOMINAL,
    FREE_WHEEL_SPEED,
    INCHES_PER_TICK,
    KS,
    KV,
    MAX_BEAM_ANGLE_DEG,
    RESET_SENSORS,
    TRACK_WIDTH,
    UNPHYSICAL_WHEEL_SPEED,
    USABLE_WHEEL_SPEED,
    WALL_POSITION,
)

# Retained as an alias: "the fastest a wheel can conceivably be measured going".
# Previously a hardcoded 74.0, which was well above this drivetrain's actual
# 64.8 in/s free speed and therefore never flagged anything.
MAX_WHEEL_SPEED = UNPHYSICAL_WHEEL_SPEED
VOLTAGE_LIMIT = 127

# Slack allowed when judging a wheel setpoint against the desaturation cap. The
# firmware clamps to MAX_WHEEL_VELOCITY exactly, but the log prints setpoints to
# 6 decimals, so a clamped value parses back marginally above the cap. Anything
# genuinely infeasible overshoots by whole in/s, far beyond this.
_SETPOINT_REL_TOL = 1e-5


def declared_track_width(run: Run) -> float | None:
    """The track width the firmware was actually built with for this run, from
    the log's "Build config:" line. None for logs recorded before that line
    existed.

    Always prefer this over the current config.hpp when judging a log. A log from
    a previous build ran with constants we no longer have any record of, so
    holding it against today's config.hpp tests nothing -- it just reports that
    the robot changed, which we already knew.
    """
    return run.build_config.get("wheelBase")


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _stats(xs: list[float]) -> dict:
    if not xs:
        return {"n": 0}
    return {
        "n": len(xs),
        "mean": mean(xs),
        "median": median(xs),
        "rms": math.sqrt(sum(x * x for x in xs) / len(xs)),
        "max": max(xs),
        "p95": sorted(xs)[int(0.95 * (len(xs) - 1))],
    }


# ---------------------------------------------------------------- tracking

def tracking_errors(run: Run) -> dict:
    """Cross-track / along-track / heading error of pose vs goal per tick."""
    cross, along, heading, euclid = [], [], [], []
    for tk in run.ticks:
        if tk.pose is None or tk.goal is None:
            continue
        px, py, pth = tk.pose
        gx, gy, gth = tk.goal
        if any(math.isnan(v) for v in (px, py, pth, gx, gy, gth)):
            continue
        ex, ey = gx - px, gy - py
        # errors in the robot frame: x = along-track, y = cross-track
        c, s = math.cos(pth), math.sin(pth)
        along.append(ex * c + ey * s)
        cross.append(-ex * s + ey * c)
        heading.append(_wrap(gth - pth))
        euclid.append(math.hypot(ex, ey))
    final_err = euclid[-1] if euclid else math.nan
    final_heading = abs(heading[-1]) if heading else math.nan
    return {
        "euclidean": _stats(euclid),
        "cross_track": _stats([abs(x) for x in cross]),
        "along_track": _stats([abs(x) for x in along]),
        "heading": _stats([abs(x) for x in heading]),
        # Signed means: a nonzero bias is systematic (positive along-track =
        # goal ahead of robot = feedforward under-delivery; positive
        # cross-track = goal to the robot's left).
        "along_track_bias_in": mean(along) if along else math.nan,
        "cross_track_bias_in": mean(cross) if cross else math.nan,
        "heading_bias_rad": mean(heading) if heading else math.nan,
        "final_position_error_in": final_err,
        "final_heading_error_rad": final_heading,
    }


# ---------------------------------------------------------------- timing

def loop_timing(run: Run) -> dict:
    """Control-loop period from consecutive Pose timestamps.

    The loop is supposed to run at a fixed 10 ms (Config::DT) via
    pros::Task::delay_until. Overruns mean the trajectory index (which
    advances exactly one waypoint per iteration, with no time-based lookup)
    falls behind the trajectory's own time base.
    """
    periods = []
    for a, b in zip(run.ticks, run.ticks[1:]):
        periods.append(b.t - a.t)
    if not periods:
        return {"n": 0}
    overruns = [p for p in periods if p > DT_NOMINAL * 1.5]
    stalls = [p for p in periods if p > 0.030]
    return {
        **_stats(periods),
        "overrun_count": len(overruns),
        "overrun_fraction": len(overruns) / len(periods),
        "stall_count": len(stalls),
        "worst_stall_ms": max(periods) * 1000,
        "cumulative_slip_ms": sum(p - DT_NOMINAL for p in periods if p > DT_NOMINAL) * 1000,
    }


# ---------------------------------------------------------------- velocity

def velocity_tracking(run: Run) -> dict:
    """Setpoint vs measured wheel velocity (input/output of DrivetrainController)."""
    err_l, err_r = [], []
    for tk in run.ticks:
        if tk.left_vel_setpoint is not None and tk.left_vel_actual is not None:
            if not (math.isnan(tk.left_vel_setpoint) or math.isnan(tk.left_vel_actual)):
                err_l.append(tk.left_vel_setpoint - tk.left_vel_actual)
        if tk.right_vel_setpoint is not None and tk.right_vel_actual is not None:
            if not (math.isnan(tk.right_vel_setpoint) or math.isnan(tk.right_vel_actual)):
                err_r.append(tk.right_vel_setpoint - tk.right_vel_actual)
    return {
        "left_abs": _stats([abs(e) for e in err_l]),
        "right_abs": _stats([abs(e) for e in err_r]),
    }


def velocity_lag_ticks(run: Run, max_lag: int = 20) -> int | None:
    """Estimate actuator/estimation latency: lag (in ticks) maximizing
    correlation between velocity setpoint and measured velocity."""
    sp = [tk.left_vel_setpoint for tk in run.ticks]
    ac = [tk.left_vel_actual for tk in run.ticks]
    pairs = [(s, a) for s, a in zip(sp, ac)
             if s is not None and a is not None
             and not math.isnan(s) and not math.isnan(a)]
    if len(pairs) < 50:
        return None
    sp = [p[0] for p in pairs]
    ac = [p[1] for p in pairs]

    def corr(lag: int) -> float:
        xs = sp[: len(sp) - lag]
        ys = ac[lag:]
        n = len(xs)
        mx, my = mean(xs), mean(ys)
        sx = math.sqrt(sum((x - mx) ** 2 for x in xs))
        sy = math.sqrt(sum((y - my) ** 2 for y in ys))
        if sx == 0 or sy == 0:
            return 0.0
        return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / (sx * sy)

    best = max(range(max_lag + 1), key=corr)
    return best


# ---------------------------------------------------------------- NaN

def nan_report(run: Run) -> dict:
    """Count NaN/inf values per field across all ticks."""
    counts: dict[str, int] = {}
    for tk in run.ticks:
        for name, v in tk.numeric_fields():
            if math.isnan(v) or math.isinf(v):
                counts[name] = counts.get(name, 0) + 1
    return counts


# ---------------------------------------------------------------- saturation

def saturation(run: Run) -> dict:
    n = sat_l = sat_r = 0
    for tk in run.ticks:
        if tk.voltage_left is None:
            continue
        n += 1
        if abs(tk.voltage_left) >= VOLTAGE_LIMIT:
            sat_l += 1
        if abs(tk.voltage_right) >= VOLTAGE_LIMIT:
            sat_r += 1
    if n == 0:
        return {"n": 0}
    return {"n": n, "left_fraction": sat_l / n, "right_fraction": sat_r / n}


# ---------------------------------------------------------------- pose jumps

def pose_jumps(run: Run, threshold_in: float = 2.0) -> dict:
    """Discontinuities in consecutive pose estimates.

    A robot physically moves < 0.75 in per 10 ms tick at max speed; larger
    jumps are estimator corrections. Jumps near a DISTANCE RESET are
    attributed to it; the rest are unexplained (odometry glitches).
    """
    reset_ticks = {r.tick_index for r in run.resets}
    jumps, reset_jumps = [], []
    for i, (a, b) in enumerate(zip(run.ticks, run.ticks[1:])):
        if a.pose is None or b.pose is None:
            continue
        d = math.hypot(b.pose[0] - a.pose[0], b.pose[1] - a.pose[1])
        if d >= threshold_in:
            near_reset = any(abs(i - rt) <= 2 for rt in reset_ticks)
            (reset_jumps if near_reset else jumps).append((run.ticks[i].t, d))
    return {
        "reset_jump_count": len(reset_jumps),
        "reset_jump_max_in": max((d for _, d in reset_jumps), default=0.0),
        "reset_jumps": reset_jumps,
        "unexplained_jump_count": len(jumps),
        "unexplained_jumps": jumps[:20],
    }


# ---------------------------------------------------------------- vel spikes

def velocity_spikes(run: Run, limit: float = MAX_WHEEL_SPEED) -> dict:
    """Measured wheel velocities that exceed what the drivetrain can do.

    Odometry::update() replaces any deltaTime outside (0, 100 ms] with the
    10 ms default while the encoder delta still spans the real elapsed time,
    which multiplies the measured velocity by (real_dt / 10 ms).
    """
    spikes = []
    for tk in run.ticks:
        for v in (tk.left_vel_actual, tk.right_vel_actual):
            if v is not None and not math.isnan(v) and abs(v) > limit:
                spikes.append((tk.t, v))
    return {"count": len(spikes), "worst": max((abs(v) for _, v in spikes), default=0.0),
            "examples": spikes[:10]}


# ---------------------------------------------------------------- min voltage

def min_voltage_substitution(run: Run) -> dict:
    """Ticks where commanded velocity ~ 0 but a nonzero voltage was applied.

    Robot::processTrajectory replaces an exactly-zero voltage with
    m_min_voltage, so 'stop' segments of the trajectory keep creeping.
    """
    n = 0
    creep = []
    for tk in run.ticks:
        if (tk.left_vel_setpoint is None or tk.voltage_left is None
                or math.isnan(tk.left_vel_setpoint)):
            continue
        n += 1
        if (abs(tk.left_vel_setpoint) < 0.05 and abs(tk.right_vel_setpoint) < 0.05
                and (tk.voltage_left != 0 or tk.voltage_right != 0)):
            creep.append((tk.t, tk.voltage_left, tk.voltage_right))
    return {"n": n, "creep_count": len(creep), "examples": creep[:10]}


# ---------------------------------------------------------------- warnings

# ------------------------------------------------- ported legacy analyses

def mp_ramsete_divergence(run: Run) -> dict:
    """|RAMSETE output - MP reference| for linear and angular velocity.

    (Ported from velocity_grapher.py.) Large sustained divergence means the
    feedback controller is fighting the motion profile — usually a pose
    estimate problem, not a profile problem. Only square/init-style logs
    contain these lines.
    """
    dv, dw = [], []
    for tk in run.ticks:
        if None in (tk.ramsete_v, tk.mp_v, tk.ramsete_w, tk.mp_w):
            continue
        if any(math.isnan(x) for x in (tk.ramsete_v, tk.mp_v, tk.ramsete_w, tk.mp_w)):
            continue
        dv.append(abs(tk.ramsete_v - tk.mp_v))
        dw.append(abs(tk.ramsete_w - tk.mp_w))
    return {"linear": _stats(dv), "angular": _stats(dw)}


def theta_tracking(run: Run) -> dict:
    """Signed and absolute heading tracking error in degrees.

    (Ported from theta_grapher.py.) A nonzero mean *signed* error is a bias
    (consistent under/overshoot in one direction); abs stats measure noise.
    """
    signed = []
    for tk in run.ticks:
        if tk.pose is None or tk.goal is None:
            continue
        e = _wrap(tk.goal[2] - tk.pose[2])
        if not math.isnan(e):
            signed.append(math.degrees(e))
    st = _stats([abs(x) for x in signed])
    if signed:
        st["mean_signed_deg"] = mean(signed)
    return st


def dead_reckon(run: Run, source: str = "wheel_setpoints",
                track_width: float | None = None) -> dict:
    """Integrate commanded/measured velocities into a path; compare to goals.

    track_width defaults to whatever the log's "Build config:" line says the
    firmware ran with, falling back to config.hpp. Pass an explicit value to
    sweep it.

    (Ported from goal_animator.py / velo_checker.py.) Sources:
      'mp'              — MP Output chassis velocities (square/init logs only)
      'wheel_setpoints' — commanded wheel velocities via diff-drive kinematics
      'wheel_actuals'   — measured wheel velocities via diff-drive kinematics

    Seeded from the first goal pose (as the legacy scripts did). Returns the
    simulated path, goal path, endpoint drift, and index-aligned error stats.

    Choose `source` deliberately -- the two answer different questions:

      'wheel_setpoints' integrates what the controller *asked for*, so it
      diverges from the goal whenever the drivetrain fails to deliver. It is a
      test of delivery (saturation, feedforward), NOT of kinematics.

      'wheel_actuals' integrates what the wheels *did*, so it isolates the
      kinematic chain (track width, wheel diameter) from drivetrain shortfall.

    Conflating them is why the old suite blamed track width for what was really
    voltage saturation: the setpoint-integrated path only lined up with the goal
    when track width was inflated to ~20 in, which "fixed" the error by adding a
    second, compensating error.
    """
    if track_width is None:
        track_width = declared_track_width(run) or TRACK_WIDTH
    ticks = [tk for tk in run.ticks if tk.goal is not None]
    if len(ticks) < 2:
        return {"n": 0}
    x, y, th = ticks[0].goal
    sim = [(x, y, th)]
    goals = [ticks[0].goal]
    errs = []
    for prev, tk in zip(ticks, ticks[1:]):
        dt = tk.t - prev.t
        if dt <= 0 or dt > 0.1:
            dt = DT_NOMINAL
        if source == "mp":
            v, w = prev.mp_v, prev.mp_w
        else:
            l = prev.left_vel_setpoint if source == "wheel_setpoints" else prev.left_vel_actual
            r = prev.right_vel_setpoint if source == "wheel_setpoints" else prev.right_vel_actual
            if l is None or r is None or math.isnan(l) or math.isnan(r):
                v = w = None
            else:
                v = (l + r) / 2.0
                w = (r - l) / track_width
        if v is None or w is None or math.isnan(v) or math.isnan(w):
            sim.append(sim[-1])
            goals.append(tk.goal)
            continue
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += w * dt
        sim.append((x, y, th))
        goals.append(tk.goal)
        errs.append(math.hypot(tk.goal[0] - x, tk.goal[1] - y))
    end_drift = math.hypot(goals[-1][0] - sim[-1][0], goals[-1][1] - sim[-1][1])
    return {"n": len(sim), "sim_path": sim, "goal_path": goals,
            "endpoint_drift_in": end_drift, "error": _stats(errs)}


def effective_track_width(run: Run, nominal: float = TRACK_WIDTH,
                          min_omega: float = 0.3) -> dict:
    """Estimate the real track width by regressing wheel differential on
    IMU rotation rate, per tick.

    Kinematics: (v_r - v_l) = W * omega. Pose theta is IMU-driven and is the
    trusted rotation. Regressing the measured wheel differential against the
    measured rotation rate through the origin gives W directly.

    This *used* to be computed as (ΔR_total - ΔL_total) / net_rotation over
    the whole run. That estimator is unusable on any route that reverses
    curvature: net rotation is a signed sum, so a route that turns right, then
    left, then right cancels most of it and the ratio blows up. On
    tests/logs/initial_tuning.txt it reported 16.9 in (vs a true 14.5 in),
    swinging between 13.1 and 19.3 in across 0.5 s windows -- a false alarm
    that pointed tuning at the wrong constant entirely. The per-tick
    regression is stable because every turning tick is an independent sample
    and sign cancellation cannot occur (both sides of the fit flip together).

    Ticks with |omega| < min_omega carry almost no information about W (the
    differential is in the noise) and are excluded. `r_squared` reports how
    much of the differential the fit actually explains -- treat an estimate
    with r_squared < 0.8 as untrustworthy.
    """
    ticks = [tk for tk in run.ticks if tk.pose is not None
             and tk.left_vel_actual is not None
             and tk.right_vel_actual is not None]
    if len(ticks) < 50:
        return {"n": 0, "estimate_in": None, "nominal_in": nominal}

    # (omega, differential) samples from consecutive ticks
    xs, ys = [], []
    total_abs_rot = 0.0
    net_rot = 0.0
    for a, b in zip(ticks, ticks[1:]):
        dt = b.t - a.t
        if not (0 < dt < 0.1):
            continue
        dth = _wrap(b.pose[2] - a.pose[2])
        total_abs_rot += abs(dth)
        net_rot += dth
        omega = dth / dt
        diff = a.right_vel_actual - a.left_vel_actual
        if math.isnan(omega) or math.isnan(diff):
            continue
        if abs(omega) < min_omega:
            continue
        xs.append(omega)
        ys.append(diff)

    if len(xs) < 30:
        return {"n": len(ticks), "turning_samples": len(xs),
                "net_rotation_rad": net_rot,
                "total_abs_rotation_rad": total_abs_rot,
                "estimate_in": None, "nominal_in": nominal}

    # least squares through the origin: W = sum(x*y) / sum(x*x)
    den = sum(x * x for x in xs)
    est = sum(x * y for x, y in zip(xs, ys)) / den
    ss_res = sum((y - est * x) ** 2 for x, y in zip(xs, ys))
    ss_tot = sum(y * y for y in ys)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0

    return {"n": len(ticks), "turning_samples": len(xs),
            "net_rotation_rad": net_rot,
            "total_abs_rotation_rad": total_abs_rot,
            "estimate_in": est, "nominal_in": nominal,
            "r_squared": r2,
            "ratio": est / nominal if nominal else math.nan}


def rotation_deficit(run: Run, track_width: float | None = None) -> dict:
    """Commanded vs achieved rotation, and how much of any gap is explained by
    the drivetrain failing to deliver its commanded wheel velocities.

    This is the metric that explains a one-sided cross-track error. Rotation is
    driven by the *difference* of two large wheel speeds, so it is far more
    fragile than forward motion: when the mean commanded differential is
    ~18 in/s against a ~33 in/s chassis speed, a mere 3.5 in/s shortfall on the
    outside wheel is a ~19% rotation error. The robot then under-rotates and
    runs wide on every turn in the same direction.

    `explained_fraction` near 1.0 means the whole rotation gap is wheel-velocity
    tracking error (fix the drivetrain / give it headroom), NOT a bad track
    width (fix the constant). Distinguishing those two is the entire point.

    Uses the track width the log was built with (falling back to config.hpp),
    since that is the constant RAMSETE actually divided by at record time.
    """
    if track_width is None:
        track_width = declared_track_width(run) or TRACK_WIDTH
    ticks = [tk for tk in run.ticks if tk.pose is not None
             and tk.left_vel_setpoint is not None
             and tk.right_vel_setpoint is not None
             and tk.left_vel_actual is not None
             and tk.right_vel_actual is not None]
    if len(ticks) < 50:
        return {"n": 0}

    commanded = achieved = tracking_loss = 0.0
    cmd_diff = act_diff = 0.0
    for a, b in zip(ticks, ticks[1:]):
        dt = b.t - a.t
        if not (0 < dt < 0.1):
            continue
        c_diff = a.right_vel_setpoint - a.left_vel_setpoint
        a_diff = a.right_vel_actual - a.left_vel_actual
        act_w = _wrap(b.pose[2] - a.pose[2]) / dt
        if any(math.isnan(v) for v in (c_diff, a_diff, act_w)):
            continue
        # --- track-width-free: did the wheels deliver the commanded differential?
        cmd_diff += c_diff * dt
        act_diff += a_diff * dt
        # --- track-width-dependent: rotation in radians
        commanded += (c_diff / track_width) * dt
        achieved += act_w * dt
        tracking_loss += ((a_diff - c_diff) / track_width) * dt

    deficit = achieved - commanded
    return {
        "n": len(ticks),
        # Track-width-FREE. Integrated wheel differential, actual vs commanded.
        # This is the honest measure of "did the drivetrain deliver the turn",
        # valid even for logs whose build constants are unknown: it compares
        # wheel speeds to wheel speeds and never divides by a track width.
        "commanded_differential_in": cmd_diff,
        "actual_differential_in": act_diff,
        "differential_delivered_fraction": (act_diff / cmd_diff
                                            if abs(cmd_diff) > 1e-6 else math.nan),
        # Track-width-DEPENDENT (uses `track_width`). Meaningful only when we
        # know the constant the firmware actually ran with; otherwise a wrong
        # track width shows up here as a fake rotation deficit.
        "track_width_used": track_width,
        "commanded_rotation_rad": commanded,
        "achieved_rotation_rad": achieved,
        "deficit_rad": deficit,
        "delivered_fraction": achieved / commanded if commanded else math.nan,
        "tracking_loss_rad": tracking_loss,
        # what share of the rotation gap the wheel-tracking error accounts for
        "explained_fraction": (tracking_loss / deficit
                               if abs(deficit) > 1e-6 else math.nan),
    }


def reverse_driving(run: Run, min_speed_in: float = 0.05,
                    min_cmd: float = 0.5) -> dict:
    """Ticks where the robot travels opposite its heading while commanded
    forward (and vice versa).

    (Ported from robot_animator.py's reverse detection.) Persistent
    unintended reversing means the drivetrain briefly moves against the
    command — wheel slip, collision, or sign errors.
    """
    n = unintended = 0
    for a, b in zip(run.ticks, run.ticks[1:]):
        if a.pose is None or b.pose is None:
            continue
        if a.left_vel_setpoint is None or a.right_vel_setpoint is None:
            continue
        dx, dy = b.pose[0] - a.pose[0], b.pose[1] - a.pose[1]
        if math.hypot(dx, dy) < min_speed_in:
            continue
        cmd = (a.left_vel_setpoint + a.right_vel_setpoint) / 2.0
        if math.isnan(cmd) or abs(cmd) < min_cmd:
            continue
        n += 1
        travel = math.atan2(dy, dx)
        moving_backwards = abs(_wrap(travel - a.pose[2])) > math.pi / 2
        if moving_backwards != (cmd < 0):
            unintended += 1
    return {"n": n, "unintended_count": unintended,
            "unintended_fraction": unintended / n if n else 0.0}


def _live_channels(columns: list[list[float]]) -> tuple[list[int], list[int], int]:
    """Split per-motor columns into (live, dead, corrupt_sample_count).

    Odometry::update logs a fixed three columns per side but indexes
    get_position_all() at [0], [1] and [2], and the drivetrain has only two
    motors per side (main.cpp). The third index is an out-of-bounds read on a
    2-element vector: it prints a constant 0.0 most ticks and occasionally
    whatever float happened to be adjacent in memory (-1984.052930, 25.331436,
    ...). Older logs (square.txt, init.txt) genuinely do have three motors a
    side, so the channel count must be discovered, not hardcoded.

    A live encoder moves. The discriminator is how often a column *changes*:
    a real motor steps nearly every tick while the robot drives, whereas the
    phantom column changed on 6 of 1396 ticks in 07302026-4. Keep columns that
    change at least a fifth as often as the busiest column on that side.

    Encoder counts are whole numbers, so non-integral samples are also counted
    and reported separately — they are direct evidence of the out-of-bounds
    read rather than something to silently discard.
    """
    activity, corrupt = [], 0
    for col in columns:
        changes = 0
        for a, b in zip(col, col[1:]):
            if math.isnan(a) or math.isnan(b) or math.isinf(a) or math.isinf(b):
                continue
            if a != b:
                changes += 1
        corrupt += sum(1 for v in col
                       if not math.isnan(v) and not math.isinf(v) and v != int(v))
        activity.append(changes)

    busiest = max(activity, default=0)
    live, dead = [], []
    for i, changes in enumerate(activity):
        if busiest > 0 and changes >= 0.2 * busiest:
            live.append(i)
        else:
            dead.append(i)
    return live, dead, corrupt


def _side_disagreement(ticks: list, attr: str) -> dict:
    """Per-motor agreement for one side of the drivetrain.

    'Left/Right Positions' are ABSOLUTE encoder counts. The odometry no longer
    tares the motors each tick — it keeps a running baseline (prevLeftPos /
    prevRightPos in Odometry::update, seeded once by seedSensorBaseline) and
    differences against it — so these columns carry an arbitrary per-motor
    offset that says nothing about drivetrain health. Every statistic here is
    therefore built from *differences*, never from the raw level:

    - ``delta_spread``   spread of the per-tick increments. Offset-free. Catches
      a frozen or wildly noisy encoder, but note that the two motors on a side
      are sampled a few hundred microseconds apart, so at speed this alternates
      sign tick to tick and largely cancels. Do not read a large p95 here as
      slip on its own.
    - ``net_divergence_ticks``  how far the motors' *net travel* drifted apart
      over the run (last minus first, so the offset cancels). Motors bolted to
      the same gearbox cannot really travel different distances, so anything
      that accumulates monotonically here is mechanical: a stripped pinion, a
      loose set screw, or a bad cartridge.
    - ``slip_fraction``  net divergence as a fraction of that side's total
      travel. This is the scale-free number to compare across runs — a long
      route naturally accumulates more raw ticks than a short one.
    """
    rows = [getattr(tk, attr) for tk in ticks if getattr(tk, attr)]
    width = min((len(r) for r in rows), default=0)
    if width < 2 or len(rows) < 2:
        return {"n": 0, "motors": 0}
    columns = [[r[i] for r in rows] for i in range(width)]
    live, dead, corrupt = _live_channels(columns)
    if len(live) < 2:
        # One live encoder is the expected shape once a bad motor has been cut
        # out of the odometry average; there is simply nothing to cross-check.
        return {"n": 0, "motors": len(live), "channels": live,
                "dropped_channels": dead, "corrupt_samples": corrupt}

    # --- per-tick increment spread (offset-free)
    spreads, travel = [], 0.0
    for a, b in zip(rows, rows[1:]):
        deltas = [b[i] - a[i] for i in live]
        if any(math.isnan(d) or math.isinf(d) for d in deltas):
            continue
        spreads.append(max(deltas) - min(deltas))
        travel += abs(sum(deltas) / len(deltas))

    # --- net travel per motor (first-to-last, so the untared offset cancels)
    net = [columns[i][-1] - columns[i][0] for i in live]
    divergence = max(net) - min(net)

    return {
        "n": len(spreads),
        "motors": len(live),
        "channels": live,
        "dropped_channels": dead,
        "corrupt_samples": corrupt,
        "delta_spread": _stats(spreads),
        "per_motor_net_ticks": net,
        "net_divergence_ticks": divergence,
        "net_divergence_in": divergence * INCHES_PER_TICK,
        "travel_ticks": travel,
        "travel_in": travel * INCHES_PER_TICK,
        "slip_fraction": divergence / travel if travel > 0 else math.nan,
        # Spread of the very first sample. With no per-tick tare this is just the
        # startup baseline, so it is reported and never asserted on: it is only
        # meaningful right after main.cpp's tare_position_all(), where a large
        # value means the tare did not land on every motor.
        "initial_offset_ticks": (max(columns[i][0] for i in live)
                                 - min(columns[i][0] for i in live)),
    }


def encoder_disagreement(run: Run) -> dict:
    """Agreement between the motors on each side of the drivetrain.

    Motors on the same side are geared to the same wheels, so their encoders
    must track each other. When they don't, Odometry::getAveragePosition
    averages the good reading with the bad one and the error goes silently into
    forwardDisplacement. Heading is unaffected (it comes from the IMU), so this
    shows up as a distance/position error, not a heading error.

    See ``_side_disagreement`` for the meaning of each field. Requires
    'Left/Right Positions' lines; returns n=0 per side for logs without them.
    """
    return {"left": _side_disagreement(run.ticks, "left_positions"),
            "right": _side_disagreement(run.ticks, "right_positions")}


def tracking_wheel_agreement(run: Run) -> dict:
    """Vertical tracking wheel vs drive-encoder forward travel ("TRACKWHEEL" lines).

    The vertical wheel sits on the tracking centre line and is unpowered, so it measures
    what the chassis actually travelled; the drive-encoder average measures what the
    powered wheels turned. Their gap is drive-wheel slip -- or, until both are
    calibrated, a diameter error (Config::VERTICAL_TRACKING_WHEEL_DIAMETER vs
    Config::WHEEL_DIAMETER).

    Fields:
      n                  ticks where the vertical wheel was used (vok=1)
      ticks              ticks with a TRACKWHEEL line
      vertical_fallback  ticks where Odometry fell back to the drive encoders
      lateral_fallback   ticks where Odometry assumed no sideslip
      wheel_travel_in    sum |dvert| over the used ticks
      wheel_net_in / drive_net_in   signed totals over the same ticks
      slip_fraction      |drive_net - wheel_net| / wheel_travel
      direction_dot      sum dvert*ddrive; negative means the vertical sensor is reversed
      lateral_travel_in  sum |dlat|
      wheel_abs_net_in   last minus first absolute vertical reading: everything the wheel
                         rolled, including deltas odometry rejected
      pose_forward_in    pose displacement along its heading, summed over ticks: what
                         odometry integrated. Far below wheel_abs_net_in means it dropped
                         wheel travel (boot 989: 68.8 vs 114.9 in)
    Returns {"ticks": 0} for logs without TRACKWHEEL lines.
    """
    rows = [tk.tracking_wheel for tk in run.ticks if tk.tracking_wheel]
    if not rows:
        return {"ticks": 0}
    used = [r for r in rows if r.get("vok") == 1 and math.isfinite(r.get("dvert", math.nan))
            and math.isfinite(r.get("ddrive", math.nan))]
    wheel_travel = sum(abs(r["dvert"]) for r in used)
    wheel_net = sum(r["dvert"] for r in used)
    drive_net = sum(r["ddrive"] for r in used)
    verts = [r["vert"] for r in rows if math.isfinite(r.get("vert", math.nan))]
    poses = [tk.pose for tk in run.ticks if tk.tracking_wheel and tk.pose]
    pose_forward = sum((b[0] - a[0]) * math.cos(a[2]) + (b[1] - a[1]) * math.sin(a[2])
                       for a, b in zip(poses, poses[1:]))
    return {
        "ticks": len(rows),
        "n": len(used),
        "vertical_fallback": sum(1 for r in rows if r.get("vok") != 1),
        "lateral_fallback": sum(1 for r in rows if r.get("lok") != 1),
        "wheel_travel_in": wheel_travel,
        "wheel_net_in": wheel_net,
        "drive_net_in": drive_net,
        "slip_fraction": abs(drive_net - wheel_net) / wheel_travel if wheel_travel > 0 else 0.0,
        "direction_dot": sum(r["dvert"] * r["ddrive"] for r in used),
        "lateral_travel_in": sum(abs(r["dlat"]) for r in rows
                                 if r.get("lok") == 1 and math.isfinite(r.get("dlat", math.nan))),
        "wheel_abs_net_in": verts[-1] - verts[0] if verts else 0.0,
        "pose_forward_in": pose_forward,
    }


def distance_integral_drift(run: Run) -> dict:
    """Cumulative commanded vs measured distance per side.

    (Ported from velocity_tuner.py's integral view.) End drift is the net
    distance the drivetrain under/over-delivered across the run — a direct
    measure of feedforward calibration (kS/kV in DrivetrainController).
    """
    sp_l = ac_l = sp_r = ac_r = 0.0
    abs_l = abs_r = 0.0   # total commanded travel, direction-independent
    n = 0
    for prev, tk in zip(run.ticks, run.ticks[1:]):
        if None in (prev.left_vel_setpoint, prev.left_vel_actual,
                    prev.right_vel_setpoint, prev.right_vel_actual):
            continue
        vals = (prev.left_vel_setpoint, prev.left_vel_actual,
                prev.right_vel_setpoint, prev.right_vel_actual)
        if any(math.isnan(v) for v in vals):
            continue
        dt = tk.t - prev.t
        if dt <= 0 or dt > 0.1:
            dt = DT_NOMINAL
        sp_l += prev.left_vel_setpoint * dt
        ac_l += prev.left_vel_actual * dt
        sp_r += prev.right_vel_setpoint * dt
        ac_r += prev.right_vel_actual * dt
        abs_l += abs(prev.left_vel_setpoint) * dt
        abs_r += abs(prev.right_vel_setpoint) * dt
        n += 1
    if n == 0:
        return {"n": 0}
    return {"n": n,
            "left_commanded_in": sp_l, "left_measured_in": ac_l,
            "right_commanded_in": sp_r, "right_measured_in": ac_r,
            "left_travel_in": abs_l, "right_travel_in": abs_r,
            "left_drift_in": ac_l - sp_l, "right_drift_in": ac_r - sp_r}


# ------------------------------------------- regime / gain / feasibility

def _as_runs(runs: Run | list[Run]) -> list[Run]:
    return runs if isinstance(runs, list) else [runs]


def _thin(xs: list, cap: int = 5000) -> list:
    """Evenly subsample a list to at most `cap` items (for plotting)."""
    if len(xs) <= cap:
        return xs
    step = len(xs) / cap
    return [xs[int(i * step)] for i in range(cap)]


def _wheel_pairs(tk) -> list[tuple[float, float]] | None:
    """(setpoint, actual) for both wheels of a tick, or None if incomplete."""
    vals = (tk.left_vel_setpoint, tk.left_vel_actual,
            tk.right_vel_setpoint, tk.right_vel_actual)
    if any(v is None or math.isnan(v) for v in vals):
        return None
    return [(vals[0], vals[1]), (vals[2], vals[3])]


def motion_regimes(runs: Run | list[Run],
                   straight_min_v: float = 20.0,
                   turn_min_w: float = 8.0,
                   samples: bool = False) -> dict:
    """Delivered/commanded wheel-speed gain split by motion regime.

    Ticks are classified from the wheel setpoints (v = (l+r)/2 chassis
    speed, w = (r-l)/2 wheel-speed half-difference, both in in/s):
      straight:  |v| > straight_min_v and |w| < 0.15|v|
      turn:      |w| > turn_min_w     and |v| < 0.3|w|   (in-place)
      arc:       |w| > 5 and |v| > 10
    Gain = median(actual/setpoint) over both wheels where |setpoint| > 10.

    A pure-turn gain well below the straight gain is turning scrub that the
    shared feedforward cannot represent: kS/kV/kA (Config, applied per wheel
    in DrivetrainController) are fitted to straight driving, so turns are
    systematically under-driven. Fix: separate angular feedforward constants
    superposed in wheel space, and/or heading-rate feedback from the IMU
    gyro. Also reports heading-error spread per regime — turning is where
    heading control degrades.
    """
    buckets: dict[str, list[tuple[float, float]]] = {"straight": [], "turn": [], "arc": []}
    hdg: dict[str, list[float]] = {"straight": [], "turn": []}
    for run in _as_runs(runs):
        for tk in run.ticks:
            pairs = _wheel_pairs(tk)
            if pairs is None:
                continue
            v = (pairs[0][0] + pairs[1][0]) / 2.0
            w = (pairs[1][0] - pairs[0][0]) / 2.0
            if abs(v) > straight_min_v and abs(w) < 0.15 * abs(v):
                regime = "straight"
            elif abs(w) > turn_min_w and abs(v) < 0.3 * abs(w):
                regime = "turn"
            elif abs(w) > 5.0 and abs(v) > 10.0:
                regime = "arc"
            else:
                continue
            buckets[regime] += pairs
            if regime in hdg and tk.pose is not None and tk.goal is not None:
                e = _wrap(tk.goal[2] - tk.pose[2])
                if not math.isnan(e):
                    hdg[regime].append(e)

    def gain(pairs: list[tuple[float, float]]) -> dict:
        ratios = [a / s for s, a in pairs
                  if abs(s) > 10.0 and -1.0 < a / s < 3.0]
        return {"n": len(ratios), "gain": median(ratios) if ratios else None}

    out = {name: gain(pairs) for name, pairs in buckets.items()}
    for name, errs in hdg.items():
        out[name]["heading_err_sd_rad"] = pstdev(errs) if len(errs) > 1 else None
    gs, gt = out["straight"]["gain"], out["turn"]["gain"]
    out["turn_to_straight_ratio"] = gt / gs if gs and gt else None
    if samples:
        for name in buckets:
            out[name]["samples"] = _thin(buckets[name])
    return out


def steady_state_gain(runs: Run | list[Run], window: int = 20,
                      samples: bool = False) -> dict:
    """Median measured/commanded wheel speed on settled cruise stretches.

    A stretch qualifies when the last `window` setpoints on that side are
    all > 15 in/s, below 90% of the physical max (so feedback has headroom),
    and within 10% of each other (quasi-constant). The residual after
    feedback has settled isolates feedforward calibration: gain < 1 means
    Config::KV under-delivers (kP fills part of the gap; with kI = 0 the
    rest persists). The left/right asymmetry is a drivetrain health check —
    a gap of a few percent on one side is mechanical (failing motor,
    friction), not tuning.
    """
    gains: dict[str, list[float]] = {"left": [], "right": []}
    for run in _as_runs(runs):
        for side in ("left", "right"):
            sps = [getattr(tk, f"{side}_vel_setpoint") for tk in run.ticks]
            acs = [getattr(tk, f"{side}_vel_actual") for tk in run.ticks]
            for i in range(window, len(run.ticks)):
                win = sps[i - window:i + 1]
                if any(s is None or math.isnan(s) for s in win):
                    continue
                a = acs[i]
                if a is None or math.isnan(a):
                    continue
                lo, hi = min(win), max(win)
                if lo > 15.0 and hi < 0.9 * MAX_WHEEL_SPEED and hi - lo < 0.1 * hi:
                    gains[side].append(a / sps[i])
    out: dict = {side: {"n": len(xs), "gain": median(xs) if xs else None}
                 for side, xs in gains.items()}
    gl, gr = out["left"]["gain"], out["right"]["gain"]
    out["asymmetry"] = (abs(gl - gr) / ((gl + gr) / 2.0)) if gl and gr else None
    if samples:
        for side, xs in gains.items():
            out[side]["samples"] = _thin(xs)
    return out


def setpoint_feasibility(runs: Run | list[Run],
                         vmax: float = USABLE_WHEEL_SPEED,
                         samples: bool = False) -> dict:
    """Commanded wheel speeds beyond what the drivetrain can actually deliver.

    Infeasible setpoints guarantee voltage saturation, and a saturated wheel is
    open-loop: the commanded curvature is whatever the motors happen to manage.

    The bar here is the *usable* speed under load (~85% of the 64.8 in/s free
    speed implied by config.hpp gearing), not free speed itself. The old default
    was a hardcoded 74.0 in/s -- above even this drivetrain's no-load ceiling --
    so the check could never fail, and it duly passed initial_tuning.txt while
    the left wheel sat on the voltage rail for 28% of the run. A setpoint the
    motors can only reach with 100% of their voltage is not "feasible" in any
    useful sense: it leaves the feedback term zero authority exactly when the
    robot is off-path and needs it.

    A setpoint the firmware desaturated to exactly MAX_WHEEL_VELOCITY is
    feasible, not infeasible. The log prints it rounded to 6 decimals
    (55.276323 for a cap of 55.27632273991241), so a bare `> vmax` counts every
    correctly-clamped tick as a violation -- which is how 07132026.txt reported
    4.4% infeasible setpoints while desaturation was working perfectly. Compare
    against the cap plus the print precision.
    """
    tol = vmax * _SETPOINT_REL_TOL
    n = over = 0
    worst = 0.0
    mags = []
    for run in _as_runs(runs):
        for tk in run.ticks:
            for sp in (tk.left_vel_setpoint, tk.right_vel_setpoint):
                if sp is None or math.isnan(sp):
                    continue
                n += 1
                if abs(sp) > vmax + tol:
                    over += 1
                worst = max(worst, abs(sp))
                if samples:
                    mags.append(abs(sp))
    out = {"n": n, "infeasible_count": over,
           "infeasible_fraction": over / n if n else 0.0,
           "max_setpoint_in_s": worst, "vmax_in_s": vmax,
           "free_speed_in_s": FREE_WHEEL_SPEED}
    if samples:
        out["samples"] = _thin(mags, 20000)
    return out


def saturation_cost(runs: Run | list[Run], samples: bool = False) -> dict:
    """|velocity error| on voltage-saturated vs unsaturated ticks, per wheel.

    Saturated ticks are effectively open-loop: the controller wants more
    than +/-127 and tracking degrades by whatever the shortfall is. A large
    saturated/unsaturated error ratio quantifies what infeasible setpoints
    and the missing desaturation step cost (competition logs showed
    ~15-26 in/s saturated vs ~2-3 in/s unsaturated).
    """
    sat, unsat = [], []
    for run in _as_runs(runs):
        for tk in run.ticks:
            for sp, ac, volt in (
                    (tk.left_vel_setpoint, tk.left_vel_actual, tk.voltage_left),
                    (tk.right_vel_setpoint, tk.right_vel_actual, tk.voltage_right)):
                if volt is None or sp is None or ac is None:
                    continue
                if math.isnan(sp) or math.isnan(ac):
                    continue
                (sat if abs(volt) >= VOLTAGE_LIMIT else unsat).append(abs(sp - ac))
    out = {"saturated": _stats(sat), "unsaturated": _stats(unsat)}
    if sat and unsat and mean(unsat) > 0:
        out["mean_ratio"] = mean(sat) / mean(unsat)
    else:
        out["mean_ratio"] = None
    if samples:
        out["saturated"]["samples"] = _thin(sat, 20000)
        out["unsaturated"]["samples"] = _thin(unsat, 20000)
    return out


# ------------------------------------------------------------ wall resets

@dataclass
class WallSample:
    """What one wall-reset sensor says about the pose at one tick."""
    t: float
    sensor: str              # front / back / left / right
    axis: str                # 'x' or 'y': the coordinate this sensor would reset
    wall: int                # 0 +x, 1 +y, 2 -x, 3 -y (DistanceResetOdometry's numbering)
    off_normal_deg: float    # beam angle off the wall normal
    reading_in: float
    implied: float           # the coordinate the wall puts the robot at
    error_in: float          # implied - pose: what a reset here would correct


def _lround(x: float) -> int:
    """std::lround: halves away from zero (Python's round() is banker's)."""
    return int(math.floor(abs(x) + 0.5)) * (1 if x >= 0 else -1)


def wall_estimate(pose, sensor: str, reading_in: float,
                  max_beam_deg: float = MAX_BEAM_ANGLE_DEG):
    """DistanceResetOdometry::reset's geometry, offline.

    Returns (axis, wall, off_normal_rad, implied coordinate), or None when the
    sensor is unknown or its beam is further off a wall normal than a reset accepts.
    Keep in step with src/navigation/distance_reset_odometry.cpp;
    test_offline_wall_math_matches_firmware checks it against logged corrections.
    """
    if sensor not in RESET_SENSORS:
        return None
    angle_deg, along, across = RESET_SENSORS[sensor]
    beam = _wrap(pose[2] + math.radians(angle_deg))
    wall = _lround(beam / (math.pi / 2)) % 4
    off = _wrap(beam - wall * (math.pi / 2))
    if abs(off) > math.radians(max_beam_deg):
        return None
    wall_to_center = (reading_in + along) * math.cos(off) + across * math.sin(off)
    sign = 1.0 if wall in (0, 1) else -1.0
    axis = "x" if wall in (0, 2) else "y"
    return axis, wall, off, sign * (WALL_POSITION - wall_to_center)


def wall_samples(run: Run, max_beam_deg: float = MAX_BEAM_ANGLE_DEG) -> list[WallSample]:
    """Per tick, per sensor: where its wall puts the robot vs where odometry does.

    Needs the per-tick "DISTREAD" line; returns [] for older logs. error_in is the
    correction a reset at that tick would apply (single reading, not the firmware's
    100 ms median, and no speed gate), so it shows how far odometry has drifted at
    every point a sensor squarely faces a wall, not just where the route resets.
    A beam that hits a field element instead of the wall shows up as a large,
    sudden error on that sensor alone.
    """
    out = []
    for tk in run.ticks:
        if tk.distance_mm is None or tk.pose is None:
            continue
        for sensor, mm in tk.distance_mm.items():
            if mm < 0 or mm >= 9999:
                continue
            reading = mm / 25.4
            est = wall_estimate(tk.pose, sensor, reading, max_beam_deg)
            if est is None:
                continue
            axis, wall, off, implied = est
            pos = tk.pose[0] if axis == "x" else tk.pose[1]
            out.append(WallSample(tk.t, sensor, axis, wall, math.degrees(off),
                                  reading, implied, implied - pos))
    return out


def stop_windows(run: Run, max_speed: float = 0.5, min_s: float = 0.05) -> list[tuple[int, int]]:
    """[start, end) tick ranges where the *planned* pose (Goal) moves slower than
    max_speed in/s for at least min_s: the route's stops, holds and waits, i.e. the
    places a reset could go. The goal is exactly still there, unlike the measured
    pose, which settles, creeps and gets pushed around by contact."""
    out, start = [], None
    for i in range(1, len(run.ticks)):
        a, b = run.ticks[i - 1], run.ticks[i]
        still = False
        if a.goal is not None and b.goal is not None:
            dt = b.t - a.t
            if dt <= 0 or dt > 0.1:
                dt = DT_NOMINAL
            still = math.hypot(b.goal[0] - a.goal[0], b.goal[1] - a.goal[1]) / dt < max_speed
        if still and start is None:
            start = i - 1
        elif not still and start is not None:
            if run.ticks[i - 1].t - run.ticks[start].t >= min_s:
                out.append((start, i))
            start = None
    if start is not None and run.ticks[-1].t - run.ticks[start].t >= min_s:
        out.append((start, len(run.ticks)))
    return out


def wall_errors_at_stops(run: Run, max_beam_deg: float = MAX_BEAM_ANGLE_DEG) -> list[dict]:
    """For every stop: the pose there and, per sensor facing a wall, the median
    correction a reset during the first 100 ms of that stop would have applied.

    Taken before any reset at the stop moves the pose, so it reads as "how wrong was
    odometry on arrival here". The spread says whether the sensor read steadily
    (a wall) or jumped (a beam clipping something).
    """
    samples = wall_samples(run, max_beam_deg)
    out = []
    for a, b in stop_windows(run):
        t0 = run.ticks[a].t
        first_reset = min((r.t for r in run.resets if r.applied and t0 <= r.t < run.ticks[b - 1].t),
                          default=None)
        t1 = min(t0 + 0.1, first_reset) if first_reset is not None else t0 + 0.1
        by = {}
        for s in samples:
            if t0 <= s.t <= t1:
                by.setdefault((s.sensor, s.axis), []).append(s)
        sensors = {}
        for (sensor, axis), ss in sorted(by.items()):
            errs = [s.error_in for s in ss]
            sensors[sensor] = {"axis": axis, "n": len(ss), "error_in": median(errs),
                               "spread_in": max(errs) - min(errs),
                               "off_normal_deg": median(s.off_normal_deg for s in ss),
                               "reading_in": median(s.reading_in for s in ss)}
        pose = run.ticks[a].pose
        out.append({"t": t0, "duration_s": run.ticks[b - 1].t - t0, "pose": pose,
                    "reset_here": first_reset is not None, "sensors": sensors})
    return out


def reset_agreement(run: Run) -> list[dict]:
    """Each logged reset recomputed offline with wall_estimate().

    Uses the median reading the firmware logged and the pose of the tick before the
    reset. The reset line doesn't name its sensor, so the sensor is the one whose
    DISTREAD readings around the reset are closest to that median. Resets without
    DISTREAD data around them are skipped (older logs).
    """
    out = []
    for r in run.resets:
        if r.correction_in is None or r.sensor_reading_in is None:
            continue
        if not (0 <= r.tick_index < len(run.ticks)) or run.ticks[r.tick_index].pose is None:
            continue
        near = [tk.distance_mm for tk in run.ticks[max(0, r.tick_index - 9):r.tick_index + 2]
                if tk.distance_mm is not None]
        if not near:
            continue
        pose = run.ticks[r.tick_index].pose
        best = None
        for sensor in RESET_SENSORS:
            vals = [d[sensor] / 25.4 for d in near if 0 <= d.get(sensor, -1) < 9999]
            if not vals:
                continue
            est = wall_estimate(pose, sensor, r.sensor_reading_in, max_beam_deg=90.0)
            if est is None or est[0] != r.reset_axis:
                continue
            gap = abs(median(vals) - r.sensor_reading_in)
            if best is None or gap < best[0]:
                pos = pose[0] if est[0] == "x" else pose[1]
                best = (gap, sensor, est[3] - pos)
        if best is None:
            continue
        out.append({"t": r.t, "sensor": best[1], "axis": r.reset_axis,
                    "logged_in": r.correction_in, "offline_in": best[2],
                    "diff_in": best[2] - r.correction_in})
    return out


# ---------------------------------------------------------------- warnings

def warning_summary(run: Run) -> dict:
    invalid_dt = [w for _, w in run.warnings if w.startswith("Invalid deltaTime")]
    bad_dt_values = []
    for w in invalid_dt:
        try:
            bad_dt_values.append(float(w.split(":")[1].split(",")[0]))
        except (IndexError, ValueError):
            pass
    mid_run_stalls = [v for v in bad_dt_values if 0.1 < v < 2.0]
    return {
        "invalid_deltatime_count": len(invalid_dt),
        "zero_dt_count": sum(1 for v in bad_dt_values if v <= 0),
        "mid_run_stall_count": len(mid_run_stalls),
        "worst_stall_s": max(mid_run_stalls, default=0.0),
        "invalid_sensor_count": sum(
            1 for _, w in run.warnings if w.startswith("Invalid distance sensor")),
    }


# ---------------------------------------------------------------- spins (traction A/B)

def traction_wheels(run: Run) -> int | None:
    """Traction wheels per side the firmware was built for (Config::TRACTION_WHEELS_PER_SIDE,
    "Build config: ... tractionWheels="). None for logs from before 2026-10-09."""
    n = run.build_config.get("tractionWheels")
    return int(n) if n is not None and math.isfinite(n) else None


def _spin_samples(run: Run, min_rate: float = 0.3, max_speed: float = 1.0,
                  smooth: int = 5) -> tuple[list[dict], list]:
    """Ticks where the plan turns in place (|mp_w| >= min_rate rad/s, |mp_v| <= max_speed
    in/s), each with this tick's heading change and the yaw rate the pose (IMU) shows,
    differenced over `smooth` ticks. Returns (samples, pose ticks)."""
    ticks = [tk for tk in run.ticks if tk.pose is not None]
    theta, prev = [], None
    for tk in ticks:
        theta.append(tk.pose[2] if prev is None else theta[-1] + _wrap(tk.pose[2] - prev))
        prev = tk.pose[2]
    half = smooth // 2
    out = []
    for i, tk in enumerate(ticks):
        if i == 0 or tk.mp_w is None or tk.mp_v is None:
            continue
        if abs(tk.mp_w) < min_rate or abs(tk.mp_v) > max_speed:
            continue
        lo, hi = max(0, i - half), min(len(ticks) - 1, i + half)
        span = ticks[hi].t - ticks[lo].t
        out.append({"i": i, "tick": tk, "dtheta": theta[i] - theta[i - 1],
                    "w": (theta[hi] - theta[lo]) / span if span > 0 else math.nan})
    return out, ticks


def _ols(xs: list[float], ys: list[float]) -> tuple[float, float, float]:
    """Least-squares line y = a + b x: (a, b, r^2)."""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    syy = sum((y - my) ** 2 for y in ys)
    b = sxy / sxx if sxx > 0 else math.nan
    a = my - b * mx
    r2 = (sxy * sxy / (sxx * syy)) if sxx > 0 and syy > 0 else math.nan
    return a, b, r2


def tracking_wheel_offsets(run: Run, min_rate: float = 0.3) -> dict:
    """Tracking-wheel offsets from the pivot, fitted on turn-in-place ticks.

    In a pure spin a wheel rolls only because it sits off the pivot: the lateral wheel
    reads +offset * dtheta (Config::LATERAL_WHEEL_OFFSET, +forward) and the vertical one
    -offset * dtheta (Config::VERTICAL_WHEEL_OFFSET, +left), the convention of
    navigation/tracking_arc.hpp. Each is a least-squares line against the IMU's dtheta;
    its intercept soaks up steady drift, so spinning both ways cancels it. The lateral
    diameter scales the reading and the fit alike, so an uncalibrated one barely matters.

    Fields (absent when the run has under pi rad of usable spin):
      n, radians                      ticks and |rotation| used
      lateral_offset_in, vertical_offset_in   the fits
      lateral_ccw_in / lateral_cw_in, vertical_ccw_in / vertical_cw_in   the same per
          spin direction (through the origin); far apart means the pivot moves
      lateral_residual_in_per_rad     what the lateral wheel saw beyond the fit, per
          radian turned: sideways scrub
      pivot_drift_in_per_rev          how far the pose's tracking centre ends up from
          where each spin began, per revolution
    """
    samples, ticks = _spin_samples(run, min_rate)
    lat, vert = [], []
    for sm in samples:
        tw = sm["tick"].tracking_wheel
        d = sm["dtheta"]
        if not tw or not math.isfinite(d):
            continue
        if tw.get("lok") == 1 and math.isfinite(tw.get("dlat", math.nan)):
            lat.append((d, tw["dlat"]))
        if tw.get("vok") == 1 and math.isfinite(tw.get("dvert", math.nan)):
            vert.append((d, tw["dvert"]))
    radians = sum(abs(d) for d, _ in lat)
    out = {"n": len(lat), "radians": radians}
    if len(lat) < 20 or radians < math.pi or len(vert) < 20:
        return out

    def through_origin(rows, sign):
        rows = [(d, y) for d, y in rows if d * sign > 0]
        sxx = sum(d * d for d, _ in rows)
        return sum(d * y for d, y in rows) / sxx if sxx > 0 else math.nan

    a_lat, b_lat, _ = _ols([d for d, _ in lat], [y for _, y in lat])
    _, b_vert, _ = _ols([d for d, _ in vert], [y for _, y in vert])
    resid = sum(abs(y - (a_lat + b_lat * d)) for d, y in lat)

    # Pivot drift: pose displacement across each contiguous spin segment
    drift = revs = 0.0
    seg = [samples[0]] if samples else []
    for sm in samples[1:] + [None]:
        if sm is not None and sm["i"] == seg[-1]["i"] + 1:
            seg.append(sm)
            continue
        turned = sum(abs(x["dtheta"]) for x in seg if math.isfinite(x["dtheta"]))
        if turned > math.pi:
            p0, p1 = ticks[seg[0]["i"] - 1].pose, seg[-1]["tick"].pose
            drift += math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            revs += turned / (2 * math.pi)
        seg = [sm] if sm is not None else []

    out.update({
        "lateral_offset_in": b_lat,
        "vertical_offset_in": -b_vert,
        "lateral_ccw_in": through_origin(lat, 1), "lateral_cw_in": through_origin(lat, -1),
        "vertical_ccw_in": -through_origin(vert, 1), "vertical_cw_in": -through_origin(vert, -1),
        "lateral_residual_in_per_rad": resid / radians,
        "pivot_drift_in_per_rev": drift / revs if revs > 0 else math.nan,
    })
    return out


def spin_scrub(run: Run, min_rate: float = 0.3, plateau_tol: float = 0.02) -> dict:
    """What turning in place costs, from the spin-rate routes (spin_w1..w4, spin_max).

    On the plateau of a spin (planned rate steady, neither wheel at the rail) each wheel's
    voltage beyond the straight-line feedforward, sign(v) * V - (KS + KV * |v|) with v the
    measured wheel speed, is what the turn itself cost: Config::KS_ANGULAR +
    KV_ANGULAR * |w| if those were right, plus whatever feedback added. fit_spin_scrub
    regresses it on |w| over several rates. v comes from the drive encoders, so a wrong
    WHEEL_DIAMETER biases KV * |v| -- identically for both sides of an A/B.

    Fields: n (spin ticks), plateau_n, saturated_fraction (plateau ticks at the rail),
    rate_mean / plan_rate_mean (rad/s on the plateau), delivered_ratio (their ratio),
    scrub_v / scrub_v_left / scrub_v_right (mean extra voltage, motor units),
    effective_track_width_in ((vR - vL) / w over every spin tick), top_rate (rad/s),
    top_accel (spin-up rad/s^2), points ((|w|, scrub_left, scrub_right) per plateau tick).
    """
    samples, ticks = _spin_samples(run, min_rate)
    pts, plan, eff = [], [], []
    plateau = sat = 0
    for sm in samples:
        tk, i, w = sm["tick"], sm["i"], sm["w"]
        if None in (tk.voltage_left, tk.voltage_right, tk.left_vel_actual, tk.right_vel_actual):
            continue
        if not math.isfinite(w):
            continue
        eff.append((w, tk.right_vel_actual - tk.left_vel_actual))
        prev = ticks[i - 1].mp_w
        if prev is None or abs(tk.mp_w - prev) > plateau_tol:
            continue
        plateau += 1
        if abs(tk.voltage_left) >= VOLTAGE_LIMIT or abs(tk.voltage_right) >= VOLTAGE_LIMIT:
            sat += 1
            continue
        vl, vr = tk.left_vel_actual, tk.right_vel_actual
        if min(abs(vl), abs(vr)) < 2.0:
            continue
        sl = math.copysign(1.0, vl) * tk.voltage_left - (KS + KV * abs(vl))
        sr = math.copysign(1.0, vr) * tk.voltage_right - (KS + KV * abs(vr))
        pts.append((abs(w), sl, sr))
        plan.append((abs(tk.mp_w), abs(w)))
    if not eff:
        return {"n": 0}

    ws = [sm["w"] for sm in samples if math.isfinite(sm["w"])]
    # Spin-up yaw acceleration (|w| rising) over a 10-tick span: braking is helped by the
    # scrub and the motors, so it says nothing about the drive's limit, and IMU noise on
    # the smoothed rate would set the maximum over a shorter span.
    accel = 0.0
    for k in range(5, len(samples) - 5):
        a, b = samples[k - 5], samples[k + 5]
        if b["i"] - a["i"] == 10 and math.isfinite(a["w"]) and math.isfinite(b["w"]):
            span = b["tick"].t - a["tick"].t
            if span > 0:
                accel = max(accel, (abs(b["w"]) - abs(a["w"])) / span)
    sxx = sum(w * w for w, _ in eff)
    out = {
        "n": len(eff),
        "plateau_n": plateau,
        "saturated_fraction": sat / plateau if plateau else 0.0,
        "effective_track_width_in": sum(w * dv for w, dv in eff) / sxx if sxx > 0 else math.nan,
        "top_rate": max(abs(w) for w in ws) if ws else math.nan,
        "top_accel": accel,
        "points": pts,
    }
    if pts:
        out.update({
            "rate_mean": mean(w for _, w in plan),
            "plan_rate_mean": mean(p for p, _ in plan),
            "delivered_ratio": mean(w for _, w in plan) / mean(p for p, _ in plan),
            "scrub_v_left": mean(p[1] for p in pts),
            "scrub_v_right": mean(p[2] for p in pts),
            "scrub_v": mean(0.5 * (p[1] + p[2]) for p in pts),
        })
    return out


def spin_creep(run: Run) -> dict:
    """Forward speed RAMSETE commands while the plan spins in place, per rad/s turned.

    A spin should command none. A wrong Config::LATERAL_WHEEL_OFFSET makes the pose orbit
    a point ahead of the robot, so the goal reads as ahead all through the spin and
    RAMSETE drives toward it: boot 1001 (offset 3.0 vs 2.37 measured) crept 0.72 in per
    radian, ~4.5 in per revolution, in both directions. Returns {} without spin ticks."""
    samples, _ = _spin_samples(run)
    rows = [(sm["tick"].ramsete_v, abs(sm["w"])) for sm in samples
            if sm["tick"].ramsete_v is not None and math.isfinite(sm["w"]) and abs(sm["w"]) > 0.3]
    if len(rows) < 20:
        return {}
    return {"n": len(rows), "creep_in_per_rad": mean(v for v, _ in rows) / mean(w for _, w in rows)}


def fit_spin_scrub(runs: Run | list[Run]) -> dict:
    """Turn cost against yaw rate over several spin runs (pool spin_w1..w4 for one robot).

    Fits mean extra voltage per wheel = ks + kv * |w| on every plateau tick of every run:
    ks estimates Config::KS_ANGULAR (motor units), kv KV_ANGULAR ((motor units)/(rad/s)).
    Needs at least two distinct rates to separate them. Also per side, since a side that
    drags shows up as a higher intercept on that side only.
    """
    pts = [p for r in _as_runs(runs) for p in spin_scrub(r).get("points", [])]
    if len(pts) < 30:
        return {"n": len(pts)}
    ws = [p[0] for p in pts]
    if max(ws) - min(ws) < 0.8:
        return {"n": len(pts), "rate_span": max(ws) - min(ws)}
    ks, kv, r2 = _ols(ws, [0.5 * (p[1] + p[2]) for p in pts])
    ks_l, kv_l, _ = _ols(ws, [p[1] for p in pts])
    ks_r, kv_r, _ = _ols(ws, [p[2] for p in pts])
    return {"n": len(pts), "rate_min": min(ws), "rate_max": max(ws), "ks": ks, "kv": kv, "r2": r2,
            "ks_left": ks_l, "kv_left": kv_l, "ks_right": ks_r, "kv_right": kv_r}
