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
- setpoint_feasibility   -> commanded wheel speeds beyond the physical max
                            (route generator limits + missing desaturation)
- saturation_cost        -> velocity error on saturated vs unsaturated ticks

motion_regimes / steady_state_gain / setpoint_feasibility / saturation_cost
accept a single Run or a list of Runs; pass a whole file's runs to pool
sparse regimes (in-place turns are rare within any single run).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from statistics import mean, median, pstdev

from .parser import Run

DT_NOMINAL = 0.010          # Config::DT = 10 ms
MAX_WHEEL_SPEED = 74.0      # in/s: 450 rpm * (36/48) * 3.25" * pi / 60 ~= 57.4;
                            # allow generous margin, anything above is unphysical
VOLTAGE_LIMIT = 127


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
                track_width: float = 14.0) -> dict:
    """Integrate commanded/measured velocities into a path; compare to goals.

    (Ported from goal_animator.py / velo_checker.py.) Sources:
      'mp'              — MP Output chassis velocities (square/init logs only)
      'wheel_setpoints' — commanded wheel velocities via diff-drive kinematics
      'wheel_actuals'   — measured wheel velocities via diff-drive kinematics

    Seeded from the first goal pose (as the legacy scripts did). Returns the
    simulated path, goal path, endpoint drift, and index-aligned error stats.
    Note: Config::WHEEL_BASE_WIDTH is 14.0 in; the legacy velo_checker.py
    used 12.7 — pass track_width to compare.
    """
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


def effective_track_width(run: Run, nominal: float = 14.0) -> dict:
    """Estimate the real track width from this run's data.

    Encoder-implied rotation is (ΔR - ΔL) / track_width; the pose theta
    (IMU-driven) is the trusted rotation. If total rotation is significant,
    width_est = (ΔR_total - ΔL_total) / total_rotation. A mismatch with
    Config::WHEEL_BASE_WIDTH (14.0) biases every RAMSETE wheel-velocity
    conversion and the accel split in Robot::processTrajectory.
    """
    ticks = [tk for tk in run.ticks
             if tk.cum_left is not None and tk.pose is not None]
    if len(ticks) < 50:
        return {"n": 0}
    d_enc = (ticks[-1].cum_right - ticks[0].cum_right) - \
            (ticks[-1].cum_left - ticks[0].cum_left)
    total_rot = 0.0
    for a, b in zip(ticks, ticks[1:]):
        total_rot += _wrap(b.pose[2] - a.pose[2])
    if abs(total_rot) < 1.5:  # need real net rotation for a stable estimate
        return {"n": len(ticks), "total_rotation_rad": total_rot,
                "estimate_in": None, "nominal_in": nominal}
    est = d_enc / total_rot
    return {"n": len(ticks), "total_rotation_rad": total_rot,
            "estimate_in": est, "nominal_in": nominal,
            "ratio": est / nominal if nominal else math.nan}


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


def encoder_disagreement(run: Run) -> dict:
    """Spread between same-side motor encoders per tick.

    Units are raw encoder ticks per 10 ms interval (Odometry tares the
    motors every update, so each logged 'Positions' line is a per-iteration
    delta). (Data source ported from drivetrain_readings_visualizer.py.)
    A large spread means one motor slips or reads wrong — the odometry
    average silently absorbs it. Requires 'Left/Right Positions' lines
    (square/init-style logs).
    """
    spreads_l, spreads_r = [], []
    for tk in run.ticks:
        if tk.left_positions and len(tk.left_positions) > 1:
            spreads_l.append(max(tk.left_positions) - min(tk.left_positions))
        if tk.right_positions and len(tk.right_positions) > 1:
            spreads_r.append(max(tk.right_positions) - min(tk.right_positions))
    return {"left": _stats(spreads_l), "right": _stats(spreads_r)}


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
                         vmax: float = MAX_WHEEL_SPEED,
                         samples: bool = False) -> dict:
    """Commanded wheel speeds beyond what the drivetrain can physically do.

    Infeasible setpoints guarantee voltage saturation, and with no
    desaturation step (RAMSETE limits unused; per-side clamp at +/-127 in
    DrivetrainController) the left/right ratio — the commanded curvature —
    is destroyed exactly when correction matters most. Sources: routes
    generated beyond the real velocity limit, and RAMSETE corrections on
    top of an already-fast reference.
    """
    n = over = 0
    worst = 0.0
    mags = []
    for run in _as_runs(runs):
        for tk in run.ticks:
            for sp in (tk.left_vel_setpoint, tk.right_vel_setpoint):
                if sp is None or math.isnan(sp):
                    continue
                n += 1
                if abs(sp) > vmax:
                    over += 1
                worst = max(worst, abs(sp))
                if samples:
                    mags.append(abs(sp))
    out = {"n": n, "infeasible_count": over,
           "infeasible_fraction": over / n if n else 0.0,
           "max_setpoint_in_s": worst, "vmax_in_s": vmax}
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
