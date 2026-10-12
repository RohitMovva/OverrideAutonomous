"""Parser for OverrideAutonomous robot logs.

Log structure (produced by src/utilities/logger.cpp):
    [HH:MM:SS.mmm] [LEVEL] message

A "session" starts at each "Robot initialized" line (same convention as
tests/logs/splitter.py). Within a session, each call to
Robot::followTrajectory() produces a "run", opened by an "Initial pose:" line.

Within a run, each 10 ms control-loop iteration that lands on a
TrajectoryPoint emits (see Robot::processTrajectory):
    Pose: x y theta
    Goal: x y theta
    Left wheel velocities: <setpoint> <actual>
    Right wheel velocities: <setpoint> <actual>
    Voltages: <left> <right>
Some logs additionally contain odometry / feedforward internals
(Ramsete Output, MP Output, Left Feedforward, Velocity/Acceleration, ...).

The parser groups those lines into ControlTick records anchored on the
"Pose:" line, and collects action points, distance resets, and warnings.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

LINE_RE = re.compile(r"^\[(\d{2}):(\d{2}):(\d{2})\.(\d{3})\] \[(\w+)\] (.*)$")

# printf("%f") of nan/inf shows up as nan/-nan/inf/-inf
FLOAT = r"[-+]?(?:nan|inf|[\d.]+(?:e[-+]?\d+)?)"


def _f(s: str) -> float:
    return float(s)  # python parses 'nan'/'-nan'? '-nan' -> ValueError, handle:


def parse_float(s: str) -> float:
    s = s.strip()
    if s in ("-nan", "nan"):
        return math.nan
    if s == "inf":
        return math.inf
    if s == "-inf":
        return -math.inf
    return float(s)


@dataclass
class ControlTick:
    t: float                     # seconds since midnight (log timestamp)
    pose: tuple[float, float, float] | None = None
    goal: tuple[float, float, float] | None = None
    left_vel_setpoint: float | None = None
    left_vel_actual: float | None = None
    right_vel_setpoint: float | None = None
    right_vel_actual: float | None = None
    voltage_left: int | None = None
    voltage_right: int | None = None
    ramsete_v: float | None = None
    ramsete_w: float | None = None
    mp_v: float | None = None
    mp_w: float | None = None
    ff_left: float | None = None
    ff_right: float | None = None
    fb_left: float | None = None
    fb_right: float | None = None
    delta_theta: float | None = None
    imu_heading: float | None = None
    heading: float | None = None            # filtered heading (rad)
    left_positions: list[float] | None = None    # per-motor encoder positions
    right_positions: list[float] | None = None
    cum_left: float | None = None           # cumulative avg position (inches)
    cum_right: float | None = None
    # Raw wall-reset sensor readings this tick, mm ("DISTREAD" line; -1 no sensor,
    # 9999 no object). Only in logs from firmware that logs them every tick.
    distance_mm: dict[str, int] | None = None
    # Tracking wheels this tick ("TRACKWHEEL" line, Odometry::update): vert/lat absolute
    # inches, dvert/dlat this tick's deltas (nan when unusable), ddrive the drive-encoder
    # forward delta, vok/lok 1 when the wheel was used. Absent from older logs.
    tracking_wheel: dict[str, float] | None = None

    def numeric_fields(self):
        for name in ("left_vel_setpoint", "left_vel_actual",
                     "right_vel_setpoint", "right_vel_actual",
                     "ramsete_v", "ramsete_w", "mp_v", "mp_w",
                     "ff_left", "ff_right", "fb_left", "fb_right"):
            v = getattr(self, name)
            if v is not None:
                yield name, v
        for name in ("pose", "goal"):
            v = getattr(self, name)
            if v is not None:
                for i, x in enumerate(v):
                    yield f"{name}[{i}]", x


@dataclass
class DistanceReset:
    t: float
    tick_index: int          # index into run.ticks of the last tick before reset
    sensor_reading_in: float | None = None
    reset_axis: str | None = None    # 'x' or 'y', from the "resettingX:" line
    correction_in: float | None = None   # what the firmware computed (applied or not)
    applied: bool = False            # False when a gate after the correction line refused it


@dataclass
class Run:
    """One Robot::followTrajectory() invocation."""
    t_start: float
    label: str = ""              # e.g. "s3r0" (session 3, run 0 within it)
    initial_pose: tuple[float, float, float] | None = None
    route_size: int | None = None
    # Constants the firmware was actually built with, from the "Build config:"
    # line (wheelBase, wheelDia, kS, kV, kA, kP). Empty for logs recorded before
    # that line was added. Without it there is no way to know which constants a
    # log ran with, so a mismatch against today's config.hpp is unattributable.
    build_config: dict[str, float] = field(default_factory=dict)
    # Route the run followed: the last "trajectory '<folder/name>' loaded" before it
    # (initialize or the route selector). None for logs from before that line existed.
    route: str | None = None
    ticks: list[ControlTick] = field(default_factory=list)
    actions: list[float] = field(default_factory=list)   # timestamps of action points
    resets: list[DistanceReset] = field(default_factory=list)
    warnings: list[tuple[float, str]] = field(default_factory=list)
    errors: list[tuple[float, str]] = field(default_factory=list)
    completed: bool = False


@dataclass
class Session:
    """One boot of the robot ("Robot initialized" ... next one)."""
    t_start: float
    runs: list[Run] = field(default_factory=list)
    warnings: list[tuple[float, str]] = field(default_factory=list)
    errors: list[tuple[float, str]] = field(default_factory=list)


_PATTERNS = {
    "pose": re.compile(rf"^Pose: ({FLOAT}) ({FLOAT}) ({FLOAT})$"),
    "goal": re.compile(rf"^Goal: ({FLOAT}) ({FLOAT}) ({FLOAT})$"),
    "lwv": re.compile(rf"^Left wheel velocities: ({FLOAT}) ({FLOAT})$"),
    "rwv": re.compile(rf"^Right wheel velocities: ({FLOAT}) ({FLOAT})$"),
    "volts": re.compile(r"^Voltages: (-?\d+) (-?\d+)$"),
    "ramsete": re.compile(rf"^Ramsete Output: ({FLOAT}) ({FLOAT})$"),
    "mp": re.compile(rf"^MP Output: ({FLOAT}) ({FLOAT})$"),
    "ff": re.compile(rf"^Left Feedforward: ({FLOAT}), Right Feedforward: ({FLOAT})$"),
    "fb": re.compile(rf"^Left Feedback: ({FLOAT}), Right Feedback: ({FLOAT})$"),
    "dtheta": re.compile(rf"^Delta theta: ({FLOAT})$"),
    "imu": re.compile(rf"^IMU Heading: ({FLOAT})$"),
    "initial_pose": re.compile(rf"^Initial pose: ({FLOAT}) ({FLOAT}) ({FLOAT})$"),
    "route_size": re.compile(r"^Route size: (\d+)$"),
    "action": re.compile(r"^Executing action with (\d+) parameters$"),
    "sensor_reading": re.compile(rf"^Distance sensor reading: ({FLOAT}) in$"),
    "heading": re.compile(rf"^Heading: ({FLOAT})$"),
    "positions_l": re.compile(rf"^Left Positions: ((?:{FLOAT}[ ]?)+)$"),
    "positions_r": re.compile(rf"^Right Positions: ((?:{FLOAT}[ ]?)+)$"),
    "cum_pos": re.compile(rf"^Left: ({FLOAT}), Right: ({FLOAT})$"),
    "build_config": re.compile(r"^Build config: (.*)$"),
    "route_loaded": re.compile(r"trajectory '([^']*)' loaded"),
    "trackwheel": re.compile(r"^TRACKWHEEL (.*)$"),
    "distread": re.compile(r"^DISTREAD front=(-?\d+) back=(-?\d+) left=(-?\d+) right=(-?\d+)$"),
    "reset_result": re.compile(
        rf"^resettingX: (\d), wallSign: {FLOAT}, wallToCenter: {FLOAT}, correction: ({FLOAT})$"),
}

_KV_PAIR = re.compile(rf"(\w+)=({FLOAT})")


def parse_log(path: str | Path) -> list[Session]:
    """Parse a raw log file into sessions -> runs -> control ticks."""
    sessions: list[Session] = []
    session: Session | None = None
    run: Run | None = None
    tick: ControlTick | None = None
    pending_reset = False
    pending_build: dict = {}
    pending_route: str | None = None
    # Odometry logs its lines (IMU Heading, Heading, positions, Delta theta)
    # BEFORE processTrajectory logs Pose for the same iteration, so buffer them
    # and attach to the tick the next Pose line creates.
    pending_odo: dict = {}

    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            m = LINE_RE.match(raw.strip())
            if not m:
                continue
            h, mnt, s, ms, level, msg = m.groups()
            t = int(h) * 3600 + int(mnt) * 60 + int(s) + int(ms) / 1000.0

            # Before the session check: initialize() loads the first route before it
            # logs "Robot initialized".
            pm = _PATTERNS["route_loaded"].search(msg)
            if pm:
                pending_route = pm.group(1)
                continue

            if msg == "Robot initialized":
                session = Session(t_start=t)
                sessions.append(session)
                run = None
                tick = None
                continue
            if session is None:
                continue

            if level == "WARNING":
                (run.warnings if run else session.warnings).append((t, msg))
                # The one gate after the "resettingX:" line
                if run and run.resets and msg.startswith("Distance reset rejected:") \
                        and "correction" in msg:
                    run.resets[-1].applied = False
                continue
            if level == "ERROR":
                (run.errors if run else session.errors).append((t, msg))
                continue

            # Logged just before "Initial pose", i.e. before the Run exists.
            pm = _PATTERNS["build_config"].match(msg)
            if pm:
                pending_build = {k: parse_float(v)
                                 for k, v in _KV_PAIR.findall(pm.group(1))}
                continue

            pm = _PATTERNS["initial_pose"].match(msg)
            if pm:
                run = Run(t_start=t,
                          label=f"s{len(sessions)-1}r{len(session.runs)}",
                          initial_pose=tuple(parse_float(x) for x in pm.groups()),
                          build_config=dict(pending_build),
                          route=pending_route)
                session.runs.append(run)
                tick = None
                pending_odo = {}
                continue

            if run is None:
                continue

            pm = _PATTERNS["route_size"].match(msg)
            if pm:
                run.route_size = int(pm.group(1))
                continue

            if msg == "Trajectory following completed":
                run.completed = True
                continue

            # Odometry-phase lines: buffer for the upcoming tick
            pm = _PATTERNS["distread"].match(msg)
            if pm:
                pending_odo["distance_mm"] = dict(
                    zip(("front", "back", "left", "right"), (int(x) for x in pm.groups())))
                continue
            pm = _PATTERNS["trackwheel"].match(msg)
            if pm:
                pending_odo["tracking_wheel"] = {
                    k: parse_float(v) for k, v in _KV_PAIR.findall(pm.group(1))}
                continue
            pm = _PATTERNS["heading"].match(msg)
            if pm:
                pending_odo["heading"] = parse_float(pm.group(1))
                continue
            pm = _PATTERNS["imu"].match(msg)
            if pm:
                pending_odo["imu_heading"] = parse_float(pm.group(1))
                continue
            pm = _PATTERNS["dtheta"].match(msg)
            if pm:
                pending_odo["delta_theta"] = parse_float(pm.group(1))
                continue
            pm = _PATTERNS["positions_l"].match(msg)
            if pm:
                pending_odo["left_positions"] = [parse_float(x) for x in pm.group(1).split()]
                continue
            pm = _PATTERNS["positions_r"].match(msg)
            if pm:
                pending_odo["right_positions"] = [parse_float(x) for x in pm.group(1).split()]
                continue
            pm = _PATTERNS["cum_pos"].match(msg)
            if pm:
                pending_odo["cum_left"] = parse_float(pm.group(1))
                pending_odo["cum_right"] = parse_float(pm.group(2))
                continue

            pm = _PATTERNS["pose"].match(msg)
            if pm:
                tick = ControlTick(t=t, pose=tuple(parse_float(x) for x in pm.groups()),
                                   **pending_odo)
                pending_odo = {}
                run.ticks.append(tick)
                continue

            pm = _PATTERNS["action"].match(msg)
            if pm:
                run.actions.append(t)
                continue

            if msg == "DISTANCE RESET":
                run.resets.append(DistanceReset(t=t, tick_index=len(run.ticks) - 1))
                pending_reset = True
                continue

            pm = _PATTERNS["reset_result"].match(msg)
            if pm and run.resets:
                run.resets[-1].reset_axis = "x" if pm.group(1) == "1" else "y"
                run.resets[-1].correction_in = parse_float(pm.group(2))
                # Only the max-correction gate follows this line, and it warns.
                run.resets[-1].applied = True
                continue

            pm = _PATTERNS["sensor_reading"].match(msg)
            if pm and pending_reset and run.resets:
                run.resets[-1].sensor_reading_in = parse_float(pm.group(1))
                pending_reset = False
                continue

            if tick is None:
                continue

            for key, attrs in (
                ("goal", ("goal",)),
                ("lwv", ("left_vel_setpoint", "left_vel_actual")),
                ("rwv", ("right_vel_setpoint", "right_vel_actual")),
                ("ramsete", ("ramsete_v", "ramsete_w")),
                ("mp", ("mp_v", "mp_w")),
                ("ff", ("ff_left", "ff_right")),
                ("fb", ("fb_left", "fb_right")),
            ):
                pm = _PATTERNS[key].match(msg)
                if pm:
                    vals = [parse_float(x) for x in pm.groups()]
                    if attrs == ("goal",):
                        tick.goal = tuple(vals)
                    else:
                        for attr, val in zip(attrs, vals):
                            setattr(tick, attr, val)
                    break
            else:
                pm = _PATTERNS["volts"].match(msg)
                if pm:
                    tick.voltage_left = int(pm.group(1))
                    tick.voltage_right = int(pm.group(2))

    return sessions


def trajectory_runs(sessions: list[Session], min_ticks: int = 20) -> list[Run]:
    """Runs with enough control ticks to analyze meaningfully."""
    return [r for s in sessions for r in s.runs if len(r.ticks) >= min_ticks]


def select_runs(sessions: list[Session], session: int | None = None,
                run: int | str | None = None, min_ticks: int = 20) -> list[Run]:
    """Resolve --session/--run CLI selectors to a list of runs.

    session: index into the file's sessions (None = all sessions).
    run: index into the selected runs ('all' or None = all).
    """
    if session is not None:
        if session >= len(sessions):
            raise IndexError(f"session {session} out of range (file has {len(sessions)})")
        pool = [r for r in sessions[session].runs if len(r.ticks) >= min_ticks]
    else:
        pool = trajectory_runs(sessions, min_ticks)
    if run is None or run == "all":
        return pool
    idx = int(run)
    if idx >= len(pool):
        raise IndexError(f"run {idx} out of range ({len(pool)} runs selected)")
    return [pool[idx]]
