"""Single source of truth for robot constants, parsed from include/config.hpp.

The analysis code used to hardcode its own copies of these (track width 14.0,
a magic 74.0 in/s wheel-speed ceiling). Both had silently drifted away from
config.hpp, which made the suite lie in both directions: it raised a false
track-width alarm, and it passed setpoints that the drivetrain could not
actually deliver. Parse the header instead so the constants cannot go stale.

Only plain numeric-literal constants are parsed. WHEEL_DIAMETER is defined via
Units::Constants and is resolved from a small lookup below.
"""

from __future__ import annotations

import math
import re
from pathlib import Path

_INCLUDE = Path(__file__).resolve().parents[2] / "include"
_CONFIG_HPP = _INCLUDE / "config.hpp"
_UNITS_HPP = _INCLUDE / "utilities" / "math" / "units.hpp"

# Fallbacks for the Units::Constants names config.hpp refers to. These are parsed
# out of units.hpp below; the literals here are only used if it can't be read.
# Note WHEEL_2_75_INCH is 2.76, not 2.75 -- it's the measured effective diameter,
# so do not "correct" it to match its own name.
_NAMED = {
    "WHEEL_2_75_INCH": 2.76,
    "WHEEL_3_25_INCH": 3.18,
    "WHEEL_4_INCH": 4.0,
    "V5_BLUE_MOTOR_TICKS_PER_ROTATION": 300.0,
}

# Free speed of the motor cartridge at the gearbox, in RPM. The blue cartridge
# is 600 rpm; config.hpp selects it via V5_BLUE_MOTOR_TICKS_PER_ROTATION.
BLUE_CARTRIDGE_RPM = 600.0

# Fallbacks used only if config.hpp cannot be read/parsed (keeps tests runnable
# from a bare checkout of tests/).
_DEFAULTS = {
    "WHEEL_BASE_WIDTH": 13.5,
    "GEAR_RATIO": 36.0 / 48.0,
    "WHEEL_DIAMETER": 2.76,
    "KS": 2.55, "KV": 2.0, "KA": 0.4,
    "KP": 3.0, "KI": 0.0, "KD": 0.0,
    "RAMSETE_B": 2.4, "RAMSETE_ZETA": 0.4,
    "DT": 10.0,
}

_DECL = re.compile(
    r"constexpr\s+(?:double|bool|int)\s+(\w+)\s*=\s*([^;]+);")


def _evaluate(expr: str) -> float | None:
    """Evaluate a simple constexpr initializer (literals, named constants,
    and * / + - between them). Returns None if it isn't purely numeric."""
    expr = expr.split("///")[0].split("//")[0].strip()
    expr = expr.replace("M_PI", str(math.pi))
    for name, val in _NAMED.items():
        expr = expr.replace(f"Units::Constants::{name}", str(val))
    if not re.fullmatch(r"[-+*/(). 0-9eE]+", expr):
        return None
    try:
        return float(eval(expr, {"__builtins__": {}}, {}))  # noqa: S307
    except (SyntaxError, NameError, ZeroDivisionError, TypeError):
        return None


def _load_named() -> dict[str, float]:
    """Resolve Units::Constants names from units.hpp so they cannot drift."""
    named = dict(_NAMED)
    try:
        text = _UNITS_HPP.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return named
    for name, expr in _DECL.findall(text):
        if name in named:
            val = _evaluate(expr)
            if val is not None:
                named[name] = val
    return named


def _load() -> dict[str, float]:
    values = dict(_DEFAULTS)
    _NAMED.update(_load_named())
    try:
        text = _CONFIG_HPP.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return values
    resolved: dict[str, float] = {}
    for name, expr in _DECL.findall(text):
        # Allow later constants to reference earlier ones (e.g. WHEEL_CIRCUMFERENCE).
        for prev, val in resolved.items():
            expr = re.sub(rf"\b{prev}\b", str(val), expr)
        val = _evaluate(expr)
        if val is not None:
            resolved[name] = val
    values.update(resolved)
    return values


_C = _load()

TRACK_WIDTH = _C["WHEEL_BASE_WIDTH"]          # Config::WHEEL_BASE_WIDTH (in)
WHEEL_DIAMETER = _C["WHEEL_DIAMETER"]
GEAR_RATIO = _C["GEAR_RATIO"]
KS, KV, KA = _C["KS"], _C["KV"], _C["KA"]
KP, KI, KD = _C["KP"], _C["KI"], _C["KD"]
DT_MS = _C["DT"]
DT_NOMINAL = DT_MS / 1000.0

#: Encoder counts per motor revolution (Config::TICKS_PER_ROTATION).
TICKS_PER_ROTATION = _C.get("TICKS_PER_ROTATION", 300.0)

#: Wheel travel per raw encoder count, in inches. This is the same conversion
#: Odometry::ticksToInches applies, so per-motor 'Left/Right Positions' columns
#: can be compared against the averaged 'Left:/Right:' inches the firmware logs.
INCHES_PER_TICK = (GEAR_RATIO * WHEEL_DIAMETER * math.pi / TICKS_PER_ROTATION)

#: Voltage command range of a V5 motor (DrivetrainController::MOTOR_VOLTAGE_LIMIT).
MOTOR_VOLTAGE_LIMIT = 127

#: Free (no-load) wheel speed implied by the configured gearing, in in/s.
#: 600 rpm blue cartridge * 36/48 * pi * 2.76" / 60 ~= 65.0 in/s.
#: Prefers Config::FREE_WHEEL_VELOCITY when the firmware defines it.
FREE_WHEEL_SPEED = _C.get(
    "FREE_WHEEL_VELOCITY",
    BLUE_CARTRIDGE_RPM * GEAR_RATIO * WHEEL_DIAMETER * math.pi / 60.0)

#: A measured wheel speed above this is not physically possible and indicates a
#: bad velocity computation (e.g. the Odometry deltaTime fallback dividing a
#: long encoder delta by the nominal 10 ms). 10% over free speed for noise.
UNPHYSICAL_WHEEL_SPEED = FREE_WHEEL_SPEED * 1.10

#: The speed a *loaded* drivetrain can actually hold, and the budget the firmware
#: desaturates wheel setpoints to (Config::MAX_WHEEL_VELOCITY). Free speed is not
#: achievable under load; commanding above this saturates the motor and leaves no
#: authority for feedback. Measured ceiling in initial_tuning.txt was ~56 in/s.
#:
#: Read from config.hpp so the suite judges logs against the same budget the robot
#: actually enforced, rather than a second opinion that can drift from it.
USABLE_SPEED_FRACTION = _C.get("USABLE_SPEED_FRACTION", 0.85)
USABLE_WHEEL_SPEED = _C.get(
    "MAX_WHEEL_VELOCITY", FREE_WHEEL_SPEED * USABLE_SPEED_FRACTION)


# --------------------------------------------------------------- motor ports

_MAIN_CPP = Path(__file__).resolve().parents[2] / "src" / "main.cpp"

#: Fallback drive ports, used only when main.cpp cannot be read.
_DEFAULT_DRIVE_PORTS = {"L": [-18, 19], "R": [15, -9]}

_MG_RE = re.compile(r"pros::MotorGroup\s+(\w+)\s*\(\s*\{([^}]*)\}")


def _load_drive_ports() -> dict[str, list[int]]:
    """Signed smart ports of each drive side, parsed from main.cpp.

    Sign carries reversal (-18 = port 18 reversed), and ORDER matters: it is the
    index the firmware reports as L0/L1/R0/R1 and the same order the odometry
    stream's "Left Positions:" columns use. Parsed rather than hardcoded so a
    rewire cannot leave the analysis naming the wrong physical motor.
    """
    ports = {k: list(v) for k, v in _DEFAULT_DRIVE_PORTS.items()}
    try:
        text = _MAIN_CPP.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ports
    for name, body in _MG_RE.findall(text):
        side = {"left_mg": "L", "right_mg": "R"}.get(name)
        if side is None:
            continue
        found = [int(p) for p in re.findall(r"-?\d+", body)]
        if found:
            ports[side] = found
    return ports


#: {"L": [-18, 19], "R": [15, -9]} — see _load_drive_ports().
DRIVE_PORTS = _load_drive_ports()


def drive_port(side: str, index: int) -> int | None:
    """The signed smart port for a side's Nth drive motor, or None if unknown."""
    seq = DRIVE_PORTS.get(side, [])
    return seq[index] if 0 <= index < len(seq) else None


# ------------------------------------------------------- wall-reset sensors

#: Beam direction of each wall-reset sensor relative to the robot's heading, degrees
#: CCW. Fixed by DistanceResetOdometry::resetFront/Back/Left/Right.
RESET_SENSOR_ANGLES = {"front": 0.0, "back": 180.0, "left": 90.0, "right": 270.0}

_SENSOR_OFFSET_RE = re.compile(
    r"constexpr\s+double\s+(FRONT|BACK|LEFT|RIGHT)_DISTANCE_(ALONG|ACROSS)\s*=\s*([^;]+);")


def _load_reset_sensors() -> dict[str, tuple[float, float, float]]:
    """{name: (beam angle deg, along in, across in)} for the sensors main.cpp defines.

    Offsets are the DistanceResetOdometry::DistanceSensor ones: ALONG the beam from the
    tracking centre to the face, ACROSS positive to the beam's right. Parsed from
    main.cpp so a re-measured mount cannot leave the analysis on stale geometry.
    """
    try:
        text = _MAIN_CPP.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return {}
    found: dict[str, dict[str, float]] = {}
    for name, kind, expr in _SENSOR_OFFSET_RE.findall(text):
        val = _evaluate(expr)
        if val is not None:
            found.setdefault(name.lower(), {})[kind] = val
    return {name: (RESET_SENSOR_ANGLES[name], d.get("ALONG", 0.0), d.get("ACROSS", 0.0))
            for name, d in found.items()}


#: See _load_reset_sensors(). Empty if main.cpp cannot be read.
RESET_SENSORS = _load_reset_sensors()

#: Config::DISTANCE_RESET_WALL_POSITION: field centre to wall face (in).
WALL_POSITION = _C.get("DISTANCE_RESET_WALL_POSITION", 70.25)

#: Config::LATERAL_WHEEL_OFFSET / VERTICAL_WHEEL_OFFSET: tracking wheels' offsets from the
#: pivot (in; lateral +forward, vertical +left). Checked by metrics.tracking_wheel_offsets.
LATERAL_WHEEL_OFFSET = _C.get("LATERAL_WHEEL_OFFSET", 0.0)
VERTICAL_WHEEL_OFFSET = _C.get("VERTICAL_WHEEL_OFFSET", 0.0)
#: Config::KS_ANGULAR / KV_ANGULAR: the turning-scrub feedforward metrics.spin_scrub fits.
KS_ANGULAR = _C.get("KS_ANGULAR", 0.0)
KV_ANGULAR = _C.get("KV_ANGULAR", 0.0)
#: Config::DISTANCE_RESET_MAX_BEAM_ANGLE_DEG: widest beam angle a reset accepts.
MAX_BEAM_ANGLE_DEG = _C.get("DISTANCE_RESET_MAX_BEAM_ANGLE_DEG", 20.0)
