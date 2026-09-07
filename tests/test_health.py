"""Health checks for the drive motor health stream (DRIVEHEALTH).

Two layers, same shape as test_lift.py:
  * Parser round-trips on synthetic lines, so the format is verified even before
    a log carries the newest fields. The format IS the contract — if
    Robot::logDriveHealth / logSideHealth changes, these fail loudly.
  * Invariants over any real log in tests/logs/ that carries DRIVEHEALTH
    samples. Logs without it are skipped.

The headline invariant is test_partner_currents_balanced. The two motors on a
side are mechanically coupled — same shaft speed, same command — so under a
shared load their current draw should track. A sustained split means one motor
is not carrying its share, and the message names the port to pull. That is a
hardware alarm in the same spirit as test_drivetrain_sides_symmetric: a failure
here is evidence, not noise.

Run:  pytest tests/test_health.py -v
"""

from __future__ import annotations

import math
import statistics
from pathlib import Path

import pytest

from log_analysis.health_parser import (
    parse_health_log,
    health_sessions_with_data,
    all_partner_imbalances,
)
from log_analysis.robot_config import DRIVE_PORTS

LOG_DIR = Path(__file__).parent / "logs"

# A V5 motor's steady current limit is 2.5 A; transients run higher (3.3 A
# appears in 08012026-9). Anything past this is a bad reading, not a motor.
MAX_PLAUSIBLE_MA = 5000.0

# V5 motors live around 30-55 C; the firmware derates in steps above ~55 C.
MAX_PLAUSIBLE_TEMP_C = 100.0

# The V5 raises its over-temp flag around 55 C. Assert well below that, so this
# only fires on a genuinely inconsistent pair of readings rather than on
# rounding at the threshold.
OVER_TEMP_MIN_C = 45.0

# Load-sharing split, measured only over samples where BOTH motors are engaged.
# Healthy pairs sit at 1.00-1.10 (08012026-10: left x1.05, right x1.10).
MAX_PARTNER_CURRENT_RATIO = 1.4

# Share of loaded samples where one motor sits at ~0 mA while its partner pulls.
# Healthy pairs are a couple of percent of transition samples (08012026-10 left
# is 1.6%); port 15 in the same log is 32%, entirely at stall.
MAX_CUTOUT_FRACTION = 0.10

SYNTHETIC_LOG = """\
[00:00:00.000] [INFO] Logger initialized successfully
[00:00:02.100] [LOG] DRIVEHEALTH auton L0 port=-18 tempC=35 curMA=810 W=5.2 eff=71 ovt=0
[00:00:02.100] [LOG] DRIVEHEALTH auton L1 port=19 tempC=36 curMA=888 W=5.6 eff=68 ovt=0
[00:00:02.100] [LOG] DRIVEHEALTH auton R0 port=15 tempC=35 curMA=489 W=3.1 eff=75 ovt=0
[00:00:02.100] [LOG] DRIVEHEALTH auton R1 port=-9 tempC=57 curMA=764 W=4.8 eff=70 ovt=1
[00:00:02.200] [LOG] DRIVEHEALTH driver L0 port=-18 tempC=-- curMA=-- W=-- eff=-- ovt=--
"""

LEGACY_LOG = """\
[00:00:00.000] [INFO] Logger initialized successfully
[00:00:02.192] [LOG] DRIVEHEALTH auton | L tempC=35,35 curMA=810,888 ovt=0,0 | R tempC=35,36 curMA=489,764 ovt=0,1
[00:00:02.292] [LOG] DRIVEHEALTH auton | L tempC=35,35 curMA=900,910 W=5.2,5.6 eff=71,68 ovt=0,0 | R tempC=35,36 curMA=500,700 W=3.1,4.8 eff=75,70 ovt=0,0
"""


# ------------------------------------------------------------------ parser

def test_parser_roundtrip(tmp_path):
    """The parser must recover every field of the per-motor DRIVEHEALTH format."""
    p = tmp_path / "synthetic.txt"
    p.write_text(SYNTHETIC_LOG)
    sessions = parse_health_log(p)
    assert len(sessions) == 1
    s = sessions[0]

    assert len(s.samples) == 5
    assert s.motors == [("L", 0), ("L", 1), ("R", 0), ("R", 1)]
    assert s.has_power and s.has_efficiency

    l0 = s.by_motor("L", 0)[0]
    assert l0.context == "auton" and l0.port == -18
    assert l0.temp_c == 35.0 and l0.current_ma == 810.0
    assert l0.power_w == 5.2 and l0.efficiency_pct == 71.0
    assert l0.over_temp is False
    # volts seen = P/I, the "is this motor receiving its command" discriminator
    assert abs(l0.volts - 5.2 / 0.810) < 1e-6

    r1 = s.by_motor("R", 1)[0]
    assert r1.port == -9 and r1.over_temp is True and r1.temp_c == 57.0

    assert s.ports() == {("L", 0): -18, ("L", 1): 19, ("R", 0): 15, ("R", 1): -9}
    assert {x.context for x in s.samples} == {"auton", "driver"}


def test_unreadable_channels_parse_as_nan(tmp_path):
    """"--" is what logDriveHealth prints for PROS_ERR_F/PROS_ERR; it must not
    become a number. A dropped motor has to stay visibly absent."""
    p = tmp_path / "synthetic.txt"
    p.write_text(SYNTHETIC_LOG)
    s = parse_health_log(p)[0]
    dead = [x for x in s.by_motor("L", 0) if x.context == "driver"][0]
    assert math.isnan(dead.temp_c) and math.isnan(dead.current_ma)
    assert math.isnan(dead.power_w) and math.isnan(dead.efficiency_pct)
    assert math.isnan(dead.volts)
    assert dead.over_temp is False


def test_parser_reads_legacy_per_side_format(tmp_path):
    """Logs captured before the per-motor split still parse, so they stay
    analysable. W=/eff= were added later again, hence optional."""
    p = tmp_path / "legacy.txt"
    p.write_text(LEGACY_LOG)
    s = parse_health_log(p)[0]

    assert len(s.samples) == 8          # 2 lines x 4 motors
    assert s.motors == [("L", 0), ("L", 1), ("R", 0), ("R", 1)]

    first_l1 = s.by_motor("L", 1)[0]
    assert first_l1.current_ma == 888.0 and first_l1.temp_c == 35.0
    assert first_l1.power_w is None and first_l1.efficiency_pct is None
    assert s.by_motor("R", 1)[0].over_temp is True

    second_l0 = s.by_motor("L", 0)[1]
    assert second_l0.power_w == 5.2 and second_l0.efficiency_pct == 71.0

    # No port= column in the legacy format, so ports come from main.cpp wiring.
    assert s.ports() == {("L", 0): DRIVE_PORTS["L"][0], ("L", 1): DRIVE_PORTS["L"][1],
                         ("R", 0): DRIVE_PORTS["R"][0], ("R", 1): DRIVE_PORTS["R"][1]}


# --------------------------------------------------- invariants over real logs

def _health_logs():
    if not LOG_DIR.exists():
        return []
    out = []
    for path in sorted(LOG_DIR.glob("*.txt")) + sorted(LOG_DIR.glob("*.log")):
        try:
            if health_sessions_with_data(parse_health_log(path)):
                out.append(path.name)
        except OSError:
            continue
    return out


_HEALTH_LOGS = _health_logs()


@pytest.fixture(params=_HEALTH_LOGS or [pytest.param("<none>", marks=pytest.mark.skip(
    reason="no log in tests/logs/ contains DRIVEHEALTH samples yet"))],
                ids=lambda n: n)
def health(request):
    sessions = health_sessions_with_data(parse_health_log(LOG_DIR / request.param))
    samples = [s for sess in sessions for s in sess.samples]
    return request.param, sessions, samples


def test_no_unreadable_channels(health):
    """A "--" reading means PROS could not reach that motor on its smart port.

    Occasional dropouts are a wiring/connector problem, not a logging one, so
    this is an alarm rather than a parser check.
    """
    name, _sessions, samples = health
    bad = [s for s in samples if not math.isfinite(s.temp_c) or not math.isfinite(s.current_ma)]
    if bad:
        ports = sorted({f"{s.label}" for s in bad})
        pytest.fail(f"{name}: {len(bad)}/{len(samples)} unreadable health readings "
                    f"on {', '.join(ports)} — the motor dropped off the smart port")


def test_temperature_plausible(health):
    """Temperatures must be physical; a wild value means a bad device read."""
    name, _sessions, samples = health
    for s in samples:
        if math.isfinite(s.temp_c):
            assert 0.0 <= s.temp_c <= MAX_PLAUSIBLE_TEMP_C, (
                f"{name}: {s.label} reported {s.temp_c} C at t={s.t:.3f}")


def test_current_within_plausible_range(health):
    """Current must be non-negative and inside what a V5 motor can pull."""
    name, _sessions, samples = health
    for s in samples:
        if math.isfinite(s.current_ma):
            assert -1.0 <= s.current_ma <= MAX_PLAUSIBLE_MA, (
                f"{name}: {s.label} drew {s.current_ma} mA at t={s.t:.3f}")


def test_efficiency_in_range(health):
    """Efficiency is a percentage. Logs predating the eff= column are skipped."""
    name, sessions, samples = health
    if not any(sess.has_efficiency for sess in sessions):
        pytest.skip(f"{name}: log predates the eff= column")
    for s in samples:
        if s.efficiency_pct is not None and math.isfinite(s.efficiency_pct):
            assert -0.1 <= s.efficiency_pct <= 100.1, (
                f"{name}: {s.label} reported {s.efficiency_pct}% efficiency at t={s.t:.3f}")


def test_over_temp_flag_agrees_with_temperature(health):
    """A raised over-temp flag must be backed by a hot reading.

    The two come from separate device fields in the same sample; disagreement
    means the columns drifted out of order in logSideHealth.
    """
    name, _sessions, samples = health
    for s in samples:
        if s.over_temp and math.isfinite(s.temp_c):
            assert s.temp_c >= OVER_TEMP_MIN_C, (
                f"{name}: {s.label} flagged over-temp at only {s.temp_c} C "
                "(check the field order in Robot::logSideHealth)")


def test_partner_currents_balanced(health):
    """While both motors are engaged, the two on a side must share load evenly.

    They are mechanically coupled, so they run at the same speed under the same
    command — the healthy one is a built-in control for the sick one, and no
    baseline run is needed. A sustained split while both are drawing means one
    is weaker than the other: a failing winding, or a coupling letting it slip.
    """
    name, sessions, _samples = health
    problems = []
    for sess in sessions:
        for pi in all_partner_imbalances(sess):
            if pi.n_engaged >= 10 and pi.current_ratio > MAX_PARTNER_CURRENT_RATIO:
                problems.append(
                    f"boot #{sess.index} {pi.side} pair: x{pi.current_ratio:.2f} split over "
                    f"{pi.n_engaged} engaged samples; lower draw = port {pi.worst_port}")
    assert not problems, (
        f"{name}: drive motors on a side are not sharing load —\n  "
        + "\n  ".join(problems)
        + f"\n  (healthy pairs measure 1.00-1.10; threshold {MAX_PARTNER_CURRENT_RATIO}). "
          "Inspect that port's motor and its coupling.")


def test_no_motor_cuts_out_under_load(health):
    """No motor may sit at ~0 mA while its coupled partner is pulling.

    Distinct from the sharing ratio above, and the sharper alarm: a motor that
    drives normally but contributes nothing in some regime is failing under a
    specific stress rather than being uniformly weak. In 08012026-9/-10 port 15
    does this in 32-38% of loaded samples, all of them at STALL — where back-EMF
    is zero and current draw peaks — while driving normally the rest of the time.
    That points at the motor cutting out under peak electrical stress (internal
    protection, a marginal connector, or a failing winding), not at wear.
    """
    name, sessions, _samples = health
    problems = []
    for sess in sessions:
        for pi in all_partner_imbalances(sess):
            if pi.cutout_frac > MAX_CUTOUT_FRACTION:
                problems.append(
                    f"boot #{sess.index} {pi.side} pair: port {pi.cutout_port} at ~0 mA in "
                    f"{pi.cutout_frac * 100:.0f}% of {pi.n} loaded samples while its partner pulled")
    assert not problems, (
        f"{name}: a drive motor stops contributing under load —\n  "
        + "\n  ".join(problems)
        + f"\n  (healthy pairs stay under {MAX_CUTOUT_FRACTION * 100:.0f}%). Swap that motor "
          "and its cable to confirm which one follows the fault.")


def test_ports_match_wiring(health):
    """Logged ports must match the MotorGroups in src/main.cpp.

    Guards the index -> physical motor mapping the whole diagnosis rests on: if
    they disagree, either the robot was rewired without updating main.cpp, or
    the analysis is naming the wrong motor.
    """
    name, sessions, _samples = health
    for sess in sessions:
        for (side, idx), port in sess.ports().items():
            logged = [s.port for s in sess.by_motor(side, idx) if s.port is not None]
            if not logged:
                continue                      # legacy log, no port column
            expected = DRIVE_PORTS.get(side, [])
            assert idx < len(expected) and logged[0] == expected[idx], (
                f"{name}: {side}{idx} logged port {logged[0]}, but src/main.cpp wires "
                f"{side} as {expected}")


def test_sampling_period_matches_firmware(health):
    """Samples must arrive at the throttle rate, not the control-loop rate.

    logDriveHealth self-throttles to Robot::DRIVE_HEALTH_LOG_MS (100 ms). A
    median much below that means the gate broke and the health stream is now
    costing smart-port reads on every 10 ms tick.
    """
    name, sessions, _samples = health
    for sess in sessions:
        for side, idx in sess.motors:
            ts = [s.t for s in sess.by_motor(side, idx)]
            if len(ts) < 10:
                continue
            gaps = [b - a for a, b in zip(ts, ts[1:]) if b > a]
            if not gaps:
                continue
            median_ms = statistics.median(gaps) * 1000.0
            assert median_ms >= 50.0, (
                f"{name}: {side}{idx} sampled every {median_ms:.0f} ms — the "
                "DRIVE_HEALTH_LOG_MS throttle in Robot::logDriveHealth is not holding")


def test_timestamps_monotonic_within_session(health):
    """Samples within one boot must be time-ordered (sane parse/clock)."""
    name, sessions, _samples = health
    for sess in sessions:
        for side, idx in sess.motors:
            ts = [s.t for s in sess.by_motor(side, idx)]
            assert ts == sorted(ts), (
                f"{name}: boot #{sess.index} {side}{idx} samples out of time order")
