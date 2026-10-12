"""Health checks for the roller mech debug stream (ROLLERDBG).

The roller mech is geared onto the intake, so a roller cycle and the intake turn the
same PTO motors, gears and roller. Given the same 12 V command they must reach the same
speed. The drivers saw the roller turn slower in a roller cycle than while intaking.
That was traced to the idle intake braking the motors every tick, just before
RollerMech drove them (fixed 2026-09-26 in Intake::update). These invariants are the
check that the fix worked.

Two layers, as in test_lift.py:
  * Parser and metric tests on synthetic lines (always run).
  * Invariants over real logs in tests/logs/ that carry ROLLERDBG ticks. Logs without
    them are skipped.

A failure on a real log is a finding about the robot, not a broken test. Do not loosen a
threshold to make a log pass.

Run:  pytest tests/test_roller.py -v
"""

from __future__ import annotations

from pathlib import Path

import pytest

from log_analysis.roller_parser import (
    parse_roller_log,
    roller_sessions_with_data,
    runs,
    steady_speeds,
    steady_ticks,
)

LOG_DIR = Path(__file__).parent / "logs"

# RollerMech::SPIN_MV / Intake::MAX_VOLTAGE
FULL_MV = 12000
# A full-voltage command must actually show up at the motors. Measured voltage sags a
# little under load, but a median under this means the motors are being handed
# something else (a brake) for a good share of the ticks.
MIN_APPLIED_FRACTION = 0.8
# Same motors, same gearing, same roller: a roller cycle must reach this fraction of
# the intake's steady speed.
MIN_ROLLER_VS_INTAKE = 0.85
# Enough steady ticks on each side for a median to mean something.
MIN_STEADY_TICKS = 10


def _line(t_ms: int, src: str, state: str, cmd: int, rpm: float, mv: int, ma: int,
          pos: float, dps: float) -> str:
    s, ms = divmod(t_ms, 1000)
    return (f"[00:00:{s:02d}.{ms:03d}] [LOG] ROLLERDBG src={src} state={state} cmd={cmd} "
            f"| PTO rpm={rpm:.0f},{rpm:.0f},{-rpm:.0f} mv={mv},{mv},{mv} ma={ma},{ma},{ma} "
            f"| AXLE pos={pos:.1f} dps={dps:.0f}")


def _synthetic_log() -> str:
    lines = ["[00:00:00.000] [INFO] Logger initialized successfully"]
    # Intake: 1 s at full voltage, 180 rpm after a 100 ms spin-up
    for i in range(100):
        t = 1000 + i * 10
        rpm = 180.0 if i >= 10 else 18.0 * i
        lines.append(_line(t, "INTAKE", "IDLE", 12000, rpm, 11800, 900, i * 4.0, 400))
    # Gap, then a roller cycle: 0.5 s SEEKING at 120 rpm, measured volts sagging
    for i in range(50):
        t = 3000 + i * 10
        lines.append(_line(t, "ROLLER", "SEEKING", 12000, 120.0, 7000, 2100, i * 3.0, 300))
    return "\n".join(lines) + "\n"


# ------------------------------------------------------------------ parser

def test_parser_roundtrip(tmp_path):
    p = tmp_path / "synthetic.txt"
    p.write_text(_synthetic_log())
    sessions = parse_roller_log(p)
    assert len(sessions) == 1
    ticks = sessions[0].ticks
    assert len(ticks) == 150

    tk = ticks[20]
    assert tk.src == "INTAKE" and tk.state == "IDLE" and tk.cmd == 12000
    assert tk.rpm == (180.0, 180.0, -180.0)
    assert tk.mv == (11800, 11800, 11800) and tk.ma == (900, 900, 900)
    assert tk.axle_pos == 80.0 and tk.axle_dps == 400.0
    assert tk.motor_rpm == 180.0  # |rpm| averaged, so a reversed motor does not cancel

    assert ticks[-1].src == "ROLLER" and ticks[-1].state == "SEEKING"


def test_runs_and_steady_window(tmp_path):
    p = tmp_path / "synthetic.txt"
    p.write_text(_synthetic_log())
    ticks = parse_roller_log(p)[0].ticks
    rs = runs(ticks)
    assert [r[0].src for r in rs] == ["INTAKE", "ROLLER"]
    # First 200 ms of each run are spin-up and do not count
    assert len(steady_ticks(ticks, "INTAKE")) == 80
    assert len(steady_ticks(ticks, "ROLLER")) == 30

    sp = steady_speeds(ticks)
    assert sp["INTAKE"].motor_rpm == 180.0 and sp["ROLLER"].motor_rpm == 120.0
    assert sp["ROLLER"].motor_mv == 7000.0 and sp["ROLLER"].axle_dps == 300.0


# --------------------------------------------------- invariants over real logs

def _roller_logs():
    if not LOG_DIR.exists():
        return []
    out = []
    for path in sorted(LOG_DIR.glob("*.txt")) + sorted(LOG_DIR.glob("*.log")):
        try:
            if roller_sessions_with_data(parse_roller_log(path)):
                out.append(path.name)
        except OSError:
            continue
    return out


_ROLLER_LOGS = _roller_logs()


@pytest.fixture(params=_ROLLER_LOGS or [pytest.param("<none>", marks=pytest.mark.skip(
    reason="no log in tests/logs/ contains ROLLERDBG samples yet"))],
                ids=lambda n: n)
def roller_sessions(request):
    return request.param, roller_sessions_with_data(parse_roller_log(LOG_DIR / request.param))


def test_roller_command_reaches_motors(roller_sessions):
    """A full-voltage roller cycle must show near-full voltage at the motors.

    A low median means something else commanded these motors in the same ticks. The
    known case: Intake::update braking the idle PTO every tick, just before
    RollerMech::update drove it.
    """
    name, sessions = roller_sessions
    for s in sessions:
        sp = steady_speeds(s.ticks).get("ROLLER")
        if sp is None or sp.ticks < MIN_STEADY_TICKS:
            continue
        assert sp.motor_mv >= MIN_APPLIED_FRACTION * FULL_MV, (
            f"{name}: boot #{s.index} roller cycles commanded {FULL_MV} mV but the PTO "
            f"motors measured a median {sp.motor_mv:.0f} mV over {sp.ticks} steady ticks. "
            "Something else is driving the motors in the same tick "
            "(see Intake::update / RollerMech::update ordering).")


def test_roller_as_fast_as_intake(roller_sessions):
    """A roller cycle must spin the shared motors about as fast as the intake does.

    They are geared together, so at the same command the only way the roller cycle is
    slower is a software difference in what the motors are sent.
    """
    name, sessions = roller_sessions
    for s in sessions:
        sp = steady_speeds(s.ticks)
        roller, intake = sp.get("ROLLER"), sp.get("INTAKE")
        if (roller is None or intake is None or roller.ticks < MIN_STEADY_TICKS
                or intake.ticks < MIN_STEADY_TICKS or intake.motor_rpm <= 0):
            continue
        ratio = roller.motor_rpm / intake.motor_rpm
        assert ratio >= MIN_ROLLER_VS_INTAKE, (
            f"{name}: boot #{s.index} roller cycles ran the PTO motors at "
            f"{roller.motor_rpm:.0f} rpm vs {intake.motor_rpm:.0f} rpm intaking "
            f"({ratio:.0%}, want >= {MIN_ROLLER_VS_INTAKE:.0%}); measured "
            f"{roller.motor_mv:.0f} vs {intake.motor_mv:.0f} mV")


def test_synthetic_slow_roller_is_caught(tmp_path):
    """The synthetic log's roller runs at 120/180 rpm on 7 V; both invariants must flag it."""
    p = tmp_path / "synthetic.txt"
    p.write_text(_synthetic_log())
    sp = steady_speeds(parse_roller_log(p)[0].ticks)
    assert sp["ROLLER"].motor_mv < MIN_APPLIED_FRACTION * FULL_MV
    assert sp["ROLLER"].motor_rpm / sp["INTAKE"].motor_rpm < MIN_ROLLER_VS_INTAKE
