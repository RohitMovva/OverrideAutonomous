"""Health checks for the lift debug stream (LIFTDBG).

Two layers:
  * A parser round-trip on a synthetic line, so the parser is verified even when no real
    log contains lift data yet. The format is the contract -- if Lift::log_debug_state
    changes, this fails loudly.
  * Invariants over any real logs in tests/logs/ that actually carry lift samples. Logs
    without lift data are skipped, so this stays green until a run on the new firmware
    is captured.

A failure on a real log is a finding about the robot, not a broken test. Do not loosen a
threshold to make a log pass; each assertion names the source location responsible.

Run:  pytest tests/test_lift.py -v
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from log_analysis.lift_parser import (
    parse_lift_log,
    lift_sessions_with_data,
)

LOG_DIR = Path(__file__).parent / "logs"

# Lift::MAX_VOLTAGE, and the V5 range move_voltage clamps to anyway.
V5_MV_LIMIT = 12000
# Lift::MAX_LOWER_MV -- gravity already helps, so the winch is never driven down harder.
MAX_LOWER_MV = 10000
# Lift::MAX_HEIGHT_DEG -- drive() cuts upward command at the cap.
MAX_HEIGHT_DEG = 2000.0
# Pto::SHIFT_TIME_MS, the window where own=0 is legitimate while the gears mesh.
SHIFT_TIME_MS = 150
# Lift::CLAW_SQUEEZE_MV -- what a closed claw keeps pushing with once it is shut.
CLAW_SQUEEZE_MV = 3000

SYNTHETIC_LOG = """\
[00:00:00.000] [INFO] Logger initialized successfully
[00:00:00.010] [LOG] Lift matchload macro: descending from 47.5 deg
[00:00:00.020] [LOG] Lift clamp -> CLEAR (MATCHLOAD)
[00:00:01.000] [LOG] LIFTDBG mode=MATCHLOAD state=ML_DESCEND pto=LIFT own=1 home=0 tower=9999 | WINCH tgt=500.0 act=204.7 err=295.3 mv=-10000 | ANG tgt=91.0 set=88.4 act=-6.2 err=97.2 mv=12000 | CLAMP tgt=CLEAR act=19.8 err=0.2 done=1
[00:00:02.000] [LOG] Lift matchload: loader seen (1 mm) -> closing clamp
[00:00:03.000] [LOG] LIFTDBG mode=PRIMED state=ML_CLAMP pto=LIFT own=1 home=1 tower=9999 loader=1 | WINCH tgt=500.0 act=2.0 err=498.0 mv=0 | ANG tgt=91.0 set=91.0 act=90.5 err=0.5 mv=-250 | CLAMP tgt=CLOSED act=5.2 err=-105.2 done=1
"""


# ------------------------------------------------------------------ parser

def test_parser_roundtrip(tmp_path):
    """The parser must recover every field of the LIFTDBG format, with and without
    the loader= field (logs from before the loader sensor lack it)."""
    p = tmp_path / "synthetic.txt"
    p.write_text(SYNTHETIC_LOG)
    sessions = parse_lift_log(p)
    assert len(sessions) == 1
    s = sessions[0]

    assert len(s.ticks) == 2
    assert len(s.events) == 3
    assert len(s.macro_events) == 1   # only the "matchload macro:" line says "macro"
    assert len(s.clamp_events) == 1

    t0 = s.ticks[0]
    assert t0.mode == "MATCHLOAD" and t0.state == "ML_DESCEND"
    assert t0.pto == "LIFT" and t0.own is True and t0.home is False
    assert t0.tower == 9999 and t0.loader is None
    assert t0.winch_tgt == 500.0 and t0.winch_act == 204.7 and t0.winch_err == 295.3
    assert t0.winch_mv == -10000
    assert t0.ang_tgt == 91.0 and t0.ang_set == 88.4 and t0.ang_act == -6.2
    assert t0.ang_err == 97.2 and t0.ang_mv == 12000
    assert t0.clamp_tgt == "CLEAR" and t0.clamp_act == 19.8 and t0.clamp_done is True

    t1 = s.ticks[1]
    assert t1.mode == "PRIMED" and t1.state == "ML_CLAMP" and t1.home is True
    assert t1.tower == 9999 and t1.loader == 1 and t1.winch_mv == 0 and t1.clamp_tgt == "CLOSED"

    assert s.events[0].text.startswith("matchload macro")
    assert s.clamp_events[0].text == "clamp -> CLEAR (MATCHLOAD)"


ROLLER_LOG = """\
[00:00:00.000] [INFO] Logger initialized successfully
[00:00:00.010] [LOG] Lift matchload macro: moving 0.0 -> 100.0 deg
[00:00:00.020] [LOG] Lift roller -> ON (MATCHLOAD)
[00:00:01.000] [LOG] LIFTDBG mode=MATCHLOAD state=ML_MOVE pto=LIFT own=1 home=0 tower=9999 loader=9999 | WINCH tgt=100.0 act=60.0 err=40.0 mv=6000 | ANG tgt=135.0 set=120.0 act=110.0 err=25.0 mv=3750 | ROLLER on=1 mv=12000
"""


def test_parser_roller_format(tmp_path):
    """Roller-claw logs replace the CLAMP section with ROLLER; the clamp fields are None."""
    p = tmp_path / "roller.txt"
    p.write_text(ROLLER_LOG)
    s = parse_lift_log(p)[0]
    assert len(s.ticks) == 1 and len(s.roller_events) == 1 and not s.clamp_events
    tk = s.ticks[0]
    assert tk.state == "ML_MOVE" and tk.loader == 9999 and tk.ang_tgt == 135.0
    assert tk.roller_on is True and tk.roller_mv == 12000
    assert tk.clamp_tgt is None and tk.clamp_act is None and tk.clamp_done is None
    assert tk.roller_rpm is None and tk.roller_ma is None and tk.roller_w is None
    assert tk.cycle is None
    assert s.roller_events[0].text == "roller -> ON (MATCHLOAD)"


CYCLE_LOG = """\
[00:00:00.000] [INFO] Logger initialized successfully
[00:00:00.010] [LOG] Lift intake cycle started (mode LOADING, 0.0 deg)
[00:00:01.000] [LOG] LIFTDBG mode=LOADING state=IDLE pto=INTAKE own=0 home=1 tower=9999 loader=9999 | WINCH tgt=0.0 act=0.4 err=-0.4 mv=0 | ANG tgt=-8.0 set=-8.0 act=-7.5 err=-0.5 mv=-120 | ROLLER on=1 mv=12000 rpm=-35 ma=1840 w=4.6 cyc=1
"""


def test_parser_roller_telemetry(tmp_path):
    """Newer roller ticks carry measured rpm / mA and the intake cycle flag."""
    p = tmp_path / "cycle.txt"
    p.write_text(CYCLE_LOG)
    s = parse_lift_log(p)[0]
    tk = s.ticks[0]
    assert tk.roller_on is True and tk.roller_mv == 12000
    assert tk.roller_rpm == -35.0 and tk.roller_ma == 1840 and tk.roller_w == 4.6
    assert tk.cycle is True
    assert s.events[0].text.startswith("intake cycle started")


def test_parser_roller_telemetry_without_cycle(tmp_path):
    """Once the intake cycle was removed, roller ticks keep rpm / mA / W but drop cyc=."""
    p = tmp_path / "nocycle.txt"
    p.write_text(CYCLE_LOG.replace(" cyc=1", "").replace("mode=LOADING state=IDLE",
                                                          "mode=MATCHLOAD state=ML_RAISE"))
    tk = parse_lift_log(p)[0].ticks[0]
    assert tk.state == "ML_RAISE"
    assert tk.roller_rpm == -35.0 and tk.roller_ma == 1840 and tk.roller_w == 4.6
    assert tk.cycle is None


PISTON_LOG = """\
[00:00:00.000] [INFO] Logger initialized successfully
[00:00:00.010] [LOG] Lift matchload: loader seen (50 mm) -> closing clamp
[00:00:00.010] [LOG] Lift clamp -> CLOSED (PRIMED)
[00:00:00.100] [LOG] LIFTDBG mode=PRIMED state=ML_CLAMP pto=LIFT own=1 home=1 tower=9999 loader=50 | WINCH tgt=100.0 act=0.4 err=99.6 mv=0 | ANG tgt=88.0 set=88.0 act=87.1 err=0.9 mv=135 | CLAMP tgt=CLOSED pis=1 done=0
[00:00:00.200] [LOG] LIFTDBG mode=MATCHLOAD state=IDLE pto=LIFT own=1 home=1 tower=9999 loader=9999 | WINCH tgt=100.0 act=0.4 err=99.6 mv=0 | ANG tgt=88.0 set=88.0 act=87.1 err=0.9 mv=135 | CLAMP tgt=MATCHLOAD pis=0 done=1
"""


def test_parser_piston_clamp_format(tmp_path):
    """Piston-clamp logs carry pis= in place of the motor clamp's act= / err=."""
    p = tmp_path / "piston.txt"
    p.write_text(PISTON_LOG)
    s = parse_lift_log(p)[0]
    assert len(s.ticks) == 2 and len(s.clamp_events) == 1
    t0, t1 = s.ticks
    assert t0.state == "ML_CLAMP" and t0.clamp_tgt == "CLOSED"
    assert t0.clamp_piston is True and t0.clamp_done is False
    assert t0.clamp_act is None and t0.clamp_err is None and t0.roller_on is None
    assert t1.clamp_tgt == "MATCHLOAD" and t1.clamp_piston is False and t1.clamp_done is True
    assert s.clamp_events[0].text == "clamp -> CLOSED (PRIMED)"


CLAW_MOTOR_LOG = """\
[00:00:00.000] [INFO] Logger initialized successfully
[00:00:00.010] [LOG] Lift matchload: loader seen (50 mm) -> closing claw
[00:00:00.010] [LOG] Lift clamp -> CLOSED (PRIMED)
[00:00:00.100] [LOG] LIFTDBG mode=PRIMED state=ML_CLAMP pto=LIFT own=1 home=1 tower=9999 loader=50 | WINCH tgt=100.0 act=0.4 err=99.6 mv=0 | CLAMP tgt=CLOSED act=-42.5 mv=-12000 done=0
[00:00:00.200] [LOG] LIFTDBG mode=MATCHLOAD state=IDLE pto=INTAKE own=0 home=1 tower=9999 loader=9999 | WINCH tgt=100.0 act=0.4 err=99.6 mv=0 | CLAMP tgt=OPEN act=310.0 mv=0 done=1
"""


def test_parser_claw_motor_format(tmp_path):
    """From 2026-10-09: no ANG section, and the claw motor logs act= / mv= (no err=)."""
    p = tmp_path / "claw.txt"
    p.write_text(CLAW_MOTOR_LOG)
    s = parse_lift_log(p)[0]
    assert len(s.ticks) == 2 and len(s.clamp_events) == 1
    t0, t1 = s.ticks
    assert t0.state == "ML_CLAMP" and t0.clamp_tgt == "CLOSED"
    assert t0.clamp_act == -42.5 and t0.clamp_mv == -12000 and t0.clamp_done is False
    assert t0.clamp_err is None and t0.clamp_piston is None and t0.roller_on is None
    assert t0.ang_tgt is None and t0.ang_mv is None
    assert t1.mode == "MATCHLOAD" and t1.pto == "INTAKE" and not t1.own
    assert t1.clamp_tgt == "OPEN" and t1.clamp_mv == 0 and t1.clamp_done is True


# --------------------------------------------------- invariants over real logs

def _lift_logs():
    if not LOG_DIR.exists():
        return []
    out = []
    for path in sorted(LOG_DIR.glob("*.txt")) + sorted(LOG_DIR.glob("*.log")):
        try:
            if lift_sessions_with_data(parse_lift_log(path)):
                out.append(path.name)
        except OSError:
            continue
    return out


_LIFT_LOGS = _lift_logs()


@pytest.fixture(params=_LIFT_LOGS or [pytest.param("<none>", marks=pytest.mark.skip(
    reason="no log in tests/logs/ contains LIFTDBG samples yet"))],
                ids=lambda n: n)
def lift_ticks(request):
    sessions = lift_sessions_with_data(parse_lift_log(LOG_DIR / request.param))
    ticks = [tk for s in sessions for tk in s.ticks]
    return request.param, sessions, ticks


def test_no_nan_in_lift_fields(lift_ticks):
    """No NaN/inf should reach the lift log: it means a bad PID computation."""
    name, _sessions, ticks = lift_ticks
    bad = 0
    for tk in ticks:
        for v in (tk.winch_tgt, tk.winch_act, tk.ang_tgt, tk.ang_set, tk.ang_act,
                  tk.clamp_act):
            if v is not None and not math.isfinite(v):
                bad += 1
    assert bad == 0, f"{name}: {bad} non-finite lift values (check the angler PID math)"


def test_claw_rests_after_its_move(lift_ticks):
    """Once a claw move's time is up, the motor must back off (timed claw motor logs).

    An open that is done brakes (mv=0); a close that is done squeezes at no more than
    CLAW_SQUEEZE_MV. Anything else is the claw stalled against its stop at full voltage
    for as long as it sits there, which overheats it. See Lift::apply_end_effector.
    """
    name, _sessions, ticks = lift_ticks
    for tk in ticks:
        if tk.clamp_mv is None or not tk.clamp_done:
            continue
        if tk.clamp_tgt == "OPEN":
            assert tk.clamp_mv == 0, (
                f"{name}: claw still driven {tk.clamp_mv} mV after its open finished "
                "(Lift::apply_end_effector should brake)")
        else:
            assert abs(tk.clamp_mv) <= CLAW_SQUEEZE_MV, (
                f"{name}: closed claw driven {tk.clamp_mv} mV after closing, past "
                f"CLAW_SQUEEZE_MV={CLAW_SQUEEZE_MV} (Lift::apply_end_effector)")


def test_err_matches_target_minus_actual(lift_ticks):
    """The logged err column must equal target-actual (guards field ordering).

    Both come from Lift::log_debug_state; a mismatch means either a parser bug or the
    log columns drifted out of order. Tolerance covers %.1f rounding.
    """
    name, _sessions, ticks = lift_ticks
    for tk in ticks:
        for tgt, act, err, who in ((tk.winch_tgt, tk.winch_act, tk.winch_err, "winch"),
                                   (tk.ang_tgt, tk.ang_act, tk.ang_err, "angler")):
            if tgt is None:
                continue  # no angler from 2026-10-09
            assert abs((tgt - act) - err) < 0.2, (
                f"{name}: {who} err column ({err}) != tgt-act ({tgt - act:.1f})")


def test_commands_within_voltage_limits(lift_ticks):
    """Every logged motor command must sit inside the V5 +/-12000 mV range."""
    name, _sessions, ticks = lift_ticks
    for tk in ticks:
        for mv, who in ((tk.winch_mv, "winch"), (tk.ang_mv, "angler"),
                        (tk.roller_mv, "roller"), (tk.clamp_mv, "claw")):
            if mv is None:
                continue
            assert abs(mv) <= V5_MV_LIMIT + 1, (
                f"{name}: {who} command {mv} mV exceeds the +/-{V5_MV_LIMIT} clamp "
                "(Lift::drive / apply_end_effector should clamp before move_voltage)")


def test_winch_respects_lowering_cap(lift_ticks):
    """Downward winch command is capped at MAX_LOWER_MV in Lift::drive().

    Gravity already helps on the way down, and a winch driven down harder than the
    cascade falls pays out string faster than it is taken up -- that is how the spool
    tangles. A command past the cap means drive() was bypassed.
    """
    name, _sessions, ticks = lift_ticks
    for tk in ticks:
        assert tk.winch_mv >= -(MAX_LOWER_MV + 1), (
            f"{name}: winch driven {tk.winch_mv} mV down, past the "
            f"MAX_LOWER_MV={MAX_LOWER_MV} cap in Lift::drive()")


def test_no_upward_drive_at_height_cap(lift_ticks):
    """Lift::drive() zeroes upward command at or above MAX_HEIGHT_DEG."""
    name, _sessions, ticks = lift_ticks
    for tk in ticks:
        if tk.winch_act >= MAX_HEIGHT_DEG:
            assert tk.winch_mv <= 0, (
                f"{name}: winch driven {tk.winch_mv} mV up at {tk.winch_act:.1f} deg, "
                f"at/above the MAX_HEIGHT_DEG={MAX_HEIGHT_DEG:.0f} cap")


def test_winch_not_commanded_without_pto(lift_ticks):
    """A winch command while own=0 goes nowhere, and Lift::drive() says nothing about it.

    Legitimate for up to Pto::SHIFT_TIME_MS while the gears mesh. A longer run means the
    lift is being told to move with the intake holding the motors -- the failure mode
    that looks exactly like a mechanical stall in the prose log.
    """
    name, sessions, _ticks = lift_ticks
    for s in sessions:
        run_start = None
        for tk in s.ticks:
            if tk.winch_mv != 0 and not tk.own:
                run_start = tk.t if run_start is None else run_start
                held_ms = (tk.t - run_start) * 1000.0
                assert held_ms <= SHIFT_TIME_MS * 2, (
                    f"{name}: boot #{s.index} commanded the winch {tk.winch_mv} mV for "
                    f"{held_ms:.0f} ms while the PTO was {tk.pto}/own=0 "
                    f"(> 2x Pto::SHIFT_TIME_MS={SHIFT_TIME_MS}); "
                    "the motors never saw it")
            else:
                run_start = None


def test_winch_moves_when_driven(lift_ticks):
    """A winch held at full voltage must actually be moving.

    This is the boot-0161 failure: HEIGHT_MOVE pushed MAX_VOLTAGE for 1.8 s and the
    cascade went 204.7 -> 211.3 deg, ~3.6 deg/s against the ~550 deg/s a healthy raise
    manages. Lift::update()'s HEIGHT_MOVE has no stall check of its own, so this is
    where it gets caught. Only full-voltage stretches count, so the slew ramp and the
    gentler HEIGHT_APPROACH_MV / LOADING_CLEAR_MV moves do not trip it.
    """
    name, sessions, _ticks = lift_ticks
    stall_ms = 400.0
    min_deg_per_s = 30.0
    for s in sessions:
        run = []
        for tk in s.ticks + [None]:
            driving = tk is not None and tk.own and abs(tk.winch_mv) >= V5_MV_LIMIT
            if driving:
                run.append(tk)
                continue
            if len(run) >= 2:
                span_ms = (run[-1].t - run[0].t) * 1000.0
                moved = abs(run[-1].winch_act - run[0].winch_act)
                if span_ms >= stall_ms:
                    rate = moved / (span_ms / 1000.0)
                    assert rate >= min_deg_per_s, (
                        f"{name}: boot #{s.index} winch stalled -- {run[0].winch_mv} mV "
                        f"for {span_ms:.0f} ms moved {moved:.1f} deg ({rate:.1f} deg/s) "
                        f"in state {run[0].state} at {run[0].winch_act:.1f} deg. "
                        "Something is mechanically jammed, or the claw is hooked.")
            run = []


def test_timestamps_monotonic_within_session(lift_ticks):
    """Lift samples within one boot must be time-ordered (sane parse/clock)."""
    name, sessions, _ticks = lift_ticks
    for s in sessions:
        ts = [tk.t for tk in s.ticks]
        assert ts == sorted(ts), f"{name}: boot #{s.index} lift samples out of time order"
