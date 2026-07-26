"""Health checks for the lift/chainbar debug stream (LIFTDBG/LIFTCMD/LIFTEVT).

Two layers:
  * A parser round-trip on a synthetic line, so the parser is verified even when
    no real log contains lift data yet (the format is the contract — if
    Lift::log_debug_state changes, this fails loudly).
  * Invariants over any real logs in tests/logs/ that actually carry lift
    samples. Logs without lift data are skipped, so this stays green until a
    debug-build opcontrol run is captured.

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

# Chainbar/chain/winch commands are clamped to the V5 motor range in
# Lift::update() (bar_mv std::clamp to +/-12000; move_voltage clamps the rest).
V5_MV_LIMIT = 12000.0

SYNTHETIC_LOG = """\
[00:00:00.000] [INFO] Logger initialized successfully
[00:00:00.010] [LOG] LIFTCMD start_to_loading -> begin startup choreography
[00:00:00.050] [LOG] LIFTEVT startup step BAR_CLEAR -> RAISE (bar=-30.0 winch=0.0)
[00:00:01.000] [LOG] LIFTDBG mode=PRIMING seq=IDLE wmode=HOLD p=0.00 respooled=0 | WINCH tgt=0.0 act=0.0 err=0.0 mv=60 vel=0.0 | BAR tgt=-305.0 act=-300.4 err=-4.6 mv=-12000 vel=210.5 ff=-2988 sff=3000 | EE tgt=-90.0 act=37.6 err=-127.6 mv=-12000 vel=0.0 | claw=1
[00:00:02.000] [LOG] LIFTEVT respool START @ winch=0.0
[00:00:02.200] [LOG] LIFTEVT respool TAUT after 200 ms @ winch=30.0 vel=5.0 (re-zero to 0)
[00:00:03.000] [LOG] LIFTDBG mode=STOW seq=IDLE wmode=RESPOOL p=0.50 respooled=0 | WINCH tgt=0.0 act=5.0 err=-5.0 mv=2000 vel=120.0 | BAR tgt=-20.0 act=-20.0 err=0.0 mv=-5 vel=0.0 ff=684 sff=0 | EE tgt=0.0 act=0.0 err=0.0 mv=0 vel=0.0 | claw=0
"""


# ------------------------------------------------------------------ parser

def test_parser_roundtrip(tmp_path):
    """The parser must recover every field of the LIFTDBG/CMD/EVT format."""
    p = tmp_path / "synthetic.txt"
    p.write_text(SYNTHETIC_LOG)
    sessions = parse_lift_log(p)
    assert len(sessions) == 1
    s = sessions[0]

    assert len(s.ticks) == 2
    assert len(s.commands) == 1
    assert len(s.transitions) == 3   # startup step + respool START + TAUT

    t0 = s.ticks[0]
    assert t0.mode == "PRIMING" and t0.seq == "IDLE" and t0.wmode == "HOLD"
    assert t0.p == 0.0 and t0.respooled is False
    assert t0.bar_tgt == -305.0 and t0.bar_act == -300.4 and t0.bar_err == -4.6
    assert t0.bar_mv == -12000.0 and t0.bar_ff == -2988.0 and t0.bar_sff == 3000.0
    assert t0.ee_tgt == -90.0 and t0.ee_act == 37.6 and t0.ee_mv == -12000.0
    assert t0.claw is True and t0.winch_mv == 60.0

    t1 = s.ticks[1]
    assert t1.mode == "STOW" and t1.p == 0.5
    assert t1.wmode == "RESPOOL" and t1.winch_mv == 2000.0 and t1.claw is False

    assert s.commands[0].text.startswith("start_to_loading")
    assert any("respool TAUT" in e.text for e in s.transitions)


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
    """No NaN/inf should reach the lift log: it means a bad PID/FF computation."""
    name, _sessions, ticks = lift_ticks
    bad = 0
    for tk in ticks:
        for v in (tk.winch_tgt, tk.winch_act, tk.winch_mv, tk.bar_tgt, tk.bar_act,
                  tk.bar_mv, tk.bar_ff, tk.ee_tgt, tk.ee_act, tk.ee_mv):
            if not math.isfinite(v):
                bad += 1
    assert bad == 0, f"{name}: {bad} non-finite lift values (check PID / gravity FF math)"


def test_err_matches_target_minus_actual(lift_ticks):
    """The logged err column must equal target-actual (guards field ordering).

    Both come from Lift::log_debug_state; a mismatch means either a parser bug
    or the log columns drifted out of order. Tolerance covers %.1f rounding.
    """
    name, _sessions, ticks = lift_ticks
    for tk in ticks:
        for tgt, act, err, who in ((tk.winch_tgt, tk.winch_act, tk.winch_err, "winch"),
                                   (tk.bar_tgt, tk.bar_act, tk.bar_err, "bar"),
                                   (tk.ee_tgt, tk.ee_act, tk.ee_err, "ee")):
            assert abs((tgt - act) - err) < 0.2, (
                f"{name}: {who} err column ({err}) != tgt-act ({tgt - act:.1f})")


def test_commands_within_voltage_limits(lift_ticks):
    """Every logged motor command must sit inside the V5 +/-12000 mV range."""
    name, _sessions, ticks = lift_ticks
    for tk in ticks:
        for mv, who in ((tk.winch_mv, "winch"), (tk.bar_mv, "bar"), (tk.ee_mv, "ee")):
            assert abs(mv) <= V5_MV_LIMIT + 1.0, (
                f"{name}: {who} command {mv} mV exceeds the +/-{V5_MV_LIMIT:.0f} clamp "
                "(Lift::update() should clamp before move_voltage)")


def test_respool_only_drives_up(lift_ticks):
    """While respooling, the winch is only ever driven in the raise direction.

    Lift::update() applies WINCH_RESPOOL_MV (positive = up) or 0 during RESPOOL;
    a negative command there would mean the mode machine let PID output through
    and would pay out string instead of taking up slack.
    """
    name, _sessions, ticks = lift_ticks
    for tk in ticks:
        if tk.wmode == "RESPOOL":
            assert tk.winch_mv >= -1.0, (
                f"{name}: winch driven {tk.winch_mv} mV (downward) while RESPOOL")


def test_timestamps_monotonic_within_session(lift_ticks):
    """Lift samples within one boot must be time-ordered (sane parse/clock)."""
    name, sessions, _ticks = lift_ticks
    for s in sessions:
        ts = [tk.t for tk in s.ticks]
        assert ts == sorted(ts), f"{name}: boot #{s.index} lift samples out of time order"
