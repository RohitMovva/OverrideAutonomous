"""Parser for the lift debug stream in OverrideAutonomous robot logs.

Separate from parser.py (which models trajectory following) because the lift runs its
own state machine on top of the drive loop and emits a flat key=value stream from
Lift::log_debug_state (src/hardware/lift.cpp), one line per 10 ms Lift::update():

    LIFTDBG mode=M state=S pto=P own=O home=H tower=T loader=L
        | WINCH tgt=.. act=.. err=.. mv=..
        | ANG   tgt=.. set=.. act=.. err=.. mv=..  (angler; absent from 2026-10-09)
        | CLAMP tgt=.. act=.. err=.. done=..  (clamp motor: before 2026-09-25, and
                                               again on 2026-09-27)
        | CLAMP tgt=.. pis=.. done=..          (clamp piston, 2026-09-27 to 10-08)
        | CLAMP tgt=.. act=.. mv=.. done=..    (timed claw motor, from 2026-10-09)
        | ROLLER on=.. mv=.. [rpm=.. ma=.. w=.. [cyc=..]]  (claw roller, 2026-09-25 to
                                                             09-26; rpm/ma/w from 09-26;
                                                             cyc only while the intake
                                                             cycle existed)

Reading the columns:
  * mode / state -- Lift::Mode and Lift::State. `state` is the fine-grained one; a macro
    that is not progressing shows as the same state for many ticks.
  * pto / own    -- which subsystem holds the shared motors, and whether the shift has
    settled. `own=0` while the winch is being commanded is the one way Lift::drive()
    sends the motors nowhere without logging anything.
  * tower        -- raw tower distance sensor (score macro), mm. 9999 means nothing in view.
  * loader       -- raw match-loader distance sensor (matchload detection), mm. Absent
    from logs before the second sensor was added; those logs used `tower` for both.
  * WINCH        -- winch motor deg. `tgt` is only meaningful in HEIGHT_MOVE; elsewhere
    it is whatever the last raise_to() asked for, so `err` beside it is noise there.
    `mv` is in the Lift class's convention (positive = up) and is what Lift::drive()
    was asked for, captured before the PTO check can zero it -- so `mv != 0` with
    `own=0` is a command the motors never saw.
  * ANG          -- angler output deg (logs before 2026-10-09; the angler was removed). `tgt` is the mode's target, `set` the ramped
    setpoint the PID actually tracks. tgt far from act for many ticks means the claw is
    not making a commanded swing -- it is blocked, or the ramp has not caught up.
  * ROLLER       -- whether the claw roller is on and the voltage it was sent. Newer
    logs add its measured speed (rpm), current (mA) and power (W). cyc=1 while the
    B-button intake cycle was on; that cycle has since been removed, so newer logs omit
    the field.
  * CLAMP        -- the named target (CLOSED, CLEAR, MATCHLOAD, OPEN; older logs also
    MATCHLOAD_STACK). Motor-clamp logs add the clamp's deg (act) and the error to the
    target; done=1 once it is within tolerance or its timeout has run out. Piston-clamp
    logs have pis=1 while the solenoid is closed (CLOSED and CLEAR close it, MATCHLOAD
    and OPEN open it) instead of act/err, and done=1 once CLAMP_ACTUATE_MS has passed
    since it switched. From 2026-10-09 the claw is a motor with no position control
    and only CLOSED / OPEN: act is its encoder deg (tared closed at boot), mv the
    voltage it was sent (positive = open; 0 once an open has braked), and done=1 once
    CLAW_OPEN_MS / CLAW_CLOSE_MS has passed since the target changed. The claw was a
    roller for 2026-09-25/26; those
    ticks carry ROLLER instead, and CLAMP ticks have no ROLLER fields.

Sessions are split on "Logger initialized successfully" (every boot logs it, even the
truncated boots that never reach "Robot initialized"), so a session here is one
power-on. `lift_sessions_with_data()` filters to the ones that produced LIFTDBG ticks.

The macro's own prose lines ("Lift matchload macro: moving 47.5 -> 100.0 deg", "Lift
roller -> ON (MATCHLOAD)", or "Lift clamp -> CLEAR (MATCHLOAD)" on older logs) are collected as LiftEvents. They are what marks the
boundaries of a cycle, so the dashboard annotates the traces with them.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

from .parser import FLOAT, LINE_RE, parse_float

# The full LIFTDBG line. Kept in one regex so a format drift fails loudly (the parser
# stops matching) rather than silently dropping a field. Mirrors Lift::log_debug_state.
_INT = r"[-+]?\d+"
_LIFTDBG_RE = re.compile(
    r"LIFTDBG mode=(?P<mode>\w+) state=(?P<state>\w+) pto=(?P<pto>\w+) own=(?P<own>\d) "
    r"home=(?P<home>\d) tower=(?P<tower>{I}) (?:loader=(?P<loader>{I}) )?"
    r"\| WINCH tgt=(?P<w_tgt>{F}) act=(?P<w_act>{F}) err=(?P<w_err>{F}) mv=(?P<w_mv>{I}) "
    r"(?:\| ANG tgt=(?P<a_tgt>{F}) set=(?P<a_set>{F}) act=(?P<a_act>{F}) err=(?P<a_err>{F}) "
    r"mv=(?P<a_mv>{I}) )?"
    r"(?:\| CLAMP tgt=(?P<c_tgt>\w+) "
    r"(?:act=(?P<c_act>{F}) (?:err=(?P<c_err>{F})|mv=(?P<c_mv>{I}))|pis=(?P<c_pis>\d)) "
    r"done=(?P<c_done>\d)"
    r"|\| ROLLER on=(?P<r_on>\d) mv=(?P<r_mv>{I})"
    r"(?: rpm=(?P<r_rpm>{F}) ma=(?P<r_ma>{I}) w=(?P<r_w>{F})(?: cyc=(?P<cyc>\d))?)?)"
    .replace("{F}", FLOAT).replace("{I}", _INT)
)


@dataclass
class LiftTick:
    """One Lift::update() sample (a LIFTDBG line)."""
    t: float                 # seconds since midnight (log timestamp)
    mode: str                # Lift::Mode -- PRIMED, SCORING, MATCHLOAD (LOADING before 2026-10-09)
    state: str               # Lift::State -- IDLE, ML_DESCEND, HEIGHT_MOVE, ...
    pto: str                 # which subsystem holds the shared motors: LIFT or INTAKE
    own: bool                # PTO is ours AND the shift has settled
    home: bool
    tower: int               # tower distance sensor, mm (9999 = nothing in view)
    # winch / cascade (motor deg, mV positive = up)
    winch_tgt: float
    winch_act: float
    winch_err: float
    winch_mv: int
    # loader distance sensor, mm (9999 = nothing in view). None on logs from before
    # it existed, when the tower sensor did the matchload detection too
    loader: int | None = None
    # angler (output deg, mV). None on logs from 2026-10-09 on, when it was removed
    ang_tgt: float | None = None  # the mode's target
    ang_set: float | None = None  # ramped setpoint the PID tracks
    ang_act: float | None = None
    ang_err: float | None = None
    ang_mv: int | None = None
    # clamp. None on roller-claw logs
    clamp_tgt: str | None = None  # named target: CLOSED, OPEN; older logs also CLEAR, MATCHLOAD, MATCHLOAD_STACK
    clamp_act: float | None = None  # motor clamp/claw only (deg); None for the piston
    clamp_err: float | None = None  # position-controlled clamp motor only (before 2026-09-27)
    clamp_mv: int | None = None  # timed claw motor only (from 2026-10-09): mV, positive = open
    clamp_piston: bool | None = None  # piston clamp only: solenoid closed; None for the motor
    clamp_done: bool | None = None  # motor: settled or timed out; piston: actuation time passed
    # roller (mV after ROLLER_DIRECTION). None on clamp-claw logs
    roller_on: bool | None = None
    roller_mv: int | None = None
    # claw roller measured speed (rpm) / current (mA), and the intake cycle flag. None
    # on logs from before 2026-09-26
    roller_rpm: float | None = None
    roller_ma: int | None = None
    roller_w: float | None = None
    cycle: bool | None = None


@dataclass
class LiftEvent:
    """One of the lift's prose log lines, e.g. "roller -> ON (MATCHLOAD)"."""
    t: float
    text: str                # the message with the leading "Lift " stripped

    @property
    def is_clamp(self) -> bool:
        return self.text.startswith("clamp ->")

    @property
    def is_roller(self) -> bool:
        return self.text.startswith("roller ->")

    @property
    def is_macro(self) -> bool:
        return "macro" in self.text


@dataclass
class LiftSession:
    """One power-on (from a "Logger initialized successfully" line)."""
    t_start: float
    index: int = 0
    ticks: list[LiftTick] = field(default_factory=list)
    events: list[LiftEvent] = field(default_factory=list)

    @property
    def clamp_events(self) -> list[LiftEvent]:
        return [e for e in self.events if e.is_clamp]

    @property
    def roller_events(self) -> list[LiftEvent]:
        return [e for e in self.events if e.is_roller]

    @property
    def macro_events(self) -> list[LiftEvent]:
        return [e for e in self.events if e.is_macro]


def _parse_dbg(t: float, m: re.Match) -> LiftTick:
    g = m.groupdict()

    def opt(key: str, conv):
        return conv(g[key]) if g[key] is not None else None

    def flag(v: str) -> bool:
        return v != "0"

    return LiftTick(
        t=t,
        mode=g["mode"], state=g["state"], pto=g["pto"],
        own=flag(g["own"]), home=flag(g["home"]), tower=int(g["tower"]),
        loader=opt("loader", int),
        winch_tgt=parse_float(g["w_tgt"]), winch_act=parse_float(g["w_act"]),
        winch_err=parse_float(g["w_err"]), winch_mv=int(g["w_mv"]),
        ang_tgt=opt("a_tgt", parse_float), ang_set=opt("a_set", parse_float),
        ang_act=opt("a_act", parse_float), ang_err=opt("a_err", parse_float),
        ang_mv=opt("a_mv", int),
        clamp_tgt=g["c_tgt"],
        clamp_act=opt("c_act", parse_float),
        clamp_err=opt("c_err", parse_float),
        clamp_mv=opt("c_mv", int),
        clamp_piston=opt("c_pis", flag),
        clamp_done=opt("c_done", flag),
        roller_on=opt("r_on", flag),
        roller_mv=opt("r_mv", int),
        roller_rpm=opt("r_rpm", parse_float),
        roller_ma=opt("r_ma", int),
        roller_w=opt("r_w", parse_float),
        cycle=opt("cyc", flag),
    )


def parse_lift_log(path: str | Path) -> list[LiftSession]:
    """Parse a raw log into per-boot LiftSessions (lift stream only)."""
    sessions: list[LiftSession] = []
    session: LiftSession | None = None

    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            m = LINE_RE.match(raw.strip())
            if not m:
                continue
            h, mnt, s, ms, _level, msg = m.groups()
            t = int(h) * 3600 + int(mnt) * 60 + int(s) + int(ms) / 1000.0

            if msg.startswith("Logger initialized successfully"):
                session = LiftSession(t_start=t, index=len(sessions))
                sessions.append(session)
                continue
            if session is None:
                # Lift lines before the first boot marker: start an implicit session so
                # nothing is silently dropped.
                session = LiftSession(t_start=t, index=0)
                sessions.append(session)

            if msg.startswith("LIFTDBG"):
                dm = _LIFTDBG_RE.match(msg)
                if dm:
                    session.ticks.append(_parse_dbg(t, dm))
                continue
            if msg.startswith("Lift "):
                session.events.append(LiftEvent(t=t, text=msg[len("Lift "):]))
                continue

    return sessions


def lift_sessions_with_data(sessions: list[LiftSession], min_ticks: int = 1) -> list[LiftSession]:
    """Sessions that actually produced LIFTDBG ticks (ignores truncated boots)."""
    return [s for s in sessions if len(s.ticks) >= min_ticks]


def last_lift_session(sessions: list[LiftSession], min_ticks: int = 1) -> LiftSession | None:
    """The most recent boot that produced lift data, or None."""
    have = lift_sessions_with_data(sessions, min_ticks)
    return have[-1] if have else None
