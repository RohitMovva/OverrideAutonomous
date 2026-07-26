"""Parser for the lift/chainbar debug stream in OverrideAutonomous robot logs.

Separate from parser.py (which models trajectory following) because the lift
runs in opcontrol, not during Robot::followTrajectory, and emits its own flat
key=value stream from Lift::update()/Lift::log_debug_state (src/hardware/lift.cpp):

    LIFTDBG goal=G phase=P seq=S wmode=W respooled=R
        | WINCH tgt=.. act=.. err=.. mv=.. vel=.. floor=..
        | BAR   tgt=.. act=.. err=.. mv=.. vel=.. ff=.. sff=..
        | EE    tgt=.. act=.. err=.. mv=.. vel=..
        | claw=..                                          (one throttled line/tick)

    LIFTCMD <text>     -- a driver/macro command was issued (prime, score, ...)
    LIFTEVT <text>     -- a discrete transition (respool START/TAUT/TIMEOUT,
                          startup step X -> Y)

Angles are output deg for the bar, motor deg for winch/EE; mv are motor
millivolts; velocities are motor RPM (winch) / output deg-per-s (bar) etc.

Sessions are split on "Logger initialized successfully" (every boot logs it,
even the truncated boots that never reach "Robot initialized"), so a session
here is one power-on. `lift_sessions_with_data()` filters to the ones that
actually produced LIFTDBG ticks.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

from .parser import FLOAT, LINE_RE, parse_float

# The full LIFTDBG line. Kept in one regex so a format drift fails loudly (the
# parser stops matching) rather than silently dropping a field. Mirrors
# Lift::log_debug_state (src/hardware/lift.cpp): mode + startup step + winch mode
# + raise progress, then per-actuator target/actual/err/mv/vel (+ gravity ff on
# the bar), then the claw state.
_LIFTDBG_RE = re.compile(
    r"LIFTDBG mode=(\w+) seq=(\w+) wmode=(\w+) p=({F}) respooled=(\d+) "
    r"\| WINCH tgt=({F}) act=({F}) err=({F}) mv=({F}) vel=({F}) "
    r"\| BAR tgt=({F}) act=({F}) err=({F}) mv=({F}) vel=({F}) ff=({F}) sff=({F}) "
    r"\| EE tgt=({F}) act=({F}) err=({F}) mv=({F}) vel=({F}) "
    r"\| claw=(\d+)".replace("{F}", FLOAT)
)


@dataclass
class LiftTick:
    """One throttled Lift::update() sample (a LIFTDBG line)."""
    t: float                 # seconds since midnight (log timestamp)
    mode: str                # high-level lift mode (STARTUP, STOW, LOADING, ...)
    seq: str                 # startup-choreography step (IDLE when not running)
    wmode: str               # winch mode: HOLD or RESPOOL
    p: float                 # raise progress (0..1)
    respooled: bool
    # winch (motor deg / mV / RPM)
    winch_tgt: float
    winch_act: float
    winch_err: float
    winch_mv: float
    winch_vel: float
    # chainbar (output deg / mV / deg-per-s)
    bar_tgt: float
    bar_act: float
    bar_err: float
    bar_mv: float
    bar_vel: float
    bar_ff: float            # gravity feed-forward (mV)
    bar_sff: float           # static/velocity feed-forward (mV) -- the "sff=" column
    # end effector / chain (motor deg / mV / deg-per-s)
    ee_tgt: float
    ee_act: float
    ee_err: float
    ee_mv: float
    ee_vel: float
    claw: bool


@dataclass
class LiftEvent:
    """A LIFTCMD (command) or LIFTEVT (transition) line."""
    t: float
    kind: str                # "CMD" or "EVT"
    text: str                # everything after the tag


@dataclass
class WinchDriveSample:
    """One per-tick winch trace sample from a homing phase (LIFTSEEK during the
    down-seek, or LIFTRESPOOL during the reel-in). Same fields for both.

    Reading the signature:
      * SEEK    -- does `vel` collapse to ~0 (STALL) or spike (FREESPIN) at bottom?
      * RESPOOL -- is `cur` LOW while reeling then SPIKING at the taut stall
        (stops at the bottom, correct), or sustained HIGH (hauling the lift up)?
    """
    t: float                 # log timestamp (s since midnight)
    ms: int                  # ms since the phase started
    winch: float             # winch position (motor deg)
    vel: float               # winch velocity (motor RPM)
    cur: float               # winch current draw (mA)
    mv: float                # applied voltage (mV)


# Back-compat alias (was LiftSeekSample when only the seek trace existed).
LiftSeekSample = WinchDriveSample

_LIFTSEEK_RE = re.compile(
    rf"LIFTSEEK t=(\d+) winch=({FLOAT}) vel=({FLOAT}) cur=({FLOAT}) mv=({FLOAT})")
_LIFTRESPOOL_RE = re.compile(
    rf"LIFTRESPOOL t=(\d+) winch=({FLOAT}) vel=({FLOAT}) cur=({FLOAT}) mv=({FLOAT})")


@dataclass
class LiftSession:
    """One power-on (from a "Logger initialized successfully" line)."""
    t_start: float
    index: int = 0
    ticks: list[LiftTick] = field(default_factory=list)
    events: list[LiftEvent] = field(default_factory=list)
    seeks: list[WinchDriveSample] = field(default_factory=list)      # LIFTSEEK trace
    respools: list[WinchDriveSample] = field(default_factory=list)   # LIFTRESPOOL trace

    @property
    def commands(self) -> list[LiftEvent]:
        return [e for e in self.events if e.kind == "CMD"]

    @property
    def transitions(self) -> list[LiftEvent]:
        return [e for e in self.events if e.kind == "EVT"]


def _parse_dbg(t: float, m: re.Match) -> LiftTick:
    g = m.groups()
    return LiftTick(
        t=t,
        mode=g[0], seq=g[1], wmode=g[2], p=parse_float(g[3]), respooled=(g[4] != "0"),
        winch_tgt=parse_float(g[5]), winch_act=parse_float(g[6]),
        winch_err=parse_float(g[7]), winch_mv=parse_float(g[8]),
        winch_vel=parse_float(g[9]),
        bar_tgt=parse_float(g[10]), bar_act=parse_float(g[11]),
        bar_err=parse_float(g[12]), bar_mv=parse_float(g[13]),
        bar_vel=parse_float(g[14]), bar_ff=parse_float(g[15]),
        bar_sff=parse_float(g[16]),
        ee_tgt=parse_float(g[17]), ee_act=parse_float(g[18]),
        ee_err=parse_float(g[19]), ee_mv=parse_float(g[20]),
        ee_vel=parse_float(g[21]),
        claw=(g[22] != "0"),
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
                # Lift lines before the first boot marker: start an implicit
                # session so nothing is silently dropped.
                session = LiftSession(t_start=t, index=0)
                sessions.append(session)

            if msg.startswith("LIFTDBG"):
                dm = _LIFTDBG_RE.match(msg)
                if dm:
                    session.ticks.append(_parse_dbg(t, dm))
                continue
            if msg.startswith("LIFTCMD"):
                session.events.append(LiftEvent(t=t, kind="CMD", text=msg[len("LIFTCMD "):]))
                continue
            if msg.startswith("LIFTEVT"):
                session.events.append(LiftEvent(t=t, kind="EVT", text=msg[len("LIFTEVT "):]))
                continue
            if msg.startswith("LIFTSEEK"):
                sm = _LIFTSEEK_RE.match(msg)
                if sm:
                    session.seeks.append(WinchDriveSample(
                        t=t, ms=int(sm.group(1)), winch=parse_float(sm.group(2)),
                        vel=parse_float(sm.group(3)), cur=parse_float(sm.group(4)),
                        mv=parse_float(sm.group(5))))
                continue
            if msg.startswith("LIFTRESPOOL"):
                sm = _LIFTRESPOOL_RE.match(msg)
                if sm:
                    session.respools.append(WinchDriveSample(
                        t=t, ms=int(sm.group(1)), winch=parse_float(sm.group(2)),
                        vel=parse_float(sm.group(3)), cur=parse_float(sm.group(4)),
                        mv=parse_float(sm.group(5))))
                continue

    return sessions


def lift_sessions_with_data(sessions: list[LiftSession], min_ticks: int = 1) -> list[LiftSession]:
    """Sessions that actually produced LIFTDBG ticks (ignores truncated boots)."""
    return [s for s in sessions if len(s.ticks) >= min_ticks]


def last_lift_session(sessions: list[LiftSession], min_ticks: int = 1) -> LiftSession | None:
    """The most recent boot that produced lift data, or None."""
    have = lift_sessions_with_data(sessions, min_ticks)
    return have[-1] if have else None
