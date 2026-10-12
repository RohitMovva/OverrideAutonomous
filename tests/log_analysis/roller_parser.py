"""Parser and speed metrics for the roller mech debug stream (ROLLERDBG).

RollerMech::log_debug_state (src/hardware/roller_mech.cpp) emits one line per 10 ms tick
while the shared PTO motors run in INTAKE mode:

    ROLLERDBG src=S state=ST cmd=C
        | PTO rpm=a,b,c mv=a,b,c ma=a,b,c
        | AXLE pos=P dps=D

Reading the columns:
  * src   -- ROLLER while a roller cycle drives the motors, INTAKE while the intake does.
    The roller mech is geared onto the intake, so both turn the same motors, gears and
    roller -- the same command should give the same speed either way.
  * state -- RollerMech::State (IDLE while the intake is the one driving).
  * cmd   -- last voltage sent to the PTO motors (Pto::last_command_mv), mV.
  * PTO   -- what each of the three PTO motors measures: rpm at the cartridge output
    (sign-corrected for reversed ports), voltage actually applied (mV), current (mA).
  * AXLE  -- the roller rotation sensor: travel in deg along the spin direction, and the
    sensor's own velocity reading in deg/s.

`steady_speeds()` compares the two sources on ticks well into a continuous run at full
command, so spin-up does not count against the short roller cycles.
"""

from __future__ import annotations

import re
import statistics
from dataclasses import dataclass, field
from pathlib import Path

from .parser import FLOAT, LINE_RE, parse_float

_INT = r"[-+]?\d+"
# Mirrors RollerMech::log_debug_state; one regex so format drift fails loudly.
_ROLLERDBG_RE = re.compile(
    r"ROLLERDBG src=(\w+) state=(\w+) cmd=({I}) "
    r"\| PTO rpm=({F}),({F}),({F}) mv=({I}),({I}),({I}) ma=({I}),({I}),({I}) "
    r"\| AXLE pos=({F}) dps=({F})"
    .replace("{F}", FLOAT).replace("{I}", _INT)
)

# A run only counts as steady this long after it starts (motors up to speed).
STEADY_AFTER_S = 0.2
# Ticks further apart than this split a run (the stream stops while the PTO is idle).
RUN_GAP_S = 0.05


@dataclass
class RollerTick:
    """One ROLLERDBG line."""
    t: float                  # seconds since midnight (log timestamp)
    src: str                  # ROLLER or INTAKE
    state: str                # RollerMech::State
    cmd: int                  # commanded PTO voltage, mV
    rpm: tuple[float, float, float]
    mv: tuple[int, int, int]  # measured motor voltage, mV
    ma: tuple[int, int, int]  # measured motor current, mA
    axle_pos: float           # roller travel, deg
    axle_dps: float           # roller speed from the rotation sensor, deg/s

    @property
    def motor_rpm(self) -> float:
        """Mean |rpm| of the PTO motors."""
        return statistics.fmean(abs(r) for r in self.rpm)

    @property
    def motor_mv(self) -> float:
        """Mean |measured voltage| of the PTO motors."""
        return statistics.fmean(abs(v) for v in self.mv)


@dataclass
class RollerSession:
    """One power-on (from a "Logger initialized successfully" line)."""
    t_start: float
    index: int = 0
    ticks: list[RollerTick] = field(default_factory=list)


def _parse_dbg(t: float, m: re.Match) -> RollerTick:
    g = m.groups()
    return RollerTick(
        t=t, src=g[0], state=g[1], cmd=int(g[2]),
        rpm=(parse_float(g[3]), parse_float(g[4]), parse_float(g[5])),
        mv=(int(g[6]), int(g[7]), int(g[8])),
        ma=(int(g[9]), int(g[10]), int(g[11])),
        axle_pos=parse_float(g[12]), axle_dps=parse_float(g[13]),
    )


def parse_roller_log(path: str | Path) -> list[RollerSession]:
    """Parse a raw log into per-boot RollerSessions (ROLLERDBG stream only)."""
    sessions: list[RollerSession] = []
    session: RollerSession | None = None
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            m = LINE_RE.match(raw.strip())
            if not m:
                continue
            h, mnt, s, ms, _level, msg = m.groups()
            t = int(h) * 3600 + int(mnt) * 60 + int(s) + int(ms) / 1000.0
            if msg.startswith("Logger initialized successfully"):
                session = RollerSession(t_start=t, index=len(sessions))
                sessions.append(session)
                continue
            if not msg.startswith("ROLLERDBG"):
                continue
            if session is None:
                session = RollerSession(t_start=t, index=0)
                sessions.append(session)
            dm = _ROLLERDBG_RE.match(msg)
            if dm:
                session.ticks.append(_parse_dbg(t, dm))
    return sessions


def roller_sessions_with_data(sessions: list[RollerSession],
                              min_ticks: int = 1) -> list[RollerSession]:
    """Sessions that produced ROLLERDBG ticks."""
    return [s for s in sessions if len(s.ticks) >= min_ticks]


def runs(ticks: list[RollerTick]) -> list[list[RollerTick]]:
    """Split ticks into continuous runs of one source at one command."""
    out: list[list[RollerTick]] = []
    for tk in ticks:
        cur = out[-1] if out else None
        if (cur and tk.t - cur[-1].t <= RUN_GAP_S and tk.src == cur[-1].src
                and tk.cmd == cur[-1].cmd):
            cur.append(tk)
        else:
            out.append([tk])
    return out


def steady_ticks(ticks: list[RollerTick], src: str, full_mv: int = 12000) -> list[RollerTick]:
    """Ticks from `src` at full command, at least STEADY_AFTER_S into their run."""
    return [tk for run in runs(ticks)
            if run[0].src == src and abs(run[0].cmd) >= full_mv
            for tk in run if tk.t - run[0].t >= STEADY_AFTER_S - 1e-6]  # ms timestamps in float s


@dataclass
class SpeedSummary:
    src: str
    ticks: int
    motor_rpm: float     # median mean-|rpm| of the PTO motors
    motor_mv: float      # median mean-|measured mV|
    axle_dps: float      # median |roller deg/s|


def steady_speeds(ticks: list[RollerTick]) -> dict[str, SpeedSummary]:
    """Median steady-state speed per source (ROLLER / INTAKE) that has any steady ticks."""
    out = {}
    for src in ("ROLLER", "INTAKE"):
        st = steady_ticks(ticks, src)
        if st:
            out[src] = SpeedSummary(
                src=src, ticks=len(st),
                motor_rpm=statistics.median(tk.motor_rpm for tk in st),
                motor_mv=statistics.median(tk.motor_mv for tk in st),
                axle_dps=statistics.median(abs(tk.axle_dps) for tk in st),
            )
    return out
