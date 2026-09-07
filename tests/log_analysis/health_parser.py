"""Parser for the drivetrain health stream in OverrideAutonomous robot logs.

Separate from parser.py (which models trajectory following) because this stream
spans BOTH modes — Robot::logDriveHealth is called from the followTrajectory
loop and from the opcontrol heartbeat, since motors keep heating while the robot
is driven back to the start between runs. Sessions are therefore split on boot
("Logger initialized successfully"), the same convention as lift_parser.py,
rather than on trajectory runs.

Current format (src/hardware/robot.cpp, logSideHealth) — one line per motor,
sampled every Robot::DRIVE_HEALTH_LOG_MS:

    DRIVEHEALTH <context> <side><index> port=<p> tempC=<c> curMA=<mA> \
        W=<watts> eff=<pct> ovt=<0|1>

    DRIVEHEALTH auton L0 port=-18 tempC=35 curMA=810 W=5.2 eff=71 ovt=0

`context` is "auton" or "driver". `side` is L/R, `index` is the motor's position
within that side's MotorGroup — the same ordering as the "Left Positions:" /
"Right Positions:" columns in the odometry stream, so the two can be correlated.
`port` is the signed smart port (negative = reversed), so a finding names the
motor to actually swap. Any field can read "--" when PROS reported PROS_ERR_F /
PROS_ERR for that channel; those parse to NaN.

LEGACY format (also read, so logs captured before the per-motor split remain
analysable — 08012026-9 is the only one, and it predates both W/eff and the
split). One line per sample with comma-joined per-motor values:

    DRIVEHEALTH auton | L tempC=35,35 curMA=810,888 ovt=0,0 | R tempC=...

W= and eff= are optional there; when absent they parse to None so plots and
invariants can skip those panels rather than inventing numbers.

Why per-motor matters: the two motors on a side are mechanically coupled, so
they turn at the same speed under the same command. That makes the healthy one
a built-in control for the sick one — no baseline run required — which is what
`partner_imbalance()` below measures.
"""

from __future__ import annotations

import math
import re
import statistics
from dataclasses import dataclass, field
from pathlib import Path

from .parser import FLOAT, LINE_RE, parse_float
from .robot_config import drive_port

# A reading is a float, or "--" when logDriveHealth could not read the channel.
_VAL = rf"(?:--|{FLOAT})"

_MOTOR_RE = re.compile(
    rf"DRIVEHEALTH (\S+) ([LR])(\d+) port=(-?\d+) tempC=({_VAL}) curMA=({_VAL}) "
    rf"W=({_VAL}) eff=({_VAL}) ovt=({_VAL})"
)

# Legacy per-side line: "L tempC=a,b curMA=a,b [W=a,b] [eff=a,b] ovt=a,b"
_LEGACY_SIDE_RE = re.compile(
    rf"([LR]) tempC=({_VAL}(?:,{_VAL})*) curMA=({_VAL}(?:,{_VAL})*)"
    rf"(?: W=({_VAL}(?:,{_VAL})*))?"
    rf"(?: eff=({_VAL}(?:,{_VAL})*))?"
    rf" ovt=({_VAL}(?:,{_VAL})*)"
)


def _val(s: str | None) -> float:
    """One reading; "--" (unreadable channel) and None both become NaN."""
    if s is None or s == "--":
        return math.nan
    return parse_float(s)


def _vals(s: str | None) -> list[float] | None:
    """A comma-joined legacy field, or None when the field was absent."""
    if s is None:
        return None
    return [_val(p) for p in s.split(",")]


@dataclass
class MotorSample:
    """One motor's reading at one sample instant."""
    t: float                      # seconds since midnight (log timestamp)
    context: str                  # "auton" or "driver"
    side: str                     # "L" or "R"
    index: int                    # position within that side's MotorGroup
    port: int | None              # signed smart port; None in legacy logs
    temp_c: float
    current_ma: float
    power_w: float | None         # None when the log predates the W= field
    efficiency_pct: float | None  # None when the log predates the eff= field
    over_temp: bool

    @property
    def key(self) -> tuple[str, int]:
        return (self.side, self.index)

    @property
    def label(self) -> str:
        return f"{self.side}{self.index}" + (f" (port {self.port})" if self.port is not None else "")

    @property
    def volts(self) -> float:
        """Volts the motor actually saw, recovered as P/I.

        The discriminator for a motor that is not RECEIVING its command — a bad
        connector, a damaged wire, a failing winding — as opposed to one that is
        getting the power and wasting it (which shows up in efficiency instead).
        NaN below 100 mA, where the quotient is noise.
        """
        if self.power_w is None or not math.isfinite(self.current_ma) or self.current_ma < 100:
            return math.nan
        return self.power_w / (self.current_ma / 1000.0)


@dataclass
class HealthSession:
    """One power-on (from a "Logger initialized successfully" line)."""
    t_start: float
    index: int = 0
    samples: list[MotorSample] = field(default_factory=list)

    @property
    def motors(self) -> list[tuple[str, int]]:
        """Distinct (side, index) keys, in stable L0,L1,R0,R1 order."""
        return sorted({s.key for s in self.samples})

    def by_motor(self, side: str, index: int) -> list[MotorSample]:
        return [s for s in self.samples if s.side == side and s.index == index]

    def by_side(self, side: str) -> list[MotorSample]:
        return [s for s in self.samples if s.side == side]

    def ports(self) -> dict[tuple[str, int], int | None]:
        """Port per motor. Legacy lines carry no port=, so fall back to the
        wiring in main.cpp -- a finding should name a physical motor even when
        the log predates the port column."""
        out: dict[tuple[str, int], int | None] = {}
        for s in self.samples:
            if out.get(s.key) is None:
                out[s.key] = s.port if s.port is not None else drive_port(s.side, s.index)
        return out

    @property
    def has_power(self) -> bool:
        return any(s.power_w is not None for s in self.samples)

    @property
    def has_efficiency(self) -> bool:
        return any(s.efficiency_pct is not None for s in self.samples)


def _parse_motor_line(t: float, m: re.Match) -> MotorSample:
    g = m.groups()
    return MotorSample(
        t=t, context=g[0], side=g[1], index=int(g[2]), port=int(g[3]),
        temp_c=_val(g[4]), current_ma=_val(g[5]),
        power_w=_val(g[6]), efficiency_pct=_val(g[7]),
        over_temp=(_val(g[8]) >= 0.5),
    )


def _parse_legacy_line(t: float, msg: str) -> list[MotorSample]:
    """Expand one legacy per-side line into per-motor samples."""
    context = msg.split()[1] if len(msg.split()) > 1 else "?"
    out: list[MotorSample] = []
    for sm in _LEGACY_SIDE_RE.finditer(msg):
        side = sm.group(1)
        temps = _vals(sm.group(2)) or []
        curs = _vals(sm.group(3)) or []
        pwrs = _vals(sm.group(4))
        effs = _vals(sm.group(5))
        ovts = _vals(sm.group(6)) or []
        for i in range(len(temps)):
            out.append(MotorSample(
                t=t, context=context, side=side, index=i, port=None,
                temp_c=temps[i],
                current_ma=curs[i] if i < len(curs) else math.nan,
                power_w=(pwrs[i] if pwrs and i < len(pwrs) else None),
                efficiency_pct=(effs[i] if effs and i < len(effs) else None),
                over_temp=(i < len(ovts) and ovts[i] >= 0.5),
            ))
    return out


def parse_health_log(path: str | Path) -> list[HealthSession]:
    """Parse a raw log into per-boot HealthSessions (drive health stream only)."""
    sessions: list[HealthSession] = []
    session: HealthSession | None = None

    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            m = LINE_RE.match(raw.strip())
            if not m:
                continue
            h, mnt, s, ms, _level, msg = m.groups()
            t = int(h) * 3600 + int(mnt) * 60 + int(s) + int(ms) / 1000.0

            if msg.startswith("Logger initialized successfully"):
                session = HealthSession(t_start=t, index=len(sessions))
                sessions.append(session)
                continue
            if not msg.startswith("DRIVEHEALTH"):
                continue
            if session is None:
                # Health lines before the first boot marker: start an implicit
                # session so nothing is silently dropped.
                session = HealthSession(t_start=t, index=0)
                sessions.append(session)

            mm = _MOTOR_RE.match(msg)
            if mm:
                session.samples.append(_parse_motor_line(t, mm))
            else:
                session.samples.extend(_parse_legacy_line(t, msg))

    return sessions


def health_sessions_with_data(sessions: list[HealthSession],
                              min_samples: int = 1) -> list[HealthSession]:
    """Sessions that actually produced DRIVEHEALTH samples (ignores truncated boots)."""
    return [s for s in sessions if len(s.samples) >= min_samples]


# ------------------------------------------------------------------ metrics

# Below this the side is coasting, and the partner comparison is dominated by
# offset noise rather than by how the two motors share a real load.
LOADED_SIDE_MA = 600.0

# A motor drawing at least this much is engaged and can be compared against its
# partner. Below it the motor is contributing nothing and belongs to the
# cut-out statistic instead, not to the sharing ratio.
ENGAGED_MA = 150.0

# "Not contributing while the partner works": this motor under CUTOUT_MA while
# its partner is over PULLING_MA.
CUTOUT_MA = 100.0
PULLING_MA = 400.0


@dataclass
class PartnerImbalance:
    """How differently the two motors on one side behave under load.

    They are mechanically coupled — same shaft speed, same command — so under a
    shared load their readings should track each other, which makes the healthy
    one a control for the sick one with no baseline run required.

    Two separate numbers, because measuring them together hides the interesting
    case. In 08012026-10 the right pair pooled to x1.94, which reads as "both
    motors driving, badly matched" — but split apart it is x1.10 while moving
    (as well matched as the left pair's x1.05) and a clean 0 mA vs 723 mA at
    STALL. One motor cutting out only under peak electrical stress is a
    different fault, and a different repair, from two motors sharing badly.
    """
    side: str
    n: int                        # loaded samples examined
    n_engaged: int                # of those, samples where BOTH motors drew current
    current_ratio: float          # max/min of medians over engaged samples; 1.0 = balanced
    power_ratio: float            # nan when the log has no W= column
    efficiency_gap: float         # percentage points, nan without eff=
    temp_gap: float               # deg C
    cutout_frac: float            # worst motor's share of loaded samples at ~0 mA
    cutout_port: int | None       # that motor's port
    cutout_key: tuple[str, int] | None
    worst_port: int | None        # port of the lower-median-current motor
    worst_key: tuple[str, int] | None


def partner_imbalance(session: HealthSession, side: str,
                      loaded_ma: float = LOADED_SIDE_MA) -> PartnerImbalance | None:
    """Compare the two motors on `side` over samples where the side is loaded.

    Returns None for a side that is not a coupled pair (a single motor has no
    control to compare against) or that never sees load in this session.
    """
    keys = [k for k in session.motors if k[0] == side]
    if len(keys) != 2:
        return None

    a = {s.t: s for s in session.by_motor(*keys[0])}
    b = {s.t: s for s in session.by_motor(*keys[1])}

    cur_a, cur_b, pw_a, pw_b, ef_a, ef_b, tp_a, tp_b = ([] for _ in range(8))
    n_loaded = 0
    cut_a = cut_b = 0
    for t in sorted(set(a) & set(b)):
        sa, sb = a[t], b[t]
        if not (math.isfinite(sa.current_ma) and math.isfinite(sb.current_ma)):
            continue
        if sa.current_ma + sb.current_ma < loaded_ma:
            continue
        n_loaded += 1

        # Cut-out: one motor contributing nothing while the other works.
        if sa.current_ma < CUTOUT_MA and sb.current_ma > PULLING_MA:
            cut_a += 1
        if sb.current_ma < CUTOUT_MA and sa.current_ma > PULLING_MA:
            cut_b += 1

        # Sharing ratio only over samples where BOTH are engaged; a cut-out
        # sample would otherwise divide by ~0 and swamp the median.
        if sa.current_ma < ENGAGED_MA or sb.current_ma < ENGAGED_MA:
            continue
        cur_a.append(sa.current_ma)
        cur_b.append(sb.current_ma)
        if sa.power_w is not None and sb.power_w is not None:
            pw_a.append(sa.power_w)
            pw_b.append(sb.power_w)
        if sa.efficiency_pct is not None and sb.efficiency_pct is not None:
            ef_a.append(sa.efficiency_pct)
            ef_b.append(sb.efficiency_pct)
        if math.isfinite(sa.temp_c) and math.isfinite(sb.temp_c):
            tp_a.append(sa.temp_c)
            tp_b.append(sb.temp_c)

    if not n_loaded:
        return None

    def ratio(xs: list[float], ys: list[float]) -> float:
        if not xs:
            return math.nan
        mx, my = statistics.median(xs), statistics.median(ys)
        lo, hi = sorted((abs(mx), abs(my)))
        return hi / lo if lo > 1e-9 else math.inf

    def gap(xs: list[float], ys: list[float]) -> float:
        if not xs:
            return math.nan
        return abs(statistics.median(xs) - statistics.median(ys))

    # The motor drawing LESS current under a shared load is the one not pulling
    # its weight, so it is the one to look at first.
    ports = session.ports()
    if cur_a:
        worst_key = keys[0] if statistics.median(cur_a) <= statistics.median(cur_b) else keys[1]
    else:
        worst_key = None
    cutout_key = keys[0] if cut_a >= cut_b else keys[1]

    return PartnerImbalance(
        side=side,
        n=n_loaded,
        n_engaged=len(cur_a),
        current_ratio=ratio(cur_a, cur_b),
        power_ratio=ratio(pw_a, pw_b),
        efficiency_gap=gap(ef_a, ef_b),
        temp_gap=gap(tp_a, tp_b),
        cutout_frac=max(cut_a, cut_b) / n_loaded,
        cutout_port=ports.get(cutout_key),
        cutout_key=cutout_key,
        worst_port=ports.get(worst_key) if worst_key else None,
        worst_key=worst_key,
    )


def all_partner_imbalances(session: HealthSession,
                           loaded_ma: float = LOADED_SIDE_MA) -> list[PartnerImbalance]:
    out = []
    for side in ("L", "R"):
        pi = partner_imbalance(session, side, loaded_ma)
        if pi is not None:
            out.append(pi)
    return out
