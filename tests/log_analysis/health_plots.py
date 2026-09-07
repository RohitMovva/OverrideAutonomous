"""Plots for the drivetrain health stream (see health_parser.py).

`health_dashboard(session)` draws a column per side and a row per metric, with
both of a side's motors OVERLAID in the same axes. That layout is the whole
point: the two motors on a side are mechanically coupled, so under a shared load
their traces should sit on top of each other, and a bad motor is a visible gap
rather than a number you have to look up a baseline for.

Rows adapt to what the log carries — W= and eff= arrived after the first
DRIVEHEALTH logs were captured, so older logs simply get fewer rows instead of
blank panels.
"""

from __future__ import annotations

import math

from .health_parser import HealthSession, all_partner_imbalances

# Distinguish the two motors of a pair; index 0 solid, index 1 dashed.
_COLORS = ["tab:blue", "tab:orange"]
_STYLES = ["-", "--"]


def _series(session: HealthSession, side: str, index: int, attr: str,
            t0: float) -> tuple[list[float], list[float]]:
    xs, ys = [], []
    for s in session.by_motor(side, index):
        v = getattr(s, attr)
        if v is None:
            continue
        xs.append(s.t - t0)
        ys.append(v)
    return xs, ys


def _shade_contexts(ax, session: HealthSession, t0: float):
    """Tint the driver-control stretches, so heat soak between autons is visible
    as such rather than looking like part of a run."""
    spans, start, prev = [], None, None
    for s in sorted(session.samples, key=lambda x: x.t):
        if s.context != prev:
            if prev == "driver" and start is not None:
                spans.append((start, s.t))
            start = s.t
            prev = s.context
    if prev == "driver" and start is not None:
        spans.append((start, session.samples[-1].t))
    for a, b in spans:
        ax.axvspan(a - t0, b - t0, color="tab:gray", alpha=0.10, lw=0)


def health_dashboard(session: HealthSession):
    """Per-motor current / power / efficiency / temperature, partners overlaid."""
    import matplotlib.pyplot as plt

    if not session.samples:
        raise ValueError("session has no DRIVEHEALTH samples to plot")

    t0 = min(s.t for s in session.samples)
    ports = session.ports()

    rows = [("current_ma", "current (mA)")]
    if session.has_power:
        rows.append(("power_w", "power (W)"))
        rows.append(("volts", "volts seen (P/I)"))
    if session.has_efficiency:
        rows.append(("efficiency_pct", "efficiency (%)"))
    rows.append(("temp_c", "temp (deg C)"))

    sides = [s for s in ("L", "R") if any(k[0] == s for k in session.motors)]

    fig, axes = plt.subplots(len(rows), len(sides), figsize=(6.5 * len(sides), 2.5 * len(rows)),
                             sharex=True, squeeze=False)

    for col, side in enumerate(sides):
        idxs = sorted(i for (sd, i) in session.motors if sd == side)
        for row, (attr, ylabel) in enumerate(rows):
            ax = axes[row][col]
            _shade_contexts(ax, session, t0)
            for n, i in enumerate(idxs):
                xs, ys = _series(session, side, i, attr, t0)
                if not xs:
                    continue
                port = ports.get((side, i))
                ax.plot(xs, ys, color=_COLORS[n % len(_COLORS)],
                        ls=_STYLES[n % len(_STYLES)], lw=1.1, alpha=0.85,
                        label=f"{side}{i}" + (f" port {port}" if port is not None else ""))
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.3)
            if row == 0:
                ax.set_title(f"{'Left' if side == 'L' else 'Right'} side")
                ax.legend(loc="upper right", fontsize=8)
        axes[-1][col].set_xlabel("time (s, relative to first health sample)")

    # Verdict box: the number you actually came for.
    lines = []
    for pi in all_partner_imbalances(session):
        bits = [f"engaged current x{pi.current_ratio:.2f}"]
        if math.isfinite(pi.power_ratio):
            bits.append(f"power x{pi.power_ratio:.2f}")
        if math.isfinite(pi.efficiency_gap):
            bits.append(f"eff gap {pi.efficiency_gap:.0f}pp")
        bits.append(f"cut-out {pi.cutout_frac * 100:.0f}% on port {pi.cutout_port}")
        flag = "  <-- CHECK" if (pi.current_ratio > 1.4 or pi.cutout_frac > 0.10) else ""
        lines.append(f"{pi.side} pair (n={pi.n}, {pi.n_engaged} engaged): "
                     + ", ".join(bits) + flag)
    if lines:
        fig.text(0.5, 0.005, "   |   ".join(lines), ha="center", fontsize=8,
                 family="monospace")

    n_auton = sum(1 for s in session.samples if s.context == "auton")
    fig.suptitle(f"Drive health — boot #{session.index}  "
                 f"({len(session.samples)} samples, {n_auton} auton / "
                 f"{len(session.samples) - n_auton} driver; shaded = driver)")
    fig.tight_layout(rect=(0, 0.03, 1, 0.97))
    return fig
