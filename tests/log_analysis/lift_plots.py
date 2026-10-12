"""Plots for the lift debug stream (see lift_parser.py).

`lift_dashboard(session)` draws one figure with a row per actuator (winch, angler,
clamp): desired vs. actual on the left axis and the commanded motor voltage on a twin
axis, with the macro's own log lines overlaid as vertical markers. This is the view for
tuning the angler PID and the clamp timings against real target-vs-actual data, and for
spotting the two failures the prose lines cannot show on their own:

  * a winch driven at full mV whose `act` does not move -- a mechanical stall
  * an angler whose `act` never reaches `tgt` -- a claw swing that is being blocked

A fourth row carries the discrete context (mode, state, PTO ownership, tower distance),
because "which state was it in when the winch stopped moving" is the first question.
"""

from __future__ import annotations

from .lift_parser import LiftSession


def _rel_times(session: LiftSession) -> list[float]:
    t0 = session.ticks[0].t
    return [tk.t - t0 for tk in session.ticks]


def _overlay_events(ax, session: LiftSession, t0: float):
    """Vertical markers: macro boundaries solid, clamp commands dashed."""
    for e in session.events:
        if e.t < t0:
            continue
        ax.axvline(e.t - t0, color="tab:purple" if e.is_macro else "tab:gray",
                   ls="-" if e.is_macro else "--", lw=1.0, alpha=0.5)


def lift_dashboard(session: LiftSession):
    """Winch / angler / clamp-or-roller desired-vs-actual + voltage, with event markers."""
    import matplotlib.pyplot as plt

    if not session.ticks:
        raise ValueError("session has no LIFTDBG ticks to plot")

    t = _rel_times(session)
    t0 = session.ticks[0].t

    fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True,
                             height_ratios=[3, 3, 3, 2])

    # The claw has been a clamp motor, a roller, a clamp piston, and now a timed claw
    # motor. The angler was removed on 2026-10-09; its row stays empty on newer logs.
    first = session.ticks[0]
    has_clamp = first.clamp_tgt is not None
    rows = [
        ("Winch (motor deg)", "winch_tgt", "winch_act", "winch_mv", None, axes[0]),
        ("Angler (output deg)", "ang_tgt", "ang_act", "ang_mv", "ang_set", axes[1])
        if first.ang_act is not None
        else None,
        ("Claw motor (deg)", None, "clamp_act", "clamp_mv", None, axes[2])
        if first.clamp_mv is not None
        else ("Clamp (output deg)", None, "clamp_act", None, None, axes[2])
        if first.clamp_act is not None
        else ("Clamp piston (1 = closed)", None, "clamp_piston", None, None, axes[2])
        if has_clamp
        else ("Roller (mV)", None, "roller_mv", None, None, axes[2]),
    ]

    if rows[1] is None:
        axes[1].set_axis_off()
        axes[1].text(0.5, 0.5, "no angler (removed 2026-10-09)", ha="center", va="center",
                     transform=axes[1].transAxes, color="gray")
    for title, tgt, act, mv, extra, ax in (r for r in rows if r is not None):
        act_v = [getattr(tk, act) for tk in session.ticks]
        if tgt is not None:
            ax.plot(t, [getattr(tk, tgt) for tk in session.ticks],
                    color="tab:blue", lw=1.6, label="target")
        if extra is not None:
            ax.plot(t, [getattr(tk, extra) for tk in session.ticks],
                    color="tab:cyan", lw=1.0, ls="--", alpha=0.8, label="ramped setpoint")
        ax.plot(t, act_v, color="tab:orange", lw=1.4, label="actual")
        ax.set_ylabel(title)
        ax.grid(True, alpha=0.3)

        if mv is not None:
            axv = ax.twinx()
            axv.plot(t, [getattr(tk, mv) for tk in session.ticks],
                     color="tab:green", lw=0.9, alpha=0.6, label="cmd mV")
            axv.set_ylabel("mV", color="tab:green")
            axv.tick_params(axis="y", labelcolor="tab:green")
            h2, l2 = axv.get_legend_handles_labels()
        else:
            h2, l2 = [], []

        _overlay_events(ax, session, t0)
        h1, l1 = ax.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, loc="upper right", fontsize=8, ncol=2)

    # The clamp's target is a named position, so band it rather than plotting a number.
    if has_clamp:
        clamp_ax = axes[2]
        changes = [(0.0, session.ticks[0].clamp_tgt)]
        for rel, tk in zip(t, session.ticks):
            if tk.clamp_tgt != changes[-1][1]:
                changes.append((rel, tk.clamp_tgt))
        ymax = clamp_ax.get_ylim()[1]
        for rel, name in changes:
            clamp_ax.annotate(name, xy=(rel, ymax), fontsize=7, rotation=90,
                              va="top", ha="left", color="tab:blue", alpha=0.9)

    # Discrete context row: state changes, PTO ownership, tower sensor.
    ctx = axes[3]
    states = sorted({tk.state for tk in session.ticks})
    idx = {s: i for i, s in enumerate(states)}
    ctx.step(t, [idx[tk.state] for tk in session.ticks], where="post",
             color="tab:purple", lw=1.2)
    ctx.set_yticks(range(len(states)))
    ctx.set_yticklabels(states, fontsize=7)
    ctx.set_ylabel("state")
    ctx.grid(True, alpha=0.3)

    ctxv = ctx.twinx()
    ctxv.plot(t, [min(tk.tower, 200) for tk in session.ticks],
              color="tab:brown", lw=0.9, alpha=0.7, label="tower mm (clipped 200)")
    if any(tk.loader is not None for tk in session.ticks):
        ctxv.plot(t, [min(tk.loader, 200) if tk.loader is not None else float("nan")
                      for tk in session.ticks],
                  color="tab:olive", lw=0.9, alpha=0.7, label="loader mm (clipped 200)")
    # Shade any stretch where the winch was commanded without owning the PTO.
    for rel, tk in zip(t, session.ticks):
        if tk.winch_mv != 0 and not tk.own:
            ctx.axvspan(rel, rel + 0.01, color="tab:red", alpha=0.25, lw=0)
    ctxv.set_ylabel("distance mm", color="tab:brown")
    ctxv.tick_params(axis="y", labelcolor="tab:brown")
    ctxv.legend(loc="upper right", fontsize=8)

    # Annotate the macro lines along the top so you can see what was requested when.
    top = axes[0]
    ymax = top.get_ylim()[1]
    for e in session.macro_events:
        if e.t < t0:
            continue
        top.annotate(e.text.split(":")[0], xy=(e.t - t0, ymax), fontsize=7,
                     rotation=90, va="top", ha="right", color="tab:purple", alpha=0.8)

    axes[-1].set_xlabel("time (s, relative to first lift sample)")
    fig.suptitle(f"Lift debug — boot #{session.index}  "
                 f"({len(session.ticks)} samples, {len(session.macro_events)} macro events, "
                 f"{len(session.clamp_events) + len(session.roller_events)} clamp/roller commands)")
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    return fig
