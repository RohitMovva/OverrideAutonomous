"""Plots for the lift/chainbar debug stream (see lift_parser.py).

`lift_dashboard(session)` draws one figure with a row per actuator (winch,
chainbar, end effector): desired vs. actual on the left axis and the commanded
motor voltage on a twin axis, with command/transition markers overlaid. This is
the view for tuning the position PIDs, the chainbar gravity feed-forward, and
the respool stall threshold against real target-vs-actual data.
"""

from __future__ import annotations

from .lift_parser import LiftSession


def _rel_times(session: LiftSession) -> list[float]:
    t0 = session.ticks[0].t
    return [tk.t - t0 for tk in session.ticks]


def _overlay_events(ax, session: LiftSession, t0: float):
    """Draw vertical markers for commands (solid) and transitions (dashed)."""
    for e in session.events:
        if e.t < t0:
            continue
        is_cmd = e.kind == "CMD"
        ax.axvline(e.t - t0, color="tab:purple" if is_cmd else "tab:gray",
                   ls="-" if is_cmd else "--", lw=1.0, alpha=0.5)


def lift_dashboard(session: LiftSession):
    """Winch / bar / EE desired-vs-actual + voltage, with event markers."""
    import matplotlib.pyplot as plt

    if not session.ticks:
        raise ValueError("session has no LIFTDBG ticks to plot")

    t = _rel_times(session)
    t0 = session.ticks[0].t

    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)

    rows = [
        ("Winch (motor deg)", "winch_tgt", "winch_act", "winch_mv", None,
         axes[0]),
        ("Chainbar (output deg)", "bar_tgt", "bar_act", "bar_mv", "bar_ff",
         axes[1]),
        ("End effector (motor deg)", "ee_tgt", "ee_act", "ee_mv", None,
         axes[2]),
    ]

    for title, tgt, act, mv, ff, ax in rows:
        tgt_v = [getattr(tk, tgt) for tk in session.ticks]
        act_v = [getattr(tk, act) for tk in session.ticks]
        mv_v = [getattr(tk, mv) for tk in session.ticks]

        ax.plot(t, tgt_v, color="tab:blue", lw=1.6, label="target")
        ax.plot(t, act_v, color="tab:orange", lw=1.4, label="actual")
        ax.set_ylabel(title)
        ax.grid(True, alpha=0.3)

        axv = ax.twinx()
        axv.plot(t, mv_v, color="tab:green", lw=0.9, alpha=0.6, label="cmd mV")
        if ff is not None:
            ff_v = [getattr(tk, ff) for tk in session.ticks]
            axv.plot(t, ff_v, color="tab:red", lw=0.9, alpha=0.5, ls=":",
                     label="gravity ff mV")
        axv.set_ylabel("mV", color="tab:green")
        axv.tick_params(axis="y", labelcolor="tab:green")

        _overlay_events(ax, session, t0)

        # Merge legends from both axes.
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = axv.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, loc="upper right", fontsize=8, ncol=2)

    # Annotate the commands along the top so you can see what was requested when.
    top = axes[0]
    ymax = top.get_ylim()[1]
    for e in session.commands:
        if e.t < t0:
            continue
        top.annotate(e.text.split(" ->")[0].split(" (")[0],
                     xy=(e.t - t0, ymax), fontsize=7, rotation=90,
                     va="top", ha="right", color="tab:purple", alpha=0.8)

    axes[-1].set_xlabel("time (s, relative to first lift sample)")
    n_resp = sum(1 for e in session.transitions if e.text.startswith("respool"))
    fig.suptitle(f"Lift debug — boot #{session.index}  "
                 f"({len(session.ticks)} samples, {len(session.commands)} commands, "
                 f"{n_resp} respool events)")
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    return fig
