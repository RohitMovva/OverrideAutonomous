"""Static matplotlib figures for parsed robot-log runs.

Every function takes a Run and returns a matplotlib Figure; saving/showing
is the caller's job (see tests/visualize.py). Replaces the legacy scripts
in tests/log_analyis/ (error_visualizer, theta_grapher, velocity_grapher,
velocity_tuner, drivetrain_readings_visualizer) with one shared parser and
consistent styling.
"""

from __future__ import annotations

import math

import matplotlib.pyplot as plt
import numpy as np

from . import metrics
from .parser import Run

FIELD_HALF = 70.7  # inches, field wall coordinate (see DistanceResetOdometry)


def _rel_times(ticks):
    t0 = ticks[0].t
    return [tk.t - t0 for tk in ticks]


def _title(run: Run, what: str) -> str:
    return f"{what} — run {run.label} ({len(run.ticks)} ticks)"


# ------------------------------------------------------------------ tracking

def tracking_figure(run: Run) -> plt.Figure:
    """Positional + angular tracking error vs time (was error_visualizer.py)."""
    ts, pos_err, ang_err = [], [], []
    t0 = run.ticks[0].t
    for tk in run.ticks:
        if tk.pose is None or tk.goal is None:
            continue
        ts.append(tk.t - t0)
        pos_err.append(math.hypot(tk.goal[0] - tk.pose[0], tk.goal[1] - tk.pose[1]))
        e = tk.goal[2] - tk.pose[2]
        while e > math.pi:
            e -= 2 * math.pi
        while e < -math.pi:
            e += 2 * math.pi
        ang_err.append(math.degrees(e))

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(12, 7))
    ax1.plot(ts, ang_err, color="tab:green", lw=0.9)
    ax1.axhline(0, color="gray", lw=0.5)
    ax1.set_ylabel("heading error (deg)")
    ax1.grid(alpha=0.3)
    ax2.plot(ts, pos_err, color="tab:red", lw=0.9)
    ax2.set_ylabel("position error (in)")
    ax2.set_xlabel("time (s)")
    ax2.grid(alpha=0.3)
    if pos_err:
        stats = (f"pos: mean {np.mean(pos_err):.2f}  max {np.max(pos_err):.2f} in\n"
                 f"ang: mean |e| {np.mean(np.abs(ang_err)):.2f}  "
                 f"max |e| {np.max(np.abs(ang_err)):.2f} deg")
        ax2.text(0.99, 0.95, stats, transform=ax2.transAxes, ha="right", va="top",
                 fontsize=9, bbox=dict(boxstyle="round", fc="white", alpha=0.8))
    fig.suptitle(_title(run, "Tracking error"))
    fig.tight_layout()
    return fig


def theta_figure(run: Run) -> plt.Figure:
    """Actual vs goal heading and signed error (was theta_grapher.py)."""
    ts, act, goal, err = [], [], [], []
    t0 = run.ticks[0].t
    for tk in run.ticks:
        if tk.pose is None or tk.goal is None:
            continue
        ts.append(tk.t - t0)
        act.append(tk.pose[2])
        goal.append(tk.goal[2])
        e = tk.goal[2] - tk.pose[2]
        while e > math.pi:
            e -= 2 * math.pi
        while e < -math.pi:
            e += 2 * math.pi
        err.append(e)

    fig, ax = plt.subplots(figsize=(12, 6))
    ax.plot(ts, act, "b-", lw=1.0, label="actual θ")
    ax.plot(ts, goal, "r--", lw=1.0, label="goal θ")
    ax.plot(ts, err, "g:", lw=1.0, label="error (goal − actual)")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("theta (rad)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")
    st = metrics.theta_tracking(run)
    if st.get("n"):
        ax.set_title(f"|err| mean {st['mean']:.2f}° max {st['max']:.2f}° "
                     f"p95 {st['p95']:.2f}° | signed mean {st.get('mean_signed_deg', 0):.2f}°",
                     fontsize=10)
    fig.suptitle(_title(run, "Heading tracking"))
    fig.tight_layout()
    return fig


# ------------------------------------------------------------------ outputs

def outputs_figure(run: Run) -> plt.Figure:
    """MP reference vs RAMSETE output (was velocity_grapher.py)."""
    ts, mp_v, rs_v, mp_w, rs_w = [], [], [], [], []
    t0 = run.ticks[0].t
    for tk in run.ticks:
        if None in (tk.mp_v, tk.ramsete_v, tk.mp_w, tk.ramsete_w):
            continue
        ts.append(tk.t - t0)
        mp_v.append(tk.mp_v)
        rs_v.append(tk.ramsete_v)
        mp_w.append(tk.mp_w)
        rs_w.append(tk.ramsete_w)

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(12, 7))
    if not ts:
        fig.text(0.5, 0.5, "log has no MP/Ramsete Output lines", ha="center")
        return fig
    ax1.plot(ts, mp_v, "b-", lw=1.0, label="MP (profile)")
    ax1.plot(ts, rs_v, "r--", lw=1.0, label="RAMSETE")
    ax1.set_ylabel("linear vel (in/s)")
    ax1.legend(loc="upper right")
    ax1.grid(alpha=0.3)
    ax2.plot(ts, mp_w, "b-", lw=1.0, label="MP (profile)")
    ax2.plot(ts, rs_w, "r--", lw=1.0, label="RAMSETE")
    ax2.set_ylabel("angular vel (rad/s)")
    ax2.set_xlabel("time (s)")
    ax2.legend(loc="upper right")
    ax2.grid(alpha=0.3)
    fig.suptitle(_title(run, "MP vs RAMSETE output"))
    fig.tight_layout()
    return fig


# ------------------------------------------------------------------ wheels

def wheels_figure(run: Run, integral: bool = False) -> plt.Figure:
    """Wheel velocity setpoint vs measured, with voltage on a twin axis
    (was velocity_tuner.py). integral=True plots cumulative distance."""
    ts, sp_l, ac_l, sp_r, ac_r, v_l, v_r = [], [], [], [], [], [], []
    t0 = run.ticks[0].t
    for tk in run.ticks:
        if None in (tk.left_vel_setpoint, tk.left_vel_actual,
                    tk.right_vel_setpoint, tk.right_vel_actual):
            continue
        ts.append(tk.t - t0)
        sp_l.append(tk.left_vel_setpoint)
        ac_l.append(tk.left_vel_actual)
        sp_r.append(tk.right_vel_setpoint)
        ac_r.append(tk.right_vel_actual)
        v_l.append(tk.voltage_left if tk.voltage_left is not None else np.nan)
        v_r.append(tk.voltage_right if tk.voltage_right is not None else np.nan)

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(12, 8))
    if not ts:
        fig.text(0.5, 0.5, "log has no wheel velocity lines", ha="center")
        return fig

    if integral:
        dts = np.diff(ts, prepend=ts[0])
        dts = np.clip(dts, 0, 0.1)
        series = [(np.cumsum(np.array(sp_l) * dts), np.cumsum(np.array(ac_l) * dts)),
                  (np.cumsum(np.array(sp_r) * dts), np.cumsum(np.array(ac_r) * dts))]
        ylabel = "distance (in)"
    else:
        series = [(sp_l, ac_l), (sp_r, ac_r)]
        ylabel = "velocity (in/s)"

    for ax, (sp, ac), volts, side in ((ax1, series[0], v_l, "left"),
                                      (ax2, series[1], v_r, "right")):
        ax.plot(ts, sp, "b-", lw=0.9, label=f"{side} commanded")
        ax.plot(ts, ac, "r-", lw=0.9, alpha=0.8, label=f"{side} measured")
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.3)
        axv = ax.twinx()
        axv.plot(ts, volts, color="gray", lw=0.6, alpha=0.6, label="voltage")
        axv.set_ylabel("voltage (motor units)", color="gray")
        axv.set_ylim(-140, 140)
        ax.legend(loc="upper left", fontsize=8)
    ax2.set_xlabel("time (s)")
    what = "Wheel distance (∫v dt)" if integral else "Wheel velocity tracking"
    fig.suptitle(_title(run, what))
    fig.tight_layout()
    return fig


# ------------------------------------------------------------------ encoders

def encoders_figure(run: Run) -> plt.Figure:
    """Per-motor encoder divergence and cumulative side positions
    (was drivetrain_readings_visualizer.py).

    The top panel plots each motor's travel *relative to its own first sample*,
    minus the side's mean. 'Left/Right Positions' are absolute untared counts,
    so plotting them raw just shows one thick line per side (the position
    dominates) with the per-motor offsets baked in; the residual is what
    actually shows a motor drifting away from its partners.
    """
    from .metrics import encoder_disagreement

    ts_pos, lefts, rights = [], [], []
    ts_cum, cum_l, cum_r = [], [], []
    t0 = run.ticks[0].t
    for tk in run.ticks:
        if tk.left_positions and tk.right_positions:
            ts_pos.append(tk.t - t0)
            lefts.append(tk.left_positions)
            rights.append(tk.right_positions)
        if tk.cum_left is not None:
            ts_cum.append(tk.t - t0)
            cum_l.append(tk.cum_left)
            cum_r.append(tk.cum_right)

    ed = encoder_disagreement(run)

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(12, 8))
    if ts_pos:
        for side, rows, ls in (("left", lefts, "-"), ("right", rights, "--")):
            live = ed[side].get("channels")
            if not live:
                continue
            arr = np.array([[r[i] for i in live] for r in rows], dtype=float)
            arr = arr - arr[0]                      # drop the untared offset
            resid = arr - arr.mean(axis=1, keepdims=True)
            for col, motor in enumerate(live):
                ax1.plot(ts_pos, resid[:, col], lw=0.9, alpha=0.8, ls=ls,
                         label=f"{side[0].upper()} motor {motor}")
        ax1.axhline(0, color="gray", lw=0.6, alpha=0.6)
        ax1.set_ylabel("ticks from side mean\n(offset removed)")
        ax1.legend(loc="upper right", fontsize=7, ncol=2)
        subs = []
        for side in ("left", "right"):
            sf = ed[side].get("slip_fraction")
            if sf is not None and not np.isnan(sf):
                subs.append(f"{side} {100*sf:.2f}% of travel")
        if subs:
            ax1.set_title("net divergence: " + ", ".join(subs), fontsize=9)
    else:
        ax1.text(0.5, 0.5, "log has no Left/Right Positions lines",
                 ha="center", transform=ax1.transAxes)
    ax1.grid(alpha=0.3)
    if ts_cum:
        ax2.plot(ts_cum, cum_l, "b-", lw=1.0, label="left avg position")
        ax2.plot(ts_cum, cum_r, "r-", lw=1.0, label="right avg position")
        ax2.legend(loc="upper left")
    ax2.set_ylabel("cumulative position (in)")
    ax2.set_xlabel("time (s)")
    ax2.grid(alpha=0.3)
    fig.suptitle(_title(run, "Drivetrain encoders"))
    fig.tight_layout()
    return fig


# ------------------------------------------------------------------ field

def _draw_field(ax):
    ax.add_patch(plt.Rectangle((-FIELD_HALF, -FIELD_HALF), 2 * FIELD_HALF,
                               2 * FIELD_HALF, fill=False, color="black", lw=1.2))
    ax.axhline(0, color="gray", lw=0.4, alpha=0.5)
    ax.axvline(0, color="gray", lw=0.4, alpha=0.5)
    ax.set_aspect("equal")
    ax.set_xlabel("x (in)")
    ax.set_ylabel("y (in)")


def field_figure(run: Run, sim: dict | None = None, sim_label: str = "") -> plt.Figure:
    """Overhead XY view: goal path vs actual path, resets marked.
    Optionally overlays a dead-reckoned path from metrics.dead_reckon()."""
    gx, gy, px, py = [], [], [], []
    for tk in run.ticks:
        if tk.goal is not None:
            gx.append(tk.goal[0])
            gy.append(tk.goal[1])
        if tk.pose is not None:
            px.append(tk.pose[0])
            py.append(tk.pose[1])

    fig, ax = plt.subplots(figsize=(9, 9))
    _draw_field(ax)
    ax.plot(gx, gy, "r--", lw=1.2, label="goal path")
    ax.plot(px, py, "b-", lw=1.2, label="actual path")
    if px:
        ax.plot(px[0], py[0], "go", ms=9, label="start")
        ax.plot(px[-1], py[-1], "bs", ms=9, label="end (actual)")
    if gx:
        ax.plot(gx[-1], gy[-1], "r*", ms=13, label="end (goal)")
    for reset in run.resets:
        i = reset.tick_index
        if 0 <= i < len(run.ticks) and run.ticks[i].pose is not None:
            ax.plot(run.ticks[i].pose[0], run.ticks[i].pose[1], "kx", ms=11,
                    mew=2.5, label="distance reset" if reset is run.resets[0] else None)
    if sim and sim.get("n"):
        sx = [p[0] for p in sim["sim_path"]]
        sy = [p[1] for p in sim["sim_path"]]
        ax.plot(sx, sy, "g-", lw=1.0, alpha=0.8,
                label=sim_label or "dead-reckoned")
    ax.legend(loc="upper right", fontsize=9)
    fig.suptitle(_title(run, "Field view"))
    fig.tight_layout()
    return fig


# ------------------------------------------------------------------ dashboard

def dashboard_figure(run: Run) -> plt.Figure:
    """One-page summary: field, tracking error, wheel velocities, loop
    timing histogram, voltage trace, and headline metrics."""
    fig = plt.figure(figsize=(16, 11))
    gs = fig.add_gridspec(3, 3, hspace=0.35, wspace=0.3)

    t0 = run.ticks[0].t

    # Field (left column, tall)
    ax_field = fig.add_subplot(gs[0:2, 0])
    _draw_field(ax_field)
    gx = [tk.goal[0] for tk in run.ticks if tk.goal]
    gy = [tk.goal[1] for tk in run.ticks if tk.goal]
    px = [tk.pose[0] for tk in run.ticks if tk.pose]
    py = [tk.pose[1] for tk in run.ticks if tk.pose]
    ax_field.plot(gx, gy, "r--", lw=1.0, label="goal")
    ax_field.plot(px, py, "b-", lw=1.0, label="actual")
    for reset in run.resets:
        i = reset.tick_index
        if 0 <= i < len(run.ticks) and run.ticks[i].pose is not None:
            ax_field.plot(run.ticks[i].pose[0], run.ticks[i].pose[1], "kx", ms=10, mew=2)
    ax_field.legend(fontsize=8)
    ax_field.set_title("field", fontsize=10)

    # Tracking error
    ax_err = fig.add_subplot(gs[0, 1:])
    ts, perr = [], []
    for tk in run.ticks:
        if tk.pose and tk.goal:
            ts.append(tk.t - t0)
            perr.append(math.hypot(tk.goal[0] - tk.pose[0], tk.goal[1] - tk.pose[1]))
    ax_err.plot(ts, perr, "r-", lw=0.8)
    for reset in run.resets:
        ax_err.axvline(reset.t - t0, color="k", ls=":", lw=1)
    ax_err.set_title("position error (in); dotted = distance reset", fontsize=10)
    ax_err.grid(alpha=0.3)

    # Wheel velocities
    ax_vel = fig.add_subplot(gs[1, 1:])
    tv, sp, ac = [], [], []
    for tk in run.ticks:
        if tk.left_vel_setpoint is not None and tk.left_vel_actual is not None:
            tv.append(tk.t - t0)
            sp.append(tk.left_vel_setpoint)
            ac.append(tk.left_vel_actual)
    ax_vel.plot(tv, sp, "b-", lw=0.7, label="L commanded")
    ax_vel.plot(tv, ac, "r-", lw=0.7, alpha=0.8, label="L measured")
    ax_vel.legend(fontsize=8)
    ax_vel.set_title("left wheel velocity (in/s)", fontsize=10)
    ax_vel.grid(alpha=0.3)

    # Loop period histogram
    ax_dt = fig.add_subplot(gs[2, 0])
    periods = [1000 * (b.t - a.t) for a, b in zip(run.ticks, run.ticks[1:])]
    if periods:
        ax_dt.hist(periods, bins=40, color="tab:blue")
        ax_dt.axvline(10, color="r", ls="--", lw=1)
        ax_dt.set_yscale("log")
    ax_dt.set_title("loop period (ms), red = nominal", fontsize=10)

    # Voltages
    ax_v = fig.add_subplot(gs[2, 1])
    tvv = [tk.t - t0 for tk in run.ticks if tk.voltage_left is not None]
    vl = [tk.voltage_left for tk in run.ticks if tk.voltage_left is not None]
    vr = [tk.voltage_right for tk in run.ticks if tk.voltage_right is not None]
    ax_v.plot(tvv, vl, lw=0.6, label="L")
    ax_v.plot(tvv, vr, lw=0.6, label="R")
    ax_v.axhline(127, color="r", ls="--", lw=0.7)
    ax_v.axhline(-127, color="r", ls="--", lw=0.7)
    ax_v.legend(fontsize=8)
    ax_v.set_title("motor command (±127 = saturated)", fontsize=10)
    ax_v.grid(alpha=0.3)

    # Headline metrics text
    ax_txt = fig.add_subplot(gs[2, 2])
    ax_txt.axis("off")
    te = metrics.tracking_errors(run)
    tm = metrics.loop_timing(run)
    st = metrics.saturation(run)
    tw = metrics.effective_track_width(run)
    lines = []
    if te["cross_track"].get("n"):
        lines.append(f"cross-track rms {te['cross_track']['rms']:.2f} in, "
                     f"max {te['cross_track']['max']:.2f} in")
        lines.append(f"final err {te['final_position_error_in']:.2f} in / "
                     f"{te['final_heading_error_rad']:.3f} rad")
    if tm.get("n"):
        lines.append(f"loop overruns {tm['overrun_count']} "
                     f"({100 * tm['overrun_fraction']:.1f}%), "
                     f"worst {tm['worst_stall_ms']:.0f} ms")
    if st.get("n"):
        lines.append(f"saturation L {100 * st['left_fraction']:.0f}% / "
                     f"R {100 * st['right_fraction']:.0f}%")
    if tw.get("estimate_in"):
        lines.append(f"track width est {tw['estimate_in']:.1f} in "
                     f"(config {tw['nominal_in']:.1f})")
    lines.append(f"resets: {len(run.resets)}, warnings: {len(run.warnings)}")
    ax_txt.text(0, 0.95, "\n".join(lines), va="top", fontsize=10, family="monospace")

    fig.suptitle(_title(run, "Dashboard"), fontsize=14)
    return fig


# ------------------------------------------------------------------ gains

def gains_figure(runs: Run | list[Run], label: str = "") -> plt.Figure:
    """Feedforward calibration & feasibility view for the regime/gain metrics
    (motion_regimes, steady_state_gain, setpoint_feasibility, saturation_cost).

    Pools the given runs — regimes like in-place turns are sparse within any
    single run, so this is usually generated once per log file.
    """
    runs_list = runs if isinstance(runs, list) else [runs]
    mr = metrics.motion_regimes(runs_list, samples=True)
    ss = metrics.steady_state_gain(runs_list, samples=True)
    sf = metrics.setpoint_feasibility(runs_list, samples=True)
    sc = metrics.saturation_cost(runs_list, samples=True)

    fig, axes = plt.subplots(2, 2, figsize=(15, 11))
    ax_scatter, ax_ss, ax_feas, ax_cost = axes.flat

    # 1. actual vs commanded wheel speed, colored by motion regime
    colors = {"straight": "tab:blue", "arc": "tab:orange", "turn": "tab:red"}
    lim = 0.0
    for regime, color in colors.items():
        pts = mr[regime].get("samples", [])
        if not pts:
            continue
        sp = [p[0] for p in pts]
        ac = [p[1] for p in pts]
        lim = max(lim, max(abs(v) for v in sp + ac))
        g = mr[regime]["gain"]
        gtxt = f" (gain {g:.2f})" if g is not None else ""
        ax_scatter.scatter(sp, ac, s=3, alpha=0.25, color=color,
                           label=f"{regime}{gtxt}, n={mr[regime]['n']}")
    lim = lim * 1.05 or 60.0
    ax_scatter.plot([-lim, lim], [-lim, lim], "k--", lw=1, label="ideal (actual = commanded)")
    ax_scatter.set_xlabel("commanded wheel speed (in/s)")
    ax_scatter.set_ylabel("measured wheel speed (in/s)")
    ax_scatter.set_title("delivered vs commanded by motion regime", fontsize=10)
    ax_scatter.legend(fontsize=8, loc="upper left")
    ax_scatter.grid(alpha=0.3)

    # 2. steady-state gain distribution per side
    any_ss = False
    for side, color in (("left", "tab:blue"), ("right", "tab:red")):
        xs = ss[side].get("samples", [])
        if not xs:
            continue
        any_ss = True
        g = ss[side]["gain"]
        ax_ss.hist(xs, bins=60, range=(0.7, 1.3), alpha=0.55, color=color,
                   label=f"{side} (median {g:.3f}, n={ss[side]['n']})")
    ax_ss.axvline(1.0, color="k", ls="--", lw=1)
    asym = ss.get("asymmetry")
    sub = f"L/R asymmetry {100*asym:.1f}%" if asym is not None else "no settled stretches"
    ax_ss.set_title(f"steady-state measured/commanded ratio — {sub}", fontsize=10)
    ax_ss.set_xlabel("gain on settled cruise (1.0 = perfect kV)")
    if any_ss:
        ax_ss.legend(fontsize=8)
    ax_ss.grid(alpha=0.3)

    # 3. setpoint feasibility
    mags = sf.get("samples", [])
    if mags:
        ax_feas.hist(mags, bins=80, color="tab:blue")
        ax_feas.set_yscale("log")
    ax_feas.axvline(sf["vmax_in_s"], color="r", ls="--", lw=1.2,
                    label=f"physical max ≈ {sf['vmax_in_s']:.0f} in/s")
    ax_feas.set_title(
        f"commanded wheel speeds — {100*sf['infeasible_fraction']:.1f}% infeasible, "
        f"worst {sf['max_setpoint_in_s']:.0f} in/s", fontsize=10)
    ax_feas.set_xlabel("|commanded wheel speed| (in/s)")
    ax_feas.legend(fontsize=8)
    ax_feas.grid(alpha=0.3)

    # 4. saturation cost
    sat = sc["saturated"].get("samples", [])
    unsat = sc["unsaturated"].get("samples", [])
    if sat or unsat:
        hi = max(sat + unsat)
        bins = np.linspace(0, min(hi, 60), 60)
        if unsat:
            ax_cost.hist(unsat, bins=bins, alpha=0.6, color="tab:green", density=True,
                         label=f"unsaturated (mean {sc['unsaturated']['mean']:.1f} in/s, "
                               f"n={sc['unsaturated']['n']})")
        if sat:
            ax_cost.hist(sat, bins=bins, alpha=0.6, color="tab:red", density=True,
                         label=f"saturated (mean {sc['saturated']['mean']:.1f} in/s, "
                               f"n={sc['saturated']['n']})")
        ax_cost.legend(fontsize=8)
    ratio = sc.get("mean_ratio")
    rtxt = f"{ratio:.1f}x worse when saturated" if ratio else "no saturated ticks"
    ax_cost.set_title(f"|velocity error| by voltage saturation — {rtxt}", fontsize=10)
    ax_cost.set_xlabel("|commanded − measured| (in/s)")
    ax_cost.grid(alpha=0.3)

    n_ticks = sum(len(r.ticks) for r in runs_list)
    fig.suptitle(f"Gain & feasibility — {label or 'pooled'} "
                 f"({len(runs_list)} runs, {n_ticks} ticks)", fontsize=14)
    fig.tight_layout()
    return fig


FIGURES = {
    "tracking": tracking_figure,
    "theta": theta_figure,
    "outputs": outputs_figure,
    "wheels": wheels_figure,
    "encoders": encoders_figure,
    "field": field_figure,
    "dashboard": dashboard_figure,
}

# Pooled (per-file) figures: take a list of runs instead of a single run
POOLED_FIGURES = {
    "gains": gains_figure,
}
