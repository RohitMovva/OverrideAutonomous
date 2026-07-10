"""Field animations for parsed robot-log runs.

Replaces robot_animator.py, goal_animator.py, velo_checker.py and the
rotating-footprint renderer from the old tests/test.py. Each builder
returns (fig, FuncAnimation); the caller shows it or saves a GIF.
"""

from __future__ import annotations

import math

import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

from . import metrics
from .parser import Run
from .plots import _draw_field, _title

ROBOT_SIZE = 15.0  # inches, square footprint


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _pose_arrays(run: Run):
    ticks = [tk for tk in run.ticks if tk.pose is not None and tk.goal is not None]
    pose = np.array([tk.pose for tk in ticks])
    goal = np.array([tk.goal for tk in ticks])
    ts = np.array([tk.t - ticks[0].t for tk in ticks]) if ticks else np.array([])
    return ts, pose, goal


def _setup_axes(run: Run, what: str):
    fig, ax = plt.subplots(figsize=(9, 9))
    _draw_field(ax)
    ax.set_xlim(-FIELD_LIM, FIELD_LIM)
    ax.set_ylim(-FIELD_LIM, FIELD_LIM)
    fig.suptitle(_title(run, what))
    return fig, ax


FIELD_LIM = 75.0


def _animate_paths(run: Run, what: str, actual: np.ndarray, goal: np.ndarray,
                   ts: np.ndarray, actual_label: str, interval_ms: int = 25,
                   step: int = 1, extra_text=None):
    """Shared animation core: goal path vs an 'actual' path with a rotating
    robot footprint, heading arrow (blue), and travel-direction arrow (green,
    flipped when driving in reverse — robot_animator.py behavior)."""
    fig, ax = _setup_axes(run, what)
    ax.plot(goal[:, 0], goal[:, 1], "r--", lw=1.0, alpha=0.6, label="goal path")

    trail, = ax.plot([], [], "b-", lw=1.2, label=actual_label)
    footprint = patches.Rectangle((0, 0), ROBOT_SIZE, ROBOT_SIZE,
                                  rotation_point="center", fc="lightgray",
                                  ec="black", alpha=0.85)
    ax.add_patch(footprint)
    heading_arrow = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                                arrowprops=dict(color="tab:blue", width=2, headwidth=8))
    travel_arrow = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                               arrowprops=dict(color="tab:green", width=2, headwidth=8))
    goal_pt, = ax.plot([], [], "r*", ms=12)
    info = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", fontsize=9,
                   family="monospace",
                   bbox=dict(boxstyle="round", fc="white", alpha=0.8))
    ax.legend(loc="upper right", fontsize=9)

    frames = range(0, len(actual), step)

    def update(i):
        x, y, th = actual[i]
        trail.set_data(actual[: i + 1, 0], actual[: i + 1, 1])
        footprint.set_xy((x - ROBOT_SIZE / 2, y - ROBOT_SIZE / 2))
        footprint.set_angle(math.degrees(th))
        alen = 12.0
        heading_arrow.set_position((x, y))
        heading_arrow.xy = (x + alen * math.cos(th), y + alen * math.sin(th))
        # travel direction from recent displacement; flip when reversing
        if i > 0:
            dx, dy = actual[i, 0] - actual[i - 1, 0], actual[i, 1] - actual[i - 1, 1]
            if math.hypot(dx, dy) > 0.01:
                tr = math.atan2(dy, dx)
                travel_arrow.set_position((x, y))
                travel_arrow.xy = (x + alen * math.cos(tr), y + alen * math.sin(tr))
        goal_pt.set_data([goal[i, 0]], [goal[i, 1]])
        perr = math.hypot(goal[i, 0] - x, goal[i, 1] - y)
        aerr = math.degrees(_wrap(goal[i, 2] - th))
        reversing = ""
        if i > 0:
            dx, dy = actual[i, 0] - actual[i - 1, 0], actual[i, 1] - actual[i - 1, 1]
            if math.hypot(dx, dy) > 0.01 and abs(_wrap(math.atan2(dy, dx) - th)) > math.pi / 2:
                reversing = "  [REVERSING]"
        txt = (f"t = {ts[i]:6.2f} s{reversing}\n"
               f"pos err = {perr:6.2f} in\n"
               f"ang err = {aerr:6.1f} deg")
        if extra_text is not None:
            txt += "\n" + extra_text(i)
        info.set_text(txt)
        return trail, footprint, heading_arrow, travel_arrow, goal_pt, info

    anim = animation.FuncAnimation(fig, update, frames=frames,
                                   interval=interval_ms, blit=False, repeat=True)
    return fig, anim


def robot_animation(run: Run, step: int = 2):
    """Logged pose vs goal path (was robot_animator.py + test.py footprint)."""
    ts, pose, goal = _pose_arrays(run)
    if len(pose) == 0:
        raise ValueError("run has no pose/goal ticks")
    return _animate_paths(run, "Actual pose vs goal", pose, goal, ts,
                          "actual path", step=step)


def mp_animation(run: Run, step: int = 2):
    """Dead-reckoned path from MP Output chassis velocities vs goal
    (was goal_animator.py). Requires MP Output lines (square/init logs)."""
    sim = metrics.dead_reckon(run, source="mp")
    if not sim.get("n"):
        raise ValueError("run has no MP Output data to integrate")
    ts, _, _ = _pose_arrays(run)
    actual = np.array(sim["sim_path"])
    goal = np.array(sim["goal_path"])
    n = min(len(actual), len(goal), len(ts))
    return _animate_paths(run, "MP dead-reckoned vs goal", actual[:n], goal[:n],
                          ts[:n], "integrated MP commands", step=step)


def wheels_animation(run: Run, source: str = "wheel_setpoints", step: int = 2,
                     track_width: float = 14.0):
    """Dead-reckoned path from wheel velocities via diff-drive kinematics vs
    goal (was velo_checker.py)."""
    sim = metrics.dead_reckon(run, source=source, track_width=track_width)
    if not sim.get("n"):
        raise ValueError("run has no wheel velocity data to integrate")
    ticks = [tk for tk in run.ticks if tk.goal is not None]
    ts = np.array([tk.t - ticks[0].t for tk in ticks])
    actual = np.array(sim["sim_path"])
    goal = np.array(sim["goal_path"])
    n = min(len(actual), len(goal), len(ts))

    def wheel_text(i):
        tk = ticks[min(i, len(ticks) - 1)]
        l = tk.left_vel_setpoint if source == "wheel_setpoints" else tk.left_vel_actual
        r = tk.right_vel_setpoint if source == "wheel_setpoints" else tk.right_vel_actual
        return f"L = {l if l is not None else float('nan'):6.2f}  " \
               f"R = {r if r is not None else float('nan'):6.2f} in/s"

    label = "integrated wheel " + ("setpoints" if source == "wheel_setpoints" else "actuals")
    return _animate_paths(run, f"Wheel dead-reckoned vs goal (W={track_width}\")",
                          actual[:n], goal[:n], ts[:n], label, step=step,
                          extra_text=wheel_text)


ANIMATIONS = {
    "animate": robot_animation,
    "animate-mp": mp_animation,
    "animate-wheels": wheels_animation,
}
