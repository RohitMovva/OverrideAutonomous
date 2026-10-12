"""Parser and metric checks for the TRACKWHEEL stream, on synthetic logs.

These build small logs in the shape Odometry::update writes them and check the
parser and metrics.tracking_wheel_agreement read them correctly, independent of
which robot logs are in tests/logs/.
"""
from __future__ import annotations

import math

import pytest

from log_analysis import metrics
from log_analysis.parser import parse_log, trajectory_runs


def _log(tmp_path, rows):
    """rows: (dvert, dlat, ddrive, vok, lok[, rolled]) per tick; positions accumulate.

    The pose advances by the delta odometry used (dvert, or ddrive when dvert is nan);
    the wheel's absolute reading advances by `rolled` (default: the used delta), so a
    rejected delta can roll the wheel without moving the pose.
    """
    lines = ["[00:00:00.000] [INFO] Robot initialized",
             "[00:00:00.010] [INFO] Initial pose: 0.000000 0.000000 0.000000"]
    vert = lat = x = 0.0
    for i, (dvert, dlat, ddrive, vok, lok, *rolled) in enumerate(rows):
        ms = 20 + 10 * i
        stamp = f"[00:00:{ms // 1000:02d}.{ms % 1000:03d}]"
        used = ddrive if math.isnan(dvert) else dvert
        vert += rolled[0] if rolled else (0.0 if math.isnan(dvert) else dvert)
        lat += 0.0 if math.isnan(dlat) else dlat
        x += used
        lines.append(f"{stamp} [INFO] TRACKWHEEL vert={vert:f} lat={lat:f} dvert={dvert:f} "
                     f"dlat={dlat:f} ddrive={ddrive:f} vok={vok} lok={lok}")
        lines.append(f"{stamp} [INFO] Pose: {x:f} 0.000000 0.000000")
    path = tmp_path / "trackwheel.txt"
    path.write_text("\n".join(lines) + "\n")
    runs = trajectory_runs(parse_log(path))
    assert len(runs) == 1
    return runs[0]


def test_parser_attaches_trackwheel_to_tick(tmp_path):
    run = _log(tmp_path, [(0.5, 0.01, 0.52, 1, 1)] * 30)
    tw = run.ticks[0].tracking_wheel
    assert tw == pytest.approx({"vert": 0.5, "lat": 0.01, "dvert": 0.5, "dlat": 0.01,
                                "ddrive": 0.52, "vok": 1, "lok": 1})


def test_agreement_measures_slip(tmp_path):
    # Drive reports 4% more travel than the wheel: 200 ticks x 0.5 in = 100 in
    run = _log(tmp_path, [(0.5, 0.0, 0.52, 1, 1)] * 200)
    tw = metrics.tracking_wheel_agreement(run)
    assert tw["n"] == 200 and tw["vertical_fallback"] == 0
    assert tw["wheel_travel_in"] == pytest.approx(100.0)
    assert tw["slip_fraction"] == pytest.approx(0.04)
    assert tw["direction_dot"] > 0


def test_agreement_flags_reversed_sensor(tmp_path):
    run = _log(tmp_path, [(-0.5, 0.0, 0.5, 1, 1)] * 200)
    assert metrics.tracking_wheel_agreement(run)["direction_dot"] < 0


def test_agreement_counts_fallbacks_and_skips_their_deltas(tmp_path):
    rows = [(0.5, 0.0, 0.5, 1, 1)] * 100 + [(math.nan, math.nan, 0.5, 0, 0)] * 20
    tw = metrics.tracking_wheel_agreement(_log(tmp_path, rows))
    assert tw["ticks"] == 120 and tw["n"] == 100
    assert tw["vertical_fallback"] == 20 and tw["lateral_fallback"] == 20
    assert tw["wheel_net_in"] == pytest.approx(50.0)
    assert tw["drive_net_in"] == pytest.approx(50.0)


def test_agreement_absent_without_trackwheel_lines(tmp_path):
    path = tmp_path / "old.txt"
    path.write_text("[00:00:00.000] [INFO] Robot initialized\n"
                    "[00:00:00.010] [INFO] Initial pose: 0 0 0\n"
                    + "".join(f"[00:00:00.{20 + i:03d}] [INFO] Pose: 0 0 0\n" for i in range(30)))
    run = trajectory_runs(parse_log(path))[0]
    assert metrics.tracking_wheel_agreement(run) == {"ticks": 0}


def test_pose_forward_matches_wheel_when_every_delta_is_used(tmp_path):
    tw = metrics.tracking_wheel_agreement(_log(tmp_path, [(0.5, 0.0, 0.5, 1, 1)] * 200))
    assert tw["pose_forward_in"] == pytest.approx(tw["wheel_abs_net_in"])


def test_pose_forward_shows_rejected_wheel_travel(tmp_path):
    # Boot 989's pattern: a stale 0 then a rejected 1.0 in jump (pose takes ddrive 0.5)
    rows = [(0.0, 0.0, 0.5, 1, 1), (math.nan, 0.0, 0.5, 0, 1, 1.0)] * 100
    tw = metrics.tracking_wheel_agreement(_log(tmp_path, rows))
    assert tw["wheel_abs_net_in"] == pytest.approx(100.0)
    assert tw["pose_forward_in"] == pytest.approx(50.0)
