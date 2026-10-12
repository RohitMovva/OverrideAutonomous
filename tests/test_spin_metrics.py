"""Spin metrics (traction A/B, tracking-wheel offsets) on synthetic spin logs.

The logs are built in the shape the firmware writes them (TRACKWHEEL, Pose, MP Output,
wheel velocities, Voltages per tick) from a robot with known offsets, track width and
turn cost, and the metrics must recover those numbers.
"""
from __future__ import annotations

import math

import pytest

from log_analysis import metrics
from log_analysis.parser import parse_log, trajectory_runs
from log_analysis.robot_config import KS, KV

LAT_OFF, VERT_OFF = 2.5, -0.4      # in: lateral wheel 2.5 in forward, vertical 0.4 in right
TRACK_EFF = 13.0                   # in: what the wheels see per radian, scrub included
KS_TRUE, KV_TRUE = 10.0, 8.0       # turn cost per wheel: motor units, per rad/s


def _plan(rate, accel=7.0, turns=2.0, dt=0.01):
    """Planned yaw rate per tick: trapezoid CCW, hold, the same CW, hold."""
    seq = []
    for sign in (1, -1):
        total, w, ang = turns * 2 * math.pi, 0.0, 0.0
        while ang < total:
            braking = w * w / (2 * accel) >= total - ang
            w = max(w - accel * dt, 0.05) if braking else min(w + accel * dt, rate)
            ang += w * dt
            seq.append(sign * w)
        seq += [0.0] * 100
    return seq


def _log(tmp_path, rate, name="spin", traction=None, creep=0.0):
    lines = ["[00:00:00.000] [INFO] Robot initialized"]
    if traction is not None:
        lines.append(f"[00:00:00.005] [LOG] Build config: wheelBase=11.350 tractionWheels={traction}")
    lines.append("[00:00:00.010] [INFO] Initial pose: 0.000000 0.000000 0.000000")
    theta = vert = lat = 0.0
    for i, w in enumerate(_plan(rate)):
        ms = 20 + 10 * i
        stamp = f"[00:{ms // 60000:02d}:{(ms // 1000) % 60:02d}.{ms % 1000:03d}]"
        d = w * 0.01
        theta += d
        dlat, dvert = LAT_OFF * d, -VERT_OFF * d
        lat += dlat
        vert += dvert
        vl, vr = -w * TRACK_EFF / 2, w * TRACK_EFF / 2
        extra = (KS_TRUE + KV_TRUE * abs(w)) if abs(w) > 1e-9 else 0.0
        volt = [round(math.copysign(1, v) * (KS + KV * abs(v) + extra)) if abs(v) > 1e-9 else 0
                for v in (vl, vr)]
        lines += [
            f"{stamp} [INFO] TRACKWHEEL vert={vert:f} lat={lat:f} dvert={dvert:f} dlat={dlat:f} "
            f"ddrive=0.000000 vok=1 lok=1",
            f"{stamp} [INFO] Pose: 0.000000 0.000000 {math.remainder(theta, 2 * math.pi):f}",
            f"{stamp} [INFO] Ramsete Output: {creep * abs(w):f} {w:f}",
            f"{stamp} [INFO] MP Output: 0.000000 {w:f}",
            f"{stamp} [INFO] Left wheel velocities: {vl:f} {vl:f}",
            f"{stamp} [INFO] Right wheel velocities: {vr:f} {vr:f}",
            f"{stamp} [INFO] Voltages: {volt[0]} {volt[1]}",
        ]
    path = tmp_path / f"{name}_{rate:g}.txt"
    path.write_text("\n".join(lines) + "\n")
    runs = trajectory_runs(parse_log(path))
    assert len(runs) == 1
    return runs[0]


def test_offsets_recovered_from_a_spin(tmp_path):
    off = metrics.tracking_wheel_offsets(_log(tmp_path, 2.0))
    assert off["radians"] == pytest.approx(8 * math.pi, rel=0.05)
    assert off["lateral_offset_in"] == pytest.approx(LAT_OFF, abs=0.01)
    assert off["vertical_offset_in"] == pytest.approx(VERT_OFF, abs=0.01)
    assert off["lateral_ccw_in"] == pytest.approx(off["lateral_cw_in"], abs=0.01)
    assert off["lateral_residual_in_per_rad"] < 0.01


def test_scrub_fit_recovers_turn_cost_over_two_rates(tmp_path):
    runs = [_log(tmp_path, 1.0), _log(tmp_path, 3.0)]
    fit = metrics.fit_spin_scrub(runs)
    # Voltages are rounded to whole motor units, as the firmware logs them
    assert fit["ks"] == pytest.approx(KS_TRUE, abs=0.6)
    assert fit["kv"] == pytest.approx(KV_TRUE, abs=0.4)
    assert fit["ks_left"] == pytest.approx(fit["ks_right"], abs=0.6)


def test_spin_scrub_reports_track_width_rate_and_plan(tmp_path):
    sc = metrics.spin_scrub(_log(tmp_path, 3.0))
    assert sc["effective_track_width_in"] == pytest.approx(TRACK_EFF, rel=0.01)
    assert sc["top_rate"] == pytest.approx(3.0, rel=0.02)
    assert sc["top_accel"] == pytest.approx(7.0, rel=0.15)
    assert sc["delivered_ratio"] == pytest.approx(1.0, abs=0.02)
    assert sc["saturated_fraction"] == 0.0


def test_one_rate_cannot_separate_ks_from_kv(tmp_path):
    assert "ks" not in metrics.fit_spin_scrub([_log(tmp_path, 2.0)])


def test_traction_wheels_from_build_config(tmp_path):
    assert metrics.traction_wheels(_log(tmp_path, 2.0, traction=1)) == 1
    assert metrics.traction_wheels(_log(tmp_path, 2.0, name="old")) is None


def test_spin_creep_measures_forward_command(tmp_path):
    assert metrics.spin_creep(_log(tmp_path, 2.0))["creep_in_per_rad"] == pytest.approx(0.0, abs=1e-6)
    crept = metrics.spin_creep(_log(tmp_path, 2.0, name="crept", creep=0.72))
    assert crept["creep_in_per_rad"] == pytest.approx(0.72, rel=0.03)
