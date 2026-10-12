#!/usr/bin/env python3
"""Curve diagnostics. Optional export verifies exactly which reference was run.

python3 tests/analyze_curve.py tests/logs/09122026-3.txt --reference /mnt/sdcard/routes/turn_test.txt
"""
import argparse
import hashlib
import math
from pathlib import Path

from log_analysis import metrics
from log_analysis.parser import parse_log, trajectory_runs


def analyze(run, reference=None):
    ticks = run.ticks
    print(f'{len(ticks)} control ticks; build={run.build_config}')
    print('Tracking:', metrics.tracking_errors(run))
    print('Wheel/IMU fit (filtered wheel speeds; not a calibration):',
          metrics.effective_track_width(run, nominal=metrics.declared_track_width(run) or metrics.TRACK_WIDTH))
    print('Rotation budget:', metrics.rotation_deficit(run))
    for side in ('left', 'right'):
        saturated = [t for t in ticks if abs(getattr(t, f'voltage_{side}') or 0) >= 126]
        if saturated:
            errors = [abs(getattr(t, f'{side}_vel_setpoint') - getattr(t, f'{side}_vel_actual')) for t in saturated]
            print(f'{side} saturation: {len(saturated)} ticks, '
                  f't={saturated[0].t-ticks[0].t:.3f}..{saturated[-1].t-ticks[0].t:.3f}s, '
                  f'mean wheel error={sum(errors)/len(errors):.3f} in/s')
        # Only the first two entries are real motors on this build; historical
        # firmware read a nonexistent third entry. Compare travel, not offsets.
        positions = [getattr(t, f'{side}_positions') for t in ticks]
        positions = [p for p in positions if p is not None and len(p) >= 2]
        if positions:
            travel = [positions[-1][i] - positions[0][i] for i in range(2)]
            print(f'{side} motor encoder travel (ticks): {travel}; '
                  f'partner difference={abs(travel[0]-travel[1])/max(1, max(map(abs, travel))):.1%}')
    if reference is None:
        return
    data = reference.read_bytes()
    rows = [[float(v) for v in line.split()[1:7]]
            for line in data.decode().splitlines() if line.split() and line.split()[0] == '0']
    if len(rows) != len(ticks):
        raise ValueError('Export point count does not match log; cannot attribute reference')
    error = max(abs(row[i+1]-tick.goal[i]) for row, tick in zip(rows, ticks) for i in range(3))
    if error > 1e-5:
        raise ValueError(f'Export poses do not match log (max error {error})')
    print(f'Matching export SHA256: {hashlib.sha256(data).hexdigest()} (max pose difference {error:.3g})')
    distance = angle = chord = 0.0
    max_interval_error = 0.0
    for a, b in zip(rows, rows[1:]):
        dt = b[0]-a[0]
        ds = math.hypot(b[1]-a[1], b[2]-a[2])
        integral = (a[4]+b[4])*dt/2
        distance += integral
        angle += (a[5]+b[5])*dt/2
        chord += ds
        max_interval_error = max(max_interval_error, abs(ds-abs(integral)))
    print(f'Reference: integrated travel={distance:.6f}in, chord length={chord:.6f}in, '
          f'integrated yaw={math.degrees(angle):.6f}deg; '
          f'max 10ms distance/integrated-speed discrepancy={max_interval_error:.6f}in')
    print(f'Final exported speed={rows[-1][4]:.6f}in/s at t={rows[-1][0]:.3f}s')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--reference', type=Path)
    args = parser.parse_args()
    for run in trajectory_runs(parse_log(args.log)):
        analyze(run, args.reference)
