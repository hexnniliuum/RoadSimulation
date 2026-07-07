"""Analyze vehicle_positions.csv for braking anomalies.

Computes per-vehicle speed/acceleration from consecutive position samples and
reports events that exceed the physical braking capability of the cars
(MaxBraking = 10 units/s^2), which indicate scripted instant stops or teleports
rather than natural braking.

Usage: python brake_analysis.py [csv_path] [--label NAME]
"""
import csv
import math
import sys
from collections import defaultdict

MAX_BRAKING = 10.0          # CarController.MaxBraking (units/s^2)
HARD_BRAKE_THRESHOLD = 25.0  # decel beyond any legitimate braking
EXTREME_THRESHOLD = 60.0     # near-instant stop (e.g. 17 units/s -> 0 in one frame)
TELEPORT_SPEED = 40.0        # implied speed above 2x TargetSpeed = position jump
MIN_SPEED_FOR_EVENT = 3.0    # ignore jitter around standstill
MAX_INTERVAL_DT = 0.25       # skip intervals spanning editor frame hitches


def load(path):
    tracks = defaultdict(list)
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            if len(row) < 5:
                continue
            vid = int(row[0])
            x, y, z, t = (float(v) for v in row[1:5])
            tracks[vid].append((t, x, y, z))
    for vid in tracks:
        tracks[vid].sort(key=lambda s: s[0])
    return tracks


def analyze(tracks):
    events = []           # (severity, vid, time, decel, v_before, v_after)
    teleports = []        # (vid, time, implied_speed)
    total_samples = 0
    duration = 0.0
    speeds = []

    for vid, samples in tracks.items():
        if len(samples) < 3:
            continue
        duration = max(duration, samples[-1][0] - samples[0][0])
        # speed per interval
        vs = []
        for (t0, x0, y0, z0), (t1, x1, y1, z1) in zip(samples, samples[1:]):
            dt = t1 - t0
            if dt <= 1e-6:
                continue
            d = math.dist((x0, y0, z0), (x1, y1, z1))
            v = d / dt
            vs.append((t1, v, dt))
            total_samples += 1
            speeds.append(v)
            if v > TELEPORT_SPEED:
                teleports.append((vid, t1, v))
        # acceleration between consecutive intervals
        for (ta, va, dta), (tb, vb, dtb) in zip(vs, vs[1:]):
            if dta > MAX_INTERVAL_DT or dtb > MAX_INTERVAL_DT:
                continue  # frame hitch, not real vehicle dynamics
            accel = (vb - va) / dtb
            if accel < -HARD_BRAKE_THRESHOLD and va > MIN_SPEED_FOR_EVENT:
                sev = "EXTREME" if accel < -EXTREME_THRESHOLD else "HARD"
                events.append((sev, vid, tb, accel, va, vb))

    # Drop editor frame hitches: if many cars show an event in the same
    # instant, it is a global time-step artifact, not vehicle behavior.
    by_bucket = defaultdict(list)
    for e in events:
        by_bucket[round(e[2], 1)].append(e)
    events = [e for bucket in by_bucket.values() if len(bucket) < 5 for e in bucket]

    return events, teleports, total_samples, duration, speeds


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "vehicle_positions.csv"
    label = ""
    if "--label" in sys.argv:
        label = sys.argv[sys.argv.index("--label") + 1]

    tracks = load(path)
    events, teleports, n, duration, speeds = analyze(tracks)

    extreme = [e for e in events if e[0] == "EXTREME"]
    hard = [e for e in events if e[0] == "HARD"]
    per_min = 60.0 / duration if duration > 0 else 0

    print(f"=== Braking analysis {label} ({path}) ===")
    print(f"vehicles: {len(tracks)}, duration: {duration:.1f}s, samples: {n}")
    if speeds:
        moving = [v for v in speeds if v > 0.5]
        print(f"mean moving speed: {sum(moving)/max(1,len(moving)):.2f} units/s "
              f"(moving fraction {len(moving)/len(speeds)*100:.0f}%)")
    print(f"HARD brake events   (decel > {HARD_BRAKE_THRESHOLD:g}): "
          f"{len(hard):4d}  ({len(hard)*per_min:.1f}/min)")
    print(f"EXTREME brake events (decel > {EXTREME_THRESHOLD:g}): "
          f"{len(extreme):4d}  ({len(extreme)*per_min:.1f}/min)")
    print(f"teleport samples (implied speed > {TELEPORT_SPEED:g}): {len(teleports)}")

    worst = sorted(events, key=lambda e: e[3])[:10]
    if worst:
        print("\nworst events (car, t, decel, v_before -> v_after):")
        for sev, vid, t, a, va, vb in worst:
            print(f"  [{sev:7s}] car {vid:3d} t={t:7.2f}s decel={a:8.1f} "
                  f"v: {va:5.1f} -> {vb:5.1f}")


if __name__ == "__main__":
    main()
