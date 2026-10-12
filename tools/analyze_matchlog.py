#!/usr/bin/env python3
"""Summarise a match log written by MatchLogger (TeleOp or auto).

Usage:
    adb pull /sdcard/FIRST/data/matchlogs
    python3 tools/analyze_matchlog.py matchlogs/teleop_RED_20261012_143000.csv
    python3 tools/analyze_matchlog.py matchlogs/*.csv          # several at once
    python3 tools/analyze_matchlog.py --plot matchlogs/x.csv   # also save graphs (needs matplotlib)

Only the Python standard library is needed for the text report.
"""
import argparse
import csv
import math
import statistics
import sys
from pathlib import Path


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    out = []
    for r in rows:
        try:
            out.append({k: float(v) for k, v in r.items() if v not in ("", None)})
        except ValueError:
            continue  # partial last line if the robot lost power mid-write
    return out


def col(rows, name):
    return [r[name] for r in rows if name in r]


def fmt_s(x):
    return "-" if x is None else f"{x:.2f} s"


def shot_indices(rows):
    """Rows where a shot happened. Logs store a running total (shotsTotal) so a shot between
    20 Hz rows is never lost; older logs have a per-row 'shot' flag."""
    if "shotsTotal" in rows[0]:
        idx = []
        for i in range(1, len(rows)):
            for _ in range(int(rows[i]["shotsTotal"] - rows[i - 1]["shotsTotal"])):
                idx.append(i)
        return idx
    return [i for i, r in enumerate(rows) if r.get("shot", 0) >= 0.5]


def shot_times(rows):
    return [rows[i]["t"] for i in shot_indices(rows)]


def recovery_times(rows):
    """Time from each shot until the flywheel reports ready again."""
    out = []
    for i in sorted(set(shot_indices(rows))):
        r = rows[i]
        for later in rows[i + 1:]:
            if later.get("flyReady", 0) >= 0.5:
                out.append(later["t"] - r["t"])
                break
    return out


def cycle_times(rows):
    """Time between storage becoming empty - one full collect-and-score cycle."""
    if "storage" not in rows[0]:
        return []
    empties = []
    was_empty = rows[0]["storage"] == 0
    for r in rows[1:]:
        empty = r["storage"] == 0
        if empty and not was_empty:
            empties.append(r["t"])
        was_empty = empty
    return [b - a for a, b in zip(empties, empties[1:])]


def distance_driven(rows):
    xs, ys = col(rows, "x"), col(rows, "y")
    return sum(math.hypot(x2 - x1, y2 - y1) for x1, y1, x2, y2 in zip(xs, ys, xs[1:], ys[1:]))


def fraction(rows, name, when=None):
    sel = [r for r in rows if when is None or when(r)]
    if not sel or name not in sel[0]:
        return None
    return sum(1 for r in sel if r[name] >= 0.5) / len(sel)


def report(path):
    rows = load(path)
    if len(rows) < 2:
        print(f"{path}: no data")
        return
    duration = rows[-1]["t"] - rows[0]["t"]
    shots = shot_times(rows)
    rec = recovery_times(rows)
    cycles = cycle_times(rows)
    firing_key = "firing" if "firing" in rows[0] else "shooting"

    print(f"\n=== {Path(path).name}")
    print(f"duration            {duration:.1f} s   ({len(rows)} rows)")
    print(f"distance driven     {distance_driven(rows):.0f} in")
    print(f"shots detected      {len(shots)}")
    if len(shots) >= 2:
        gaps = [b - a for a, b in zip(shots, shots[1:]) if b - a < 2.0]
        if gaps:
            print(f"time between shots  median {statistics.median(gaps):.2f} s (within bursts)")
    if rec:
        print(f"flywheel recovery   median {statistics.median(rec):.2f} s, worst {max(rec):.2f} s")
    if cycles:
        print(f"cycle time          median {statistics.median(cycles):.1f} s, best {min(cycles):.1f} s, "
              f"count {len(cycles)}")

    firing = lambda r: r.get(firing_key, 0) >= 0.5
    ready = fraction(rows, "flyReady", firing)
    aimed = fraction(rows, "turretOnTarget", firing)
    if ready is not None:
        print(f"while firing        flywheel ready {ready:.0%} of the time"
              + (f", turret on target {aimed:.0%}" if aimed is not None else ""))
        if ready is not None and ready < 0.6:
            print("  -> flywheel is the bottleneck: tune kF/kP or raise velocity tolerance")
        if aimed is not None and aimed < 0.6:
            print("  -> turret is the bottleneck: tune TURRET_KP/KD or check the hive target")

    if "voltage" in rows[0]:
        v = col(rows, "voltage")
        print(f"battery             start {v[0]:.2f} V, min {min(v):.2f} V")
        if min(v) < 11.5:
            print("  -> battery sagged below 11.5 V: charge/replace before the next match")
    if "loopMs" in rows[0]:
        loops = [x for x in col(rows, "loopMs") if x > 0]
        if loops:
            loops.sort()
            p95 = loops[int(0.95 * (len(loops) - 1))]
            print(f"loop time           median {statistics.median(loops):.0f} ms, 95th pct {p95:.0f} ms")
            if p95 > 40:
                print("  -> slow loops: look for I2C reads or telemetry spam")
    if "step" in rows[0]:
        steps = col(rows, "step")
        print(f"auto steps reached  {int(max(steps)) + 1}")


def plot(path):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed - skipping plots (pip install matplotlib)")
        return
    rows = load(path)
    t = col(rows, "t")
    fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    ax[0].plot(t, col(rows, "flyTarget"), label="target")
    ax[0].plot(t, col(rows, "flyVel"), label="velocity")
    for s in shot_times(rows):
        ax[0].axvline(s, color="r", alpha=0.3)
    ax[0].set_ylabel("flywheel (ticks/s)")
    ax[0].legend()
    ax[1].plot(t, col(rows, "storage"))
    ax[1].set_ylabel("elements held")
    if "turretAngle" in rows[0]:
        ax[2].plot(t, [math.degrees(a) for a in col(rows, "turretAngle")], label="angle")
        ax[2].plot(t, [math.degrees(a) for a in col(rows, "turretTarget")], label="target")
        ax[2].set_ylabel("turret (deg)")
        ax[2].legend()
    ax[-1].set_xlabel("time (s)")
    out = Path(path).with_suffix(".png")
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)

    fig2, ax2 = plt.subplots(figsize=(6, 6))
    ax2.plot(col(rows, "x"), col(rows, "y"))
    ax2.set_xlim(0, 144)
    ax2.set_ylim(0, 144)
    ax2.set_aspect("equal")
    ax2.set_title("path (red wall at x=0, audience at y=0)")
    out2 = Path(path).with_name(Path(path).stem + "_path.png")
    fig2.savefig(out2)
    plt.close(fig2)
    print(f"saved {out} and {out2}")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("logs", nargs="+")
    p.add_argument("--plot", action="store_true", help="save PNG graphs next to each log")
    args = p.parse_args(argv)
    for log in args.logs:
        report(log)
        if args.plot:
            plot(log)


if __name__ == "__main__":
    sys.exit(main())
