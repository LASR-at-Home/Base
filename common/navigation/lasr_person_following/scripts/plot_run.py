#!/usr/bin/env python3
"""
Plot a follow_evaluator run: robot/host paths in the world frame and distance over time.

    plot_run.py <run_dir> [<run_dir> ...]   ->  <run_dir>/plot.png for each
"""
import csv
import json
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def plot(run_dir):
    with open(os.path.join(run_dir, "timeseries.csv")) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return
    num = lambda k: [float(r[k]) if r[k] not in ("", "nan") else float("nan") for r in rows]  # noqa: E731
    t, rx, ry, hx, hy, dist = num("t"), num("rx"), num("ry"), num("hx"), num("hy"), num("dist")
    err = num("est_err")
    summary = {}
    sp = os.path.join(run_dir, "summary.json")
    if os.path.exists(sp):
        summary = json.load(open(sp))

    fig, (a, b) = plt.subplots(1, 2, figsize=(13, 5.5))
    a.plot(hx, hy, "g-", label="host (ground truth)")
    a.plot(rx, ry, "b-", label="robot")
    for i in range(0, len(t), 50):  # pairs every 5 s
        a.plot([rx[i], hx[i]], [ry[i], hy[i]], color="0.7", lw=0.6)
        a.annotate(f"{t[i]:.0f}s", (hx[i], hy[i]), fontsize=7, color="g")
    a.plot(rx[0], ry[0], "bo")
    a.plot(hx[0], hy[0], "go")
    a.set_aspect("equal")
    a.grid(alpha=0.3)
    a.legend(loc="best", fontsize=8)
    a.set_title(os.path.basename(run_dir.rstrip("/")))
    a.set_xlabel("x (m, world)")
    a.set_ylabel("y (m, world)")

    b.plot(t, dist, "b-", label="robot-host distance")
    if any(e == e for e in err):
        b.plot(t, err, "r-", lw=0.8, label="tracker error")
    b.axhspan(0.9, 2.0, color="g", alpha=0.08, label="target band")
    walking = [r["status"] == "walking" for r in rows]
    for i in range(1, len(t)):
        if walking[i]:
            b.axvspan(t[i - 1], t[i], color="0.9", lw=0)
    b.set_xlabel("time (s), grey = host walking")
    b.set_ylabel("m")
    b.grid(alpha=0.3)
    b.legend(loc="upper left", fontsize=8)
    txt = "\n".join(f"{k}: {v}" for k, v in summary.items())
    b.text(1.02, 1.0, txt, transform=b.transAxes, fontsize=7, va="top", family="monospace")
    fig.tight_layout()
    fig.savefig(os.path.join(run_dir, "plot.png"), dpi=90)
    plt.close(fig)


if __name__ == "__main__":
    for d in sys.argv[1:]:
        plot(d)
