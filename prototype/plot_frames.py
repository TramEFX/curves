"""Plot CSV frames from run_scenarios.py: one figure per scenario, lines
colored from early (dark) to late (bright) so motion direction is visible."""

import csv
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import cm

HERE = os.path.dirname(os.path.abspath(__file__))


def load(name):
    frames = {}
    with open(os.path.join(HERE, name + ".csv")) as fh:
        for row in csv.DictReader(fh):
            t = float(row["time_s"])
            frames.setdefault(t, {})[int(row["node"])] = float(row["value"])
    times = sorted(frames)
    curves = [np.array([frames[t][i] for i in range(len(frames[t]))]) for t in times]
    return np.array(times), curves


def plot(name, title, note=""):
    times, curves = load(name)
    fig, ax = plt.subplots(figsize=(10, 5))
    colors = cm.viridis(np.linspace(0.05, 0.9, len(curves)))
    for j, (t, u, col) in enumerate(zip(times, curves, colors)):
        lbl = f"t={t:.2f}s" if j in (0, len(curves) - 1) else None
        ax.plot(u, color=col, lw=1.2, alpha=0.85, label=lbl)
    ax.set_title(f"Curves prototype — {title}")
    ax.set_xlabel("node (low frequency → high frequency)")
    ax.set_ylabel("displacement")
    if note:
        ax.text(0.01, 0.98, note, transform=ax.transAxes, va="top", fontsize=8, color="#555")
    ax.legend(loc="lower right", fontsize=8)
    ax.axhline(0, color="#999", lw=0.6)
    out = os.path.join(HERE, name + ".png")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"  -> {out}")


if __name__ == "__main__":
    names = sys.argv[1:] or ["pluck", "flagpole", "travel", "recover", "cloth", "cloth_rev"]
    titles = {
        "pluck": "pluck and release (anchor both ends)",
        "flagpole": "flagpole: pinned high end, wind 0–1.5s, release after",
        "travel": "traveling disturbance with reflections",
        "recover": "loose string -> tension restored (no stuck slow modes)",
        "cloth": "cloth wind: two anchors, distributed gust sites",
        "cloth_rev": "cloth wind reversed (low -> high)",
    }
    for n in names:
        plot(n, titles.get(n, n))
