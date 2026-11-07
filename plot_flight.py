"""Plot a decoded flight CSV (from flight_log.py parse) as time-aligned panels."""

import argparse
import math
from pathlib import Path
import re
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


SURFACE, INK, INK2, MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
GRID, AXIS, BAND = "#e1e0d9", "#c3c2b7", "#f0efec"
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"
PANELS = (
    ("Altitude (ft, filtered)", 2.0),
    ("Vertical velocity (ft/s, filtered)", 1.5),
    ("Vertical acceleration (g, filtered)", 1.3),
    ("Body acceleration (m/s²)", 1.5),
    ("Attitude (deg)", 1.5),
    ("Barometric pressure (kPa)", 1.1),
)
BOOST_WINDOW_S = 5  # the boost peak is searched for before this time
LANDING_HEIGHT_FT = 50  # the landing spike is searched for this close to the ground


def break_at_wraps(values):
    """Break the line at +/-180 degree wraps so it doesn't draw vertical streaks."""
    v = values.to_numpy(dtype=float).copy()
    v[np.r_[False, np.abs(np.diff(v)) > 180]] = np.nan
    return v


def end_labels(ax, items, min_sep=0.11):
    """Label series at the right edge, nudged apart so they never collide."""
    lo, hi = ax.get_ylim()
    rows = sorted(((y - lo) / (hi - lo), name, c) for name, y, c in items)
    pos = [r[0] for r in rows]
    for i in range(1, len(pos)):
        pos[i] = max(pos[i], pos[i - 1] + min_sep)
    overflow = pos[-1] - 0.97
    if overflow > 0:
        pos = [p - overflow for p in pos]
    for p, (_, name, c) in zip(pos, rows):
        ax.annotate("", xy=(1.022, p), xytext=(1.006, p), xycoords="axes fraction",
                    arrowprops=dict(arrowstyle="-", color=c, lw=2.5))
        ax.text(1.028, p, name, transform=ax.transAxes, va="center", color=INK2, fontsize=9.5)


def callout(ax, x, y, text, dx, dy, ha="left"):
    ax.annotate(text, xy=(x, y), xytext=(dx, dy), textcoords="offset points", ha=ha,
                va="center", fontsize=9.5, color=INK,
                arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8, shrinkA=2, shrinkB=2))
    ax.plot([x], [y], "o", ms=6, color=S1, mec=SURFACE, mew=1.5, zorder=5)


def plot_flight(d, title, output):
    launch = d.loc[d.flight_state == "LAUNCH", "timestamp_ms"]
    if launch.empty:
        raise ValueError("log never reaches LAUNCH, so it has no T=0 to plot against")
    d = d.assign(t=(d.timestamp_ms - launch.iloc[0]) / 1000.0)

    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Inter", "Noto Sans", "Liberation Sans", "DejaVu Sans"],
        "font.size": 10,
        "axes.edgecolor": AXIS,
        "axes.labelcolor": INK2,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "axes.facecolor": SURFACE,
        "figure.facecolor": SURFACE,
    })
    fig, axes = plt.subplots(len(PANELS), 1, sharex=True, figsize=(12, 15),
                             gridspec_kw={"height_ratios": [p[1] for p in PANELS], "hspace": 0.26})

    # Flight-state spans, shaded alternately on every panel.
    spans = d.groupby((d.flight_state != d.flight_state.shift()).cumsum()).agg(
        state=("flight_state", "first"), t0=("t", "first"), t1=("t", "last"))
    xmin, xmax = d.t.min(), d.t.max()
    for ax, (panel_title, _) in zip(axes, PANELS):
        for i, s in enumerate(spans.itertuples()):
            if i % 2:
                ax.axvspan(s.t0, s.t1, color=BAND, lw=0, zorder=0)
            if i:
                ax.axvline(s.t0, color=AXIS, lw=0.8, zorder=1)
        ax.set_title(panel_title, loc="left", fontsize=11, color=INK, fontweight="bold", pad=6)
        ax.grid(axis="y", color=GRID, lw=0.7, zorder=1)
        ax.set_axisbelow(True)
        ax.tick_params(length=0)
        for side in ("top", "right", "left"):
            ax.spines[side].set_visible(False)
        ax.set_xlim(xmin, xmax)

    ax_alt, ax_vel, ax_acc, ax_body, ax_att, ax_p = axes
    ax_alt.plot(d.t, d.altitude, color=S1, lw=2)
    # Headroom so the apogee callout clears the state names above the panel.
    ax_alt.set_ylim(top=d.altitude.max() + 0.128 * np.ptp(d.altitude))
    ax_vel.plot(d.t, d.velocity, color=S1, lw=1.6)
    ax_acc.plot(d.t, d.acceleration, color=S1, lw=1.1)
    ax_p.plot(d.t, d.pressure, color=S1, lw=1.6)
    for c, col in (("ax", S1), ("ay", S2), ("az", S3)):
        ax_body.plot(d.t, d[c], color=col, lw=0.9, alpha=0.95)
    for c, col in (("yaw", S1), ("pitch", S2), ("roll", S3)):
        ax_att.plot(d.t, break_at_wraps(d[c]), color=col, lw=1.1)
    ax_att.set_yticks([-180, -90, 0, 90, 180])

    # Selective direct labels: the handful of values people read this chart for.
    i = d.altitude.idxmax()
    callout(ax_alt, d.t[i], d.altitude[i], f"Apogee {d.altitude[i]:,.0f} ft at T+{d.t[i]:.1f} s", 16, 2)
    i = d.velocity.idxmax()
    callout(ax_vel, d.t[i], d.velocity[i], f"Peak {d.velocity[i]:.0f} ft/s at T+{d.t[i]:.1f} s", 16, -2)
    i = d.velocity.idxmin()
    callout(ax_vel, d.t[i], d.velocity[i], f"Descent peak −{abs(d.velocity[i]):.0f} ft/s", 0, 26, ha="center")
    i = d[d.t < BOOST_WINDOW_S].acceleration.idxmax()
    callout(ax_acc, d.t[i], d.acceleration[i], f"Boost {d.acceleration[i]:.1f} g", -14, 0, ha="right")
    if (d.flight_state == "LANDED").any():  # otherwise the log has no touchdown to label
        landing = d[d.flight_state.isin(("DEPLOYMENT", "LANDED"))
                    & (d.altitude < d.altitude.iloc[-1] + LANDING_HEIGHT_FT)]
        i = landing.acceleration.idxmax()
        callout(ax_acc, d.t[i], d.acceleration[i], f"Landing spike {d.acceleration[i]:.1f} g", -14, 0, ha="right")

    for ax, cols in ((ax_body, (("ax", S1), ("ay", S2), ("az", S3))),
                     (ax_att, (("yaw", S1), ("pitch", S2), ("roll", S3)))):
        end_labels(ax, [(c, d[c].iloc[-1], col) for c, col in cols])

    # State names in a strip above the first panel, with launch as T=0.
    for s in spans.itertuples():
        if s.t1 - s.t0 < 1:  # too short to centre a name in, e.g. the final LANDED sample
            x, ha = s.t0 + 0.4, "left"
        else:
            x, ha = (s.t0 + s.t1) / 2, "center"
        ax_alt.text(x, 1.17, s.state, transform=ax_alt.get_xaxis_transform(), ha=ha,
                    va="bottom", fontsize=9, color=INK2, fontweight="bold")

    ax_p.set_xlabel("Time since launch detection (s)", color=INK2)
    ax_p.set_xticks(np.arange(math.ceil(xmin / 10) * 10, xmax, 10))
    fig.text(0.07, 0.968, title, ha="left", va="center", fontsize=16, fontweight="bold", color=INK)
    fig.subplots_adjust(left=0.07, right=0.93, top=0.91, bottom=0.05)
    fig.savefig(output, dpi=170)
    plt.close(fig)


def default_title(path):
    match = re.fullmatch(r"flight[ _-]?(\d+)", path.stem, re.IGNORECASE)
    return f"Flight {match[1]} telemetry" if match else f"{path.stem} telemetry"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="CSV written by flight_log.py parse")
    parser.add_argument("output", type=Path, nargs="?",
                        help="image to write, replacing any existing file (default: input name with .png)")
    parser.add_argument("--title", help="chart title (default: based on the input name)")
    args = parser.parse_args()

    output = args.output or args.input.with_suffix(".png")
    try:
        plot_flight(pd.read_csv(args.input), args.title or default_title(args.input), output)
    except (OSError, ValueError) as error:
        print(f"{error}.", file=sys.stderr)
        return 1
    print(f"Wrote {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
