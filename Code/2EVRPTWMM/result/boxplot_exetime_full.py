"""Execution-time boxplots, CPLEX vs B&P, one figure per instance size.

Data: Book1.xlsx, sheet FULL
  column A  instance name (e.g. ce1-2,3,15)
  column C  number of customers
  column G  CPLEX total execution time (s)
  column AU B&P total execution time (s)
Rows whose column A is not an instance name (the per-group average rows,
which show "#DIV/0!" in column A) are skipped.

Run from this folder:  python3 boxplot_exetime_full.py
Saves exetime_FULL_<n>.png for each size that has both times.
"""
import re

import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd

mpl.rcParams.update({
    "font.family":      "serif",
    "font.serif":       ["CMU Serif", "Computer Modern Roman", "DejaVu Serif"],
    "mathtext.fontset": "cm",          # LaTeX-like maths, matching the paper
    "font.size":        9,
    "axes.titlesize":   9,
    "axes.labelsize":   9,
    "xtick.labelsize":  9,
    "ytick.labelsize":  9,
    "axes.linewidth":   0.8,
    "lines.linewidth":  0.8,
    "pdf.fonttype":     42,
    "ps.fonttype":      42,
})

DATA_FILE = "Book1.xlsx"
SHEET = "FULL"

CPLEX_COLOR = "#E69F00"
BP_COLOR    = "#0072B2"

INSTANCE = re.compile(r"^c[a-z]\d+-\d+,\d+,\d+$")

# Read by column letter: the header row of FULL does not name every column.
df = pd.read_excel(DATA_FILE, sheet_name=SHEET, header=None, skiprows=1,
                   usecols="A,C,G,AU")
df.columns = ["instance", "customers", "cplex_time", "bp_time"]
df = df[df["instance"].astype(str).str.match(INSTANCE)]
for col in ["customers", "cplex_time", "bp_time"]:
    df[col] = pd.to_numeric(df[col], errors="coerce")

LABELS = [r"$\mathcal{F}_1$", r"$\mathcal{F}_2$"]


def draw_boxplot(ax, cplex_time, bp_time):
    bp = ax.boxplot(
        [cplex_time, bp_time],
        patch_artist=True,
        widths=0.45,
        medianprops=dict(color="white", linewidth=1.5),
        whiskerprops=dict(linewidth=0.8),
        capprops=dict(linewidth=0.8),
        flierprops=dict(marker="o", markersize=3, linestyle="none", alpha=0.6),
    )

    for patch, color in zip(bp["boxes"], [CPLEX_COLOR, BP_COLOR]):
        patch.set_facecolor(color)
        patch.set_alpha(0.80)
        patch.set_linewidth(0.8)

    for flier, color in zip(bp["fliers"], [CPLEX_COLOR, BP_COLOR]):
        flier.set(markerfacecolor=color, markeredgecolor=color)

    ax.set_xticklabels(LABELS)
    ax.set_ylabel("Execution time (s)")
    ax.yaxis.grid(True, linestyle=":", linewidth=0.5, alpha=0.7)
    ax.set_axisbelow(True)
    ax.tick_params(axis="x", length=0)


# Sizes with both times, in increasing order.
sizes = []
for n in sorted(df["customers"].dropna().unique().astype(int)):
    sub = df[(df["customers"] == n)].dropna(subset=["cplex_time", "bp_time"])
    if sub.empty:
        print(f"No instance with both times for {n} customers - skipping.")
        continue
    sizes.append((n, sub))

# ---- One figure per size -------------------------------------------------
for n, sub in sizes:
    fig, ax = plt.subplots(figsize=(8.5 / 2.54, 7 / 2.54))
    draw_boxplot(ax, sub["cplex_time"].tolist(), sub["bp_time"].tolist())
    plt.tight_layout(pad=0.4)
    out = f"exetime_FULL_{n}.png"
    plt.savefig(out, bbox_inches="tight", dpi=300)
    print(f"Saved: {out}  ({len(sub)} instances; median F1 {sub['cplex_time'].median():.1f} s, "
          f"F2 {sub['bp_time'].median():.1f} s)")
    plt.close()

# ---- All sizes in one 2-column figure, ready for \textwidth ---------------
# Each panel gets its caption underneath: (a) |N_c| = 15, (b) ... as in the
# paper, so the figure can be included with
#   \includegraphics[width=\textwidth]{exetime_FULL_all.pdf}
TEXT_WIDTH_CM = 16.0
ncols = 2
nrows = (len(sizes) + ncols - 1) // ncols
fig, axes = plt.subplots(nrows, ncols,
                         figsize=(TEXT_WIDTH_CM / 2.54, nrows * 6.8 / 2.54),
                         squeeze=False)
for k, (n, sub) in enumerate(sizes):
    ax = axes[k // ncols][k % ncols]
    draw_boxplot(ax, sub["cplex_time"].tolist(), sub["bp_time"].tolist())
    ax.set_title(f"({chr(ord('a') + k)}) $|N_c| = {n}$", y=-0.24, fontsize=9)
for k in range(len(sizes), nrows * ncols):
    axes[k // ncols][k % ncols].axis("off")
fig.tight_layout(pad=0.4, h_pad=2.2, w_pad=2.0)
fig.align_ylabels(axes[:, 0])
fig.align_ylabels(axes[:, 1])
for ext in ("pdf", "png"):
    out = f"exetime_FULL_all.{ext}"
    fig.savefig(out, bbox_inches="tight", dpi=300)
    print(f"Saved: {out}")
plt.close(fig)
