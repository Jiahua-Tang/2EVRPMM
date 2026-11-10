"""Execution-time boxplots over optimally solved instances only (T2), F1 vs F2.

Data: Book1.xlsx, sheet FULL
  column A  instance name (e.g. ce1-2,3,15)
  column C  number of customers
  column G  F1 (CPLEX) total execution time (s)
  column M  F1 status: "O" optimal, "F" feasible, "No" no solution
            (for 15-30 customers it holds node counts; all those runs are optimal)
  column AU F2 (B&P) total execution time (s)
  column BC F2 status: "/" optimal, "incumbent" feasible
Same style as boxplot_exetime_full.py.

Run from this folder:  python3 boxplot_exetime_T2.py
Saves exetime_T2_<n>.png for each size in SIZES.
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
SIZES = [15, 20, 30, 50, 75]

CPLEX_COLOR = "#E69F00"
BP_COLOR    = "#0072B2"

INSTANCE = re.compile(r"^c[a-z]\d+-\d+,\d+,\d+$")

# Read by column letter: the header row of FULL does not name every column.
df = pd.read_excel(DATA_FILE, sheet_name=SHEET, header=None, skiprows=1,
                   usecols="A,C,G,M,AU,BC")
df.columns = ["instance", "customers", "cplex_time", "cplex_status",
              "bp_time", "bp_status"]
df = df[df["instance"].astype(str).str.match(INSTANCE)]
for col in ["customers", "cplex_time", "bp_time"]:
    df[col] = pd.to_numeric(df[col], errors="coerce")

cplex_opt = ~df["cplex_status"].astype(str).isin(["F", "No"])
bp_opt = df["bp_status"].astype(str) == "/"


def draw_boxplot(ax, cplex_time, bp_time):
    bp = ax.boxplot(
        [cplex_time, bp_time],
        patch_artist=True,
        widths=0.45,
        medianprops=dict(color="white", linewidth=1.5),
        whiskerprops=dict(linewidth=0.8),
        capprops=dict(linewidth=0.8),
        flierprops=dict(marker="o", markersize=3, linestyle="none", alpha=0.6),
        showmeans=True,
        meanprops=dict(marker="D", markersize=4, markerfacecolor="white",
                       markeredgecolor="black", markeredgewidth=0.8),
    )

    for patch, color in zip(bp["boxes"], [CPLEX_COLOR, BP_COLOR]):
        patch.set_facecolor(color)
        patch.set_alpha(0.80)
        patch.set_linewidth(0.8)

    for flier, color in zip(bp["fliers"], [CPLEX_COLOR, BP_COLOR]):
        flier.set(markerfacecolor=color, markeredgecolor=color)

    # Show the number of optimal instances only when some were not solved.
    def label(name, times):
        return name if len(times) == 60 else rf"{name} ($n={len(times)}$)"

    ax.set_xticks([1, 2])
    ax.set_xticklabels([label(r"$\mathcal{F}_1$", cplex_time),
                        label(r"$\mathcal{F}_2$", bp_time)])
    ax.set_ylabel("Execution time (s)")
    ax.yaxis.grid(True, linestyle=":", linewidth=0.5, alpha=0.7)
    ax.set_axisbelow(True)
    ax.tick_params(axis="x", length=0)


for n in SIZES:
    sub = df[df["customers"] == n]
    cplex_time = sub.loc[cplex_opt, "cplex_time"].dropna().tolist()
    bp_time = sub.loc[bp_opt, "bp_time"].dropna().tolist()

    fig, ax = plt.subplots(figsize=(8.5 / 2.54, 7 / 2.54))
    draw_boxplot(ax, cplex_time, bp_time)
    plt.tight_layout(pad=0.4)
    out = f"exetime_T2_{n}.png"
    plt.savefig(out, bbox_inches="tight", dpi=300)
    print(f"Saved: {out}  (optimal: F1 {len(cplex_time)}, F2 {len(bp_time)}; "
          f"mean F1 {sum(cplex_time) / len(cplex_time):.2f} s, "
          f"F2 {sum(bp_time) / len(bp_time):.2f} s)")
    plt.close()
