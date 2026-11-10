"""Solution-status figure (optimal / feasible / no solution), F1 vs F2.

Reads the bold total rows of Table result_summary_2 directly from the paper
(sections/C6_ComputationalResult/solution_quality.tex), so editing the table
and rerunning this script is enough to update the figure.

Run:  python3 solution_comparison.py [path/to/solution_quality.tex]
Saves solution_comparison.pdf / .png next to this script.
"""
import re
import sys
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt

TEX = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
    "/Users/jiahua/Downloads/2eVRPTWMM (1)/sections/C6_ComputationalResult/solution_quality.tex")
SIZES = [50, 75, 100]          # panels, left to right
TEXT_WIDTH_CM = 16.0

mpl.rcParams.update({
    "font.family":      "serif",
    "font.serif":       ["CMU Serif", "Computer Modern Roman", "DejaVu Serif"],
    "mathtext.fontset": "cm",
    "font.size":        9,
    "axes.labelsize":   9,
    "xtick.labelsize":  9,
    "ytick.labelsize":  9,
    "axes.linewidth":   0.8,
    "hatch.linewidth":  0.8,
    "pdf.fonttype":     42,
    "ps.fonttype":      42,
})

# ---- read the bold total rows of Table result_summary_2 --------------------
src = TEX.read_text()
start = src.index(r"\label{result_summary_2}")
table = src[src.rindex(r"\begin{table}", 0, start):start]
counts = {}
for line in table.splitlines():
    cells = re.findall(r"\\textbf\{([^}]*)\}", line)
    if len(cells) == 9 and cells[0].isdigit():
        n = int(cells[0])
        v = [int(c) for c in cells[3:]]
        counts[n] = {"F1": v[0:3], "F2": v[3:6]}   # opt, fsb, nosol
missing = [n for n in SIZES if n not in counts]
if missing:
    sys.exit(f"No bold total row found for {missing} in {TEX}")

# ---- draw ------------------------------------------------------------------
STATUS = [("Optimal",                         "#009E73", ""),
          ("Feasible (not proven optimal)",  "#F0E442", "///"),
          ("No solution found",               "#D55E00", "xx")]
METHODS = [("F2", r"$\mathcal{F}_2$"), ("F1", r"$\mathcal{F}_1$")]   # top, bottom

fig, axes = plt.subplots(1, len(SIZES), figsize=(TEXT_WIDTH_CM / 2.54, 5.2 / 2.54),
                         sharex=True)
for ax, n in zip(axes, SIZES):
    total = sum(counts[n]["F1"])
    for row, (key, _) in enumerate(METHODS):
        left = 0
        for (label, color, hatch), value in zip(STATUS, counts[n][key]):
            if value > 0:
                ax.barh(row, value, left=left, height=0.55, color=color, hatch=hatch,
                        edgecolor="black", linewidth=0.6, label=label)
            left += value
    ax.set_yticks(range(len(METHODS)))
    ax.set_yticklabels([lab for _, lab in METHODS])
    ax.invert_yaxis()                     # F2 on top, as in the paper's tables' order
    ax.set_xlim(0, total)
    ax.set_xticks(range(0, total + 1, 10))
    ax.set_title(rf"$|C| = {n}$", fontsize=9)
    ax.xaxis.grid(True, linewidth=0.5, alpha=0.4)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)

handles = [mpl.patches.Patch(facecolor=c, hatch=h, edgecolor="black", linewidth=0.6, label=l)
           for l, c, h in STATUS]
fig.legend(handles=handles, loc="lower center", ncol=3, frameon=False,
           bbox_to_anchor=(0.5, -0.02), handlelength=2.2, columnspacing=2.0)
fig.tight_layout(pad=0.4, w_pad=1.8, rect=(0, 0.10, 1, 1))

out_dir = Path(__file__).resolve().parent
for ext in ("pdf", "png"):
    out = out_dir / f"solution_comparison.{ext}"
    fig.savefig(out, bbox_inches="tight", dpi=300)
    print("Saved:", out)
for n in SIZES:
    print(f"  |N_c|={n}: F1 opt/fsb/nosol = {counts[n]['F1']}, F2 = {counts[n]['F2']}")
