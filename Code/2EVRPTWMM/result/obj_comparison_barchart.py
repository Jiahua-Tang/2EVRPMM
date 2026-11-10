"""Objective value of F2 (mobile microhubs) vs F3 (fixed microhubs), 2x2 bar chart.

Values are the averages reported in Table result_summary_3 of the paper
(sections/C6_ComputationalResult/managerial_insight.tex).

Run from this folder:  python3 obj_comparison_barchart.py
Saves obj_comparison_2x2.png and obj_comparison_2x2.pdf.
"""
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

mpl.rcParams.update({
    "font.family":      "serif",
    "font.serif":       ["CMU Serif", "Computer Modern Roman", "DejaVu Serif"],
    "mathtext.fontset": "cm",          # LaTeX-like maths, matching the paper
    "font.size":        9,
    "axes.titlesize":   9,
    "axes.labelsize":   9,
    "xtick.labelsize":  9,
    "ytick.labelsize":  9,
    "legend.fontsize":  9,
    "axes.linewidth":   0.8,
    "pdf.fonttype":     42,
    "ps.fonttype":      42,
})

F2_COLOR = "#0072B2"
F3_COLOR = "#CC79A7"   # F1 is #E69F00 in the execution-time figure

CUSTOMERS = [15, 20, 30, 50]
# (title, F2 objective per size, F3 objective per size)
PANELS = [
    (r"$|P| = 4$, $|K_1| = 2$", [402.28, 463.86, 592.32, 880.41], [467.60, 511.54, 652.47, 1035.49]),
    (r"$|P| = 7$, $|K_1| = 4$", [341.49, 423.26, 552.80, 733.21], [355.03, 443.23, 600.73, 828.50]),
    (r"$|P| = 9$, $|K_1| = 5$", [395.92, 464.45, 584.63, 804.20], [481.18, 578.09, 759.27, 1105.98]),
    ("Average", [379.89, 450.52, 576.58, 804.68], [434.60, 510.96, 670.82, 989.22]),
]

TEXT_WIDTH_CM = 16.0
fig, axes = plt.subplots(2, 2, figsize=(TEXT_WIDTH_CM / 2.54, 11.5 / 2.54))
x = np.arange(len(CUSTOMERS))
w = 0.38

for k, (ax, (title, f2, f3)) in enumerate(zip(axes.flat, PANELS)):
    b2 = ax.bar(x - w / 2, f2, w, color=F2_COLOR, alpha=0.80, edgecolor="black", linewidth=0.6, label=r"$\mathcal{F}_2$")
    b3 = ax.bar(x + w / 2, f3, w, color=F3_COLOR, alpha=0.80, edgecolor="black", linewidth=0.6, label=r"$\mathcal{F}_3$")
    for bars in (b2, b3):
        ax.bar_label(bars, fmt="%.0f", padding=1.5, fontsize=7)
    ax.set_xticks(x)
    ax.set_xticklabels(CUSTOMERS)
    ax.set_title(f"({chr(ord('a') + k)}) {title}", y=-0.26)
    ax.set_ylim(0, max(f3) * 1.15)
    ax.yaxis.grid(True, linestyle=":", linewidth=0.5, alpha=0.7)
    ax.set_axisbelow(True)
    ax.tick_params(axis="x", length=0)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    if k % 2 == 0:
        ax.set_ylabel("Objective value")

handles, labels = axes[0][0].get_legend_handles_labels()
fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False,
           bbox_to_anchor=(0.5, 1.02))
fig.tight_layout(pad=0.4, h_pad=2.6, w_pad=1.5, rect=(0, 0, 1, 0.95))
for ext in ("png", "pdf"):
    out = f"obj_comparison_2x2.{ext}"
    fig.savefig(out, bbox_inches="tight", dpi=300)
    print(f"Saved: {out}")
plt.close(fig)
