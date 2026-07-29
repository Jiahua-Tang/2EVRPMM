import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl

mpl.rcParams.update({
    "font.family":      "serif",
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
SHEET = "Sheet4"

CPLEX_COLOR = "#E69F00"
BP_COLOR    = "#0072B2"

df = pd.read_excel(DATA_FILE, sheet_name=SHEET, header=0)

for n in [15, 20, 30, 50]:
    mask = (df["#customer"] == n) & df["Instance"].notna()
    sub = df[mask].copy()

    cplex_time = sub["CPLEX_1"].dropna().tolist()
    bp_time    = sub["BAP_1"].dropna().tolist()

    if not cplex_time or not bp_time:
        print(f"No data for {n} customers — skipping.")
        continue

    fig, ax = plt.subplots(figsize=(8.5 / 2.54, 7 / 2.54))

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

    ax.set_xticklabels(["CPLEX", "B&P"])
    ax.set_ylabel("Execution time (s)")
    ax.yaxis.grid(True, linestyle=":", linewidth=0.5, alpha=0.7)
    ax.set_axisbelow(True)
    ax.tick_params(axis="x", length=0)

    plt.tight_layout(pad=0.4)

    out = f"result_exetime_{n}.png"
    plt.savefig(out, bbox_inches="tight", dpi=300)
    print(f"Saved: {out}  (CPLEX n={len(cplex_time)}, B&P n={len(bp_time)})")
    plt.close()
