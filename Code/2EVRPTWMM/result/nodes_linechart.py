"""Average number of search-tree nodes vs. number of customers (line chart).

Data: Book1.xlsx, sheet FULL (same source as Table result_summary_n_nodes)
  column C   number of customers
  column M   F1  - CPLEX branch-and-bound nodes
  column BD  F2  - proposed branch-and-price nodes
  column AB  F2* - branch-and-price with random-pair branching + DFS
Only instance rows are used (the per-group average rows are skipped).

Run from this folder:  python3 nodes_linechart.py
Saves nodes_linechart.pdf / .png.
"""
import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd

DATA_FILE = "Book1.xlsx"
SIZES = [15, 20, 30]
WIDTH_CM, HEIGHT_CM = 12.0, 7.0

mpl.rcParams.update({
    "font.family":      "serif",
    "font.serif":       ["CMU Serif", "Computer Modern Roman", "DejaVu Serif"],
    "mathtext.fontset": "cm",
    "font.size":        9,
    "axes.labelsize":   9,
    "xtick.labelsize":  9,
    "ytick.labelsize":  9,
    "legend.fontsize":  9,
    "axes.linewidth":   0.8,
    "pdf.fonttype":     42,
    "ps.fonttype":      42,
})

# Same colors as the execution-time figure (F1 orange, F2 blue) + Okabe-Ito
# green for F2*; checked for colour-blind separation. Each series also has its
# own marker and a direct label, so it reads in greyscale print too.
SERIES = [  # (column, label, color, marker)
    ("M",  r"$\mathcal{F}_1$",   "#E69F00", "s"),
    ("AB", r"$\mathcal{F}_2^*$", "#009E73", "^"),
    ("BD", r"$\mathcal{F}_2$",   "#0072B2", "o"),
]

df = pd.read_excel(DATA_FILE, sheet_name="FULL", header=None, skiprows=1,
                   usecols="A,C,M,AB,BD")
df.columns = ["instance", "customers", "M", "AB", "BD"]
df = df[df["instance"].astype(str).str.match(r"^c[a-z]\d+-\d+,\d+,\d+$")]
for col in ["customers", "M", "AB", "BD"]:
    df[col] = pd.to_numeric(df[col], errors="coerce")
avg = df[df["customers"].isin(SIZES)].groupby("customers")[["M", "AB", "BD"]].mean()

fig, ax = plt.subplots(figsize=(WIDTH_CM / 2.54, HEIGHT_CM / 2.54))
for col, label, color, marker in SERIES:
    y = avg.loc[SIZES, col]
    ax.plot(SIZES, y, color=color, linewidth=1.5, marker=marker, markersize=5,
            markeredgecolor="white", markeredgewidth=0.6, label=label, zorder=3)
    # direct label at the right end of each line
    ax.annotate(label, (SIZES[-1], y.iloc[-1]), xytext=(6, 0),
                textcoords="offset points", va="center", fontsize=9)
    print(f"{col}: " + ", ".join(f"{n}: {v:.2f}" for n, v in zip(SIZES, y)))

ax.set_yscale("log")
ax.set_xticks(SIZES)
ax.set_xlim(SIZES[0] - 1.5, SIZES[-1] + 3)
ax.set_xlabel(r"Number of customers $|N_c|$")
ax.set_ylabel("Average number of nodes (log scale)")
ax.yaxis.grid(True, which="major", linestyle=":", linewidth=0.5, alpha=0.7)
ax.set_axisbelow(True)
for side in ("top", "right"):
    ax.spines[side].set_visible(False)
ax.legend(frameon=False, loc="upper left")

fig.tight_layout(pad=0.4)
for ext in ("pdf", "png"):
    fig.savefig(f"nodes_linechart.{ext}", bbox_inches="tight", dpi=300)
    print(f"Saved: nodes_linechart.{ext}")
