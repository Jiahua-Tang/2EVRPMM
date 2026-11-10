"""Per-instance results table (LaTeX longtable) for the paper's appendix.

Data: Book1.xlsx, sheet FULL. Each customer size n has three blocks of 20
instance rows (|P|,|M| = 4,2 / 7,4 / 9,5), each followed by an average row;
the size ends with an extra overall-average row:
  column G  F1 execution time (s)      column AU F2 execution time (s)
  column K  F1 objective               column BB F2 objective
  column L  F1 gap ("/" when optimal)  column BC F2 status ("/" optimal,
  column M  F1 status ("O", "F", "No";            "incumbent" feasible)
            node count for 15-30 customers, where every run is optimal)
Instance names are rebuilt from the row position (ce1..ci5 within each block)
because column A is mis-pasted in the 75-customer blocks.

Run from this folder:  python3 appendix_table.py [output.tex]
"""
import sys

import openpyxl

DATA_FILE = "Book1.xlsx"
SHEET = "FULL"
OUT = sys.argv[1] if len(sys.argv) > 1 else "detailed_results.tex"

# First instance row of each customer size.
SIZES = [(15, 2), (20, 66), (30, 130), (50, 194), (75, 258), (100, 322)]
BLOCKS = [("2,3", 4, 2), ("3,5", 7, 4), ("6,4", 9, 5)]
NAMES = [f"{s}{k}" for s in ("ce", "cf", "cg", "ci") for k in range(1, 6)]

ws = openpyxl.load_workbook(DATA_FILE, data_only=True)[SHEET]


def num(v):
    return v if isinstance(v, (int, float)) and not isinstance(v, bool) else None


def fmt(v):
    return "--" if v is None else f"{v:.2f}"


def f1_cells(r):
    time, obj, gap, status = (ws.cell(r, c).value for c in (7, 11, 12, 13))
    if status == "No":
        return [fmt(num(time)), "--", "--", "NoSol"]
    if status == "F":
        return [fmt(num(time)), fmt(num(obj)), fmt(100 * num(gap)), "Fsb"]
    return [fmt(num(time)), fmt(num(obj)), "0.00", "Opt"]


def f2_cells(r):
    time, obj, status = (ws.cell(r, c).value for c in (47, 54, 55))
    if num(time) is None and num(obj) is None:
        return ["--", "--", "--"]
    label = {"/": "Opt", "incumbent": "Fsb"}.get(status, "--")
    # Without an optimality proof, the run stops at the time limit; column AU then
    # records when the last integer solution was found, not the run time.
    shown = "TL" if status == "incumbent" else fmt(num(time))
    return [shown, fmt(num(obj)), label]


lines = [
    r"\begin{small}",
    r"\setlength{\tabcolsep}{4pt}",
    r"\begin{longtable}{l rrrc rrc}",
    r"\caption{Detailed results per instance. Gap: relative gap (\%) reported by CPLEX for $\mathcal{F}_1$. "
    r"Status: Opt = solved to optimality, Fsb = feasible solution without optimality proof, "
    r"NoSol = no feasible solution within the time limit; TL = time limit reached; ``--'': not available.}"
    r"\label{tab:detailed_results}\\",
    r"\toprule",
    r"& \multicolumn{4}{c}{$\mathcal{F}_1$} & \multicolumn{3}{c}{$\mathcal{F}_2$} \\",
    r"\cmidrule(lr){2-5} \cmidrule(lr){6-8}",
    r"Instance & T(s) & Obj. & Gap(\%) & Status & T(s) & Obj. & Status \\",
    r"\midrule",
    r"\endfirsthead",
    r"\multicolumn{8}{l}{\small\itshape Table~\thetable{} (continued)} \\",
    r"\toprule",
    r"& \multicolumn{4}{c}{$\mathcal{F}_1$} & \multicolumn{3}{c}{$\mathcal{F}_2$} \\",
    r"\cmidrule(lr){2-5} \cmidrule(lr){6-8}",
    r"Instance & T(s) & Obj. & Gap(\%) & Status & T(s) & Obj. & Status \\",
    r"\midrule",
    r"\endhead",
    r"\bottomrule",
    r"\endfoot",
]

for k_size, (n, start) in enumerate(SIZES):
    for b, (code, p, m) in enumerate(BLOCKS):
        if k_size or b:
            lines.append(r"\addlinespace")
        lines.append(rf"\multicolumn{{8}}{{l}}{{$|C| = {n}$, $|P| = {p}$, $|M| = {m}$}} \\*")
        first = start + 21 * b
        for i, name in enumerate(NAMES):
            r = first + i
            cells = [rf"\quad {name}-{code},{n}"] + f1_cells(r) + f2_cells(r)
            lines.append(" & ".join(cells) + r" \\")

lines += [r"\end{longtable}", r"\end{small}", ""]

with open(OUT, "w") as fh:
    fh.write("\n".join(lines))
n_rows = sum(1 for l in lines if l.startswith(r"\quad"))
print(f"Saved: {OUT}  ({n_rows} instance rows)")
