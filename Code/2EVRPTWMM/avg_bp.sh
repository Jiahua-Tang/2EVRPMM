#!/bin/bash
# Average the time columns of a merged B&P result CSV.
# Usage:  ./avg_bp.sh <file.csv>
set -euo pipefail

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <file.csv>"
    exit 1
fi

awk -F',' '
BEGIN {
    name[8]="total_time"
    name[9]="build_time"
    name[10]="time_excl_build"
    name[11]="prep"
    name[12]="cg"
    name[13]="bap"
}
{ for (i=8;i<=13;i++) s[i]+=$i }
END {
    if (NR == 0) { print "Empty file"; exit 1 }
    printf "rows: %d\n", NR
    for (i=8;i<=13;i++) printf "%-16s avg = %.3f\n", name[i], s[i]/NR
}
' "$1"
