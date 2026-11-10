#!/bin/bash
# Submit bap.sh for every instance of a given customer-count size.
# Usage:  ./run_bap_size.sh <size> [mode]
#   mode: full (default), random_dfs, or baseline (F3: fixed microhubs)
#   e.g.  ./run_bap_size.sh 20
#         ./run_bap_size.sh 30 random_dfs
#         ./run_bap_size.sh 50 baseline
set -euo pipefail

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    echo "Usage: $0 <size> [full|random_dfs|baseline]"
    exit 1
fi

SIZE="$1"
MODE="${2:-full}"
INST_DIR="../../Data/Instances/Data"

case "$MODE" in
    full|random_dfs|baseline) ;;
    *) echo "Unknown mode '$MODE' (use full, random_dfs or baseline)"; exit 1 ;;
esac

count=0
for f in "$INST_DIR"/*,"${SIZE}".txt; do
    [ -e "$f" ] || continue
    name="$(basename "$f" .txt)"
    echo "Submitting $name"
    sbatch Ruche/bap.sh "$name" "$MODE"
    count=$((count + 1))
done

if [ "$count" -eq 0 ]; then
    echo "No instances found matching pattern *,${SIZE}.txt in $INST_DIR"
    exit 1
fi
echo "Submitted $count job(s) for size $SIZE, mode $MODE"
