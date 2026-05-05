#!/bin/bash
# Submit bap.sh for every instance of a given customer-count size.
# Usage:  ./run_bap_size.sh <size>
#   e.g.  ./run_bap_size.sh 20
set -euo pipefail

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <size>"
    exit 1
fi

SIZE="$1"
INST_DIR="../../Data/Instances/Data"

count=0
for f in "$INST_DIR"/*,"${SIZE}".txt; do
    [ -e "$f" ] || continue
    name="$(basename "$f" .txt)"
    echo "Submitting $name"
    sbatch Ruche/bap.sh "$name"
    count=$((count + 1))
done

if [ "$count" -eq 0 ]; then
    echo "No instances found matching pattern *,${SIZE}.txt in $INST_DIR"
    exit 1
fi
echo "Submitted $count job(s) for size $SIZE"
