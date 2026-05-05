#!/bin/bash
# Submit compact.sh for every instance of a given customer-count size.
# Usage:  ./run_compact_size.sh <size>
#   e.g.  ./run_compact_size.sh 20     -> submits ce1-3,5,20  cf2-2,3,20  ...
#         ./run_compact_size.sh 30     -> submits all *,30 instances
set -euo pipefail

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <size>"
    exit 1
fi

SIZE="$1"
INST_DIR="../../Data/Instances/Data"

count=0
for f in "$INST_DIR"/*,"${SIZE}".txt; do
    [ -e "$f" ] || continue        # skip the literal pattern if no match
    name="$(basename "$f" .txt)"
    echo "Submitting $name"
    sbatch Ruche/compact.sh "$name"
    count=$((count + 1))
done

if [ "$count" -eq 0 ]; then
    echo "No instances found matching pattern *,${SIZE}.txt in $INST_DIR"
    exit 1
fi
echo "Submitted $count job(s) for size $SIZE"
