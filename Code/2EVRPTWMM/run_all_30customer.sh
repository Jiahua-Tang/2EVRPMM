#!/bin/bash
# Run branch-and-price locally on every 30-customer instance (cx-x,x,30.txt),
# one at a time in this shell, and report the total number of B&P nodes
# explored across all of them.
#
# Usage: ./run_all_30customer.sh
#
# Per-instance results are appended to Output/bap_30customer_nodes_summary.csv
# (";"-delimited -- instance names contain commas themselves, e.g.
# "ce1-2,3,30" -- columns: timestamp, instance, num_customers,
# num_satellites, branching_mode, search_mode, total_bap_nodes, objective,
# total_time_seconds). branching_mode/search_mode record which
# random_pair_branching/dfs_node_selection flags were active in
# run_bap_instance.jl at the time, so rows stay identifiable even if results
# from different modes ever end up in the same file.
# Per-instance logs go to Output/bap_<instance>.txt (same as
# run_bap_instance.jl always writes).
set -euo pipefail

INST_DIR="../../Data/Instances/Data"
SUMMARY_CSV="Output/bap_30customer_nodes_summary.csv"

mkdir -p Output
rm -f "$SUMMARY_CSV"   # start fresh so the sum below reflects only this run

count=0
for f in "$INST_DIR"/*.txt; do
    name="$(basename "$f" .txt)"
    # Only run instances with 30 customers, i.e. filenames ending in ",30"
    # (cx-x,x,30) -- everything else in the folder is skipped.
    if [[ "$name" == *,30 ]]; then
        echo "Running $name ..."
        julia run_bap_instance.jl "$name"
        count=$((count + 1))
    fi
done

if [ "$count" -eq 0 ]; then
    echo "No 30-customer instances (*,30.txt) found in $INST_DIR"
    exit 1
fi

echo ""
echo "Ran $count instance(s). Per-instance node counts:"
awk -F';' '{printf "  %-20s [%s/%s] %s nodes\n", $2, $5, $6, $7}' "$SUMMARY_CSV"

total_nodes=$(awk -F';' '{sum += $7} END {print sum}' "$SUMMARY_CSV")
echo ""
echo "Total B&P nodes explored across all $count instances: $total_nodes"
