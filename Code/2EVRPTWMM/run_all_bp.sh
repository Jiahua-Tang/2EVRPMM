
#!/bin/bash
set -euo pipefail

INST_DIR="../../Data/Instances/Data"

for f in "$INST_DIR"/*.txt; do
  name="$(basename "$f" .txt)"
  echo "Submitting $name"
  sbatch Ruche/bap.sh "$name"
done

