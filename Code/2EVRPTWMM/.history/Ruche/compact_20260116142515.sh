#!/bin/bash

#SBATCH --job-name=C20
#SBATCH --output=/gpfs/workdir/tangj/2EVRPMM/Code/Result/outputSlurm/TW/%j.%x.out
#SBATCH --error=/gpfs/workdir/tangj/2EVRPMM/Code/Result/outputSlurm/TW/%j.%x.err  # Capture standard error in a separate file
#SBATCH --time=04:00:00 
#SBATCH --ntasks=1   # Since Julia typically uses multithreading, setting ntasks to 1
#SBATCH --cpus-per-task=1
#SBATCH --partition=cpu_med
#SBATCH --mail-type=END
#SBATCH --mail-type=FAIL

# Load Julia module (uncomment this if the environment requires it)
module load julia/1.11.5/gcc-13.2.0

# Error handling: If any command fails, exit the script
set -e

# Load environment (if needed)
# julia /gpfs/users/tangj/2EVRPMM/Code/configure_env.jl

# Run the main Julia code
julia /gpfs/workdir/tangj/2EVRPMM/Code/2EVRPTWMM/Ruche/ruche_compact.jl "$1" "$2" "$3" "$4"

