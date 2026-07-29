#!/bin/bash

#SBATCH --job-name=Baseline
#SBATCH --output=/gpfs/workdir/tangj/2EVRPMM/Code/Result/outputSlurm/TW/%j.%x.out
#SBATCH --error=/gpfs/workdir/tangj/2EVRPMM/Code/Result/outputSlurm/TW/%j.%x.err
#SBATCH --time=04:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --partition=cpu_med
#SBATCH --mail-type=END
#SBATCH --mail-type=FAIL

# Load Julia module
module load julia/1.11.5/gcc-13.2.0

# Error handling: If any command fails, exit the script
set -e

# Run baseline
julia /gpfs/workdir/tangj/2EVRPMM/Code/2EVRPTWMM/baseline.jl "$1"
