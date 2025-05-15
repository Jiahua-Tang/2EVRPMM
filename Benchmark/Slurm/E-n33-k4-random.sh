#!/bin/bash

#SBATCH --job-name=E-n33-k4
#SBATCH --output=/gpfs/workdir/tangj/2EVRPMM/Benchmark/Result/outputSlurm/E-n33-k4/%j.%x.out
#SBATCH --error=/gpfs/workdir/tangj/2EVRPMM/Benchmark/Result/outputSlurm/%j.%x.err  # Capture standard error in a separate file
#SBATCH --time=06:00:00 
#SBATCH --ntasks=1   # Since Julia typically uses multithreading, setting ntasks to 1
#SBATCH --cpus-per-task=10                      # Ensure you're using all 10 CPUs
#SBATCH --partition=cpu_prod

# Load Julia module (uncomment this if the environment requires it)
#module load julia/1.4.0/gcc-9.2.0

# Error handling: If any command fails, exit the script
set -e

# Load environment (if needed)
julia /gpfs/workdir/tangj/2EVRPMM/Code/CompactModel/configure_env.jl

# Run the main Julia code
julia /gpfs/workdir/tangj/2EVRPMM/Code/CompactModel/main_random_parking.jl E-n33-k4.txt 4 80
