#!/bin/bash

#SBATCH --job-name=RC104
#SBATCH --output=/gpfs/users/tangj/2EVRPMM/Code/Result/outputSlurm/InstancesC100/%j.%x.out
#SBATCH --error=/gpfs/users/tangj/2EVRPMM/Code/Result/outputSlurm/InstancesC100/%j.%x.err  # Capture standard error in a separate file
#SBATCH --time=06:00:00 
#SBATCH --ntasks=1   # Since Julia typically uses multithreading, setting ntasks to 1
#SBATCH --cpus-per-task=10                      # Ensure you're using all 10 CPUs
#SBATCH --partition=cpu_prod

# Load Julia module (uncomment this if the environment requires it)
#module load julia/1.4.0/gcc-9.2.0

# Error handling: If any command fails, exit the script
set -e

# Load environment (if needed)
julia /gpfs/users/tangj/2EVRPMM/Code/configure_env.jl

# Run the main Julia code
julia /gpfs/users/tangj/2EVRPMM/Code/2EVRPMM-V7-z-2index.jl 100/RC104.txt 600 50 340
# julia /gpfs/users/tangj/2EVRPMM/Code/2EVRPMM-compare-result-parking.jl 100/RC104.txt 600 50 340

