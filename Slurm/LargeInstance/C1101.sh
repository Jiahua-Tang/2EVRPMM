#!/bin/bash

#SBATCH --job-name=L-C1101
#SBATCH --output=/gpfs/users/tangj/2EVRPMM/Result/outputSlurm/LargeInstance/%j.%x.out
#SBATCH --error=/gpfs/users/tangj/2EVRPMM/Result/outputSlurm/LargeInstance/%j.%x.err  # Capture standard error in a separate file
#SBATCH --time=20:00:00 
#SBATCH --mail-type=ALL
#SBATCH --ntasks=1   # Since Julia typically uses multithreading, setting ntasks to 1
#SBATCH --cpus-per-task=10
#SBATCH --partition=cpu_long

# Load Julia module (uncomment this if the environment requires it)
#module load julia/1.4.0/gcc-9.2.0

# Error handling: If any command fails, exit the script
set -e

# Load environment (if needed)
julia /gpfs/users/tangj/2EVRPMM/Code/v1/configure_env.jl

# Run the main Julia code
julia /gpfs/users/tangj/2EVRPMM/Code/v1/main.jl C1_10_1.txt 1170

