#!/bin/bash

#SBATCH --job-name=C101C70F50
#SBATCH --output=./Result/outputSlurm/%x.o%j 
#SBATCH --time=00:50:00 
#SBATCH --ntasks=10
#SBATCH --partition=cpu_med
#SBATCH --cpus-per-task=10

# Load the same modules as environment configuration
#module load julia/1.4.0/gcc-9.2.0

julia configure_env.jl

# Run code
julia  2EVRPMM-V7-z-2index.jl C101-70.txt 600 50 45  


