#!/bin/bash

#SBATCH --job-name=testInstance
#SBATCH --output=./outputSlurm/%x.o%j 
#SBATCH --time=00:20:00 
#SBATCH --ntasks=10
#SBATCH --partition=cpu_short

# Load the same modules as environment configuration
#module load julia/1.4.0/gcc-9.2.0

julia configure_env.jl

# Run code
julia  2EVRPMM-V7-z-2index.jl

