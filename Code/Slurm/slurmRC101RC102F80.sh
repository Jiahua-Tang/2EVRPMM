#!/bin/bash

#SBATCH --job-name=testRC101RC102F80
#SBATCH --output=./Result/outputSlurm/%x.o%j 
#SBATCH --time=06:00:00 
#SBATCH --ntasks=10
#SBATCH --partition=cpu_prod
#SBATCH --cpus-per-task=10

# Load the same modules as environment configuration
#module load julia/1.4.0/gcc-9.2.0

julia configure_env.jl

# Run code
julia  2EVRPMM-V7-z-2index.jl 100/RC101.txt 600 80 2.8 
julia  2EVRPMM-V7-z-2index.jl 100/RC102.txt 600 80 2.8

