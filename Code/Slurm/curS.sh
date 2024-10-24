#!/bin/bash

#SBATCH --job-name=C101C70Q80
#SBATCH --output=./Result/outputSlurm/%j.%x 
#SBATCH --time=00:50:00 
#SBATCH --ntasks=10
#SBATCH --partition=cpu_med
#SBATCH --cpus-per-task=10

# Load the same modules as environment configuration
#module load julia/1.4.0/gcc-9.2.0

julia configure_env.jl

# Run code
julia 2EVRPMM-V7-2index.jl
#julia  2EVRPMM-V7-z-2index.jl C101-70.txt 600 80 45  
#exit()
#echo "Your job is finished" | mail -s "Job Complete" tangjiahua98@gmail.com

