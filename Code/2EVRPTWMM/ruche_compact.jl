using Plots, Random, DataStructures, Combinatorics, Printf, 
    HiGHS, SparseArrays, Test, DataFrames, CPLEX, JuMP, Dates, Base.Threads
using Logging, LoggingExtras

include("Utiles.jl")
include("BranchAndPrice/Utiles.jl")
include("BranchAndPrice/branchAndPrice.jl")
include("CompactModel/compactModel.jl")
include("LrpLowerBound/solveLRP.jl")


global root = "$(pwd())/TEST/"
# global root = "/gpfs/workdir/tangj/2EVRPMM/Code/Code/"


# instance_size = 70
global random_seed = 42
time_stamp = "_"*Dates.format(now(), "ddmmyy_HHMM")
num_cust = 20
file_name = "Output/c$(num_cust)"*"s"*string(random_seed)*time_stamp*".txt"
# file_name = "Output/demo.txt"
mkpath(dirname(file_name))

open(file_name, "w") do io
    redirect_stdout(io) do
        # redirect_stderr(io) do

#            # generateData(instance_size, random_seed)
            global fileName = "R103"
            # read_Solomon_Dataset_TW("../../Data/Demo/100/" * fileName * ".txt", 1200)
            retrieve_solomon_random_data("../../Data/Demo/100/" * fileName * ".txt", 1200, num_cust)
            println("\n================================================================")

            solveCompactModelDisplayResult()
    end
end
