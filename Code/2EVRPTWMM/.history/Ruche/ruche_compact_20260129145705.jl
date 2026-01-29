using Plots, Random, DataStructures, Combinatorics, Printf, 
    HiGHS, SparseArrays, Test, DataFrames, CPLEX, JuMP, Dates, Base.Threads
using Logging, LoggingExtras

include("../Utiles.jl")
include("../BranchAndPrice/Utiles.jl")
include("../BranchAndPrice/branchAndPrice.jl")
include("../CompactModel/compactModel.jl")
include("../LrpLowerBound/solveLRP.jl")


global root = "$(pwd())/../../Data/Instances/"


time_stamp = Dates.format(now(), "ddmmyy_HHMM")
num_cust = parse(Int, ARGS[1])
global random_seed = parse(Int, ARGS[2])
time_limit = parse(Int, ARGS[3])
name_diff = parse(Int, ARGS[4])

file_name = "Output/c$(num_cust)"*"s"*string(random_seed)*"t"*string(time_limit)*"_"*time_stamp*"_"*string(name_diff)*".txt"
# file_name = "Output/demo.txt"
mkpath(dirname(file_name))

open(file_name, "w") do io
    redirect_stdout(io) do
        # redirect_stderr(io) do

#            # generateData(instance_size, random_seed)

            # global fileName = "R103"
            # retrieve_solomon_random_data("../../Data/Demo/100/" * fileName * ".txt", time_limit, num_cust)

            read_nico_dataset("../../Data/Instances/Data/ce1-2,3,15.txt")
            println("\n================================================================")
            execution_time = @time @CPUtime solveCompactModelDisplayResult()
            # println("CPU time = ", t1 - t0, " seconds")
    end
end

# scp -r /path/to/local/folder username@remote_host:/path/on/remote/