using Plots, Random, DataStructures, Combinatorics, Printf, 
    HiGHS, SparseArrays, Test, DataFrames, CPLEX, JuMP, Dates, Base.Threads, CPUTime
using Logging, LoggingExtras

include("Utiles.jl")
include("BranchAndPrice/Utiles.jl")
include("BranchAndPrice/branchAndPrice.jl")
include("CompactModel/compactModel.jl")
include("LrpLowerBound/solveLRP.jl")

folder = "../../Data/NicoInstances"

for file in readdir(folder)
    path = joinpath(folder, file)

    # Only process .txt files (avoid hidden/system files)
    endswith(file, ".txt") || continue

    println("Reading $file")
    read_nico_dataset(path)
end