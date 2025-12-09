using Plots, Random, DataStructures, Combinatorics, Printf, 
    HiGHS, SparseArrays, Test, DataFrames, CPLEX, JuMP

include("Utiles.jl")
include("BranchAndPrice/Utiles.jl")
# include("BranchAndPrice/branchAndPrice.jl")
include("BranchAndPriceColuna/branchAndPriceColuna.jl")
include("CompactModel/compactModel.jl")
include("LrpLowerBound/solveLRP.jl")


global root = "$(pwd())/TEST/"
# global root = "/gpfs/workdir/tangj/2EVRPMM/Code/Code/"

generateData()

routes_2e = generate2eInitialRoutes()
branchAndPriceColuna([2,4], routes_2e)