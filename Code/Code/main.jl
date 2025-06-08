using Plots, Random, DataStructures, Combinatorics, Printf, 
    HiGHS, SparseArrays, Test, DataFrames, CPLEX, JuMP

include("Utiles.jl")
include("BranchAndPrice/Utiles.jl")
include("BranchAndPrice/branchAndPrice.jl")
include("CompactModel/compactModel.jl")

global execution_time_limit = 120 # seconds
generateData()

#=========================================================#

# solveCompactModelDisplayResult()

#=========================================================#

# solvemasterProblem()

#=========================================================#


routes_1e_complete = generateNonDominate1eRoutes(minimum_parkings_required)
lb_lrp_per_route = calculateLRPLowerBound(routes_1e_complete)
displayLRPLowerBound(deepcopy(lb_lrp_per_route), routes_1e_complete)

global num_iter_global = 1
global upperBound = Inf
global optimalSolution = nothing
global routes_2e = generate2eInitialRoutes()
global optimal_found_iteration = 0
global execution_time_subproblem = 0
global deepest_level = 0
global optimal_found_in = 0

execution_time = @elapsed begin
    while !isempty(lb_lrp_per_route) # && num_iter_global < 11
        min_value, min_idx = findmin(lb_lrp_per_route)
        if min_value > upperBound   break   end

        route_1e = Vector{Route}()
        push!(route_1e, routes_1e_complete[min_idx])
        branchAndPriceWithScore(route_1e)
        delete!(lb_lrp_per_route, min_idx)
        @info "current upper bound is $(round(upperBound,digits=2))"
        global num_iter_global
        num_iter_global += 1
    end
end

if !isnothing(optimalSolution)
    @info "Current optimal solution found in interation $optimal_found_iteration:"
    for route in optimalSolution 
        println(route.sequence)
    end   
end
println("Execution time = $(round(execution_time, digits=2)), time spent in solving subproblem = $(round(execution_time_subproblem, digits=2)), takes percentage of $(round(execution_time_subproblem/execution_time,digits=2)*100)%, deepest node dived to level $deepest_level")

#===============================================================================================#

# execution_time = @elapsed begin
#         route_1e = Vector{Route}()
#         push!(route_1e, generate1eRoute([1,3,4,5,2,1]))
#         branchAndPriceWithScore(route_1e)
#         @info "current upper bound is $(round(upperBound,digits=2)) "
#         global num_iter
#         num_iter += 1
#     end
# if !isnothing(optimalSolution)
#     println("\nExecution time = $(round(execution_time, digits=2)), time spent in solving subproblem = $(round(execution_time_subproblem, digits=2)), takes percentage of $(round(execution_time_subproblem/execution_time,digits=2)*100)%, deepest node dived to level $deepest_level")
#     println("Current optimal solution found in interation $optimal_found_iteration in level $optimal_found_in:")
#     for route in optimalSolution 
#         println(route.sequence)
#     end   
# end

# solveRMP(generate1eRoute([1,3,6,2,5,1]))