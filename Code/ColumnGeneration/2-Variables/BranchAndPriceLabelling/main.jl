using JuMP, CPLEX, Plots, Random, DataStructures, Combinatorics, Printf
import DataFrames
import HiGHS
import Plots
import SparseArrays
import Test  #src
include("Utiles.jl")
include("column_generation.jl")
include("branching_strategy.jl")
include("fixed1eRoute.jl")



# solveMasterProblem()
const data = initializeData(6,3,15)

const coor = data[1]
const nb_parking = data[2]
const nb_microhub = data[3]
const parking_availability = data[4]
const demands = data[5]
const arc_cost = data[6]

## Define parameters
const nb_vehicle_per_satellite = 10
const capacity_2e_vehicle = 30
const capacity_microhub = 35
const maximum_duration_2e_vehicle = 50
const minimum_parkings_required = Int(ceil(sum(demands)/capacity_microhub))

## Define set
const points = 1:length(demands)
const customers = 2 + nb_parking : length(coor)
const satellites = 2:1 + nb_parking
const A2 = 2:length(coor)
const A1 = 1:1+length(satellites)

for cust in customers
    println("Customer $cust has demand $(demands[cust])")
end


# savefig(plotOriginal(), "figure.png")

# solveMasterProblem()

#===============================================================================================#

routes_1e_complete = generateNonDominate1eRoutes(minimum_parkings_required)
lb_lrp_per_route, lower_bounds = calculate_lrp_lower_bound(routes_1e_complete)
lb_lrp_per_route_copy = deepcopy(lb_lrp_per_route)
global count = 1
while !isempty(lb_lrp_per_route_copy)
    min_value, min_idx = findmin(lb_lrp_per_route_copy)
    println(count, ".  ",routes_1e_complete[min_idx].sequence,"  ",getServedParking1eRoute(routes_1e_complete[min_idx]),"   lower bound= $(round(min_value, digits=2))")
    delete!(lb_lrp_per_route_copy, min_idx)
    global count
    count += 1
end


global num_iter_global = 1
global upperBound = 273.25
global optimalSolution = nothing
global routes_2e = generate2eInitialRoutes()

# for route in routes_2e 
#     println(route.sequence)
# end

global optimal_found_iteration = 0
global execution_time_subproblem = 0
# global execution_time_filter
global deepest_level = 0
global optimal_found_in = 0

execution_time = @elapsed begin
    while !isempty(lb_lrp_per_route) # && num_iter_global
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