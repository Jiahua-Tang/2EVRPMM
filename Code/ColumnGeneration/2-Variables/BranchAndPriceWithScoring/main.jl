using JuMP, CPLEX, Plots, Random, DataStructures, Combinatorics
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
const data = initializeData(8,4,15)

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


routes_1e_complete = generateNonDominate1eRoutes(minimum_parkings_required)

println("==========================")


lb_lrp_per_route, lower_bounds = calculate_lrp_lower_bound(routes_1e_complete)
lb_lrp_per_route_copy = deepcopy(lb_lrp_per_route)
while !isempty(lb_lrp_per_route_copy)
    min_value, min_idx = findmin(lb_lrp_per_route_copy)
    println(routes_1e_complete[min_idx].sequence,"  ",getServedParking1eRoute(routes_1e_complete[min_idx]),"   lower bound= $(round(min_value, digits=2))")
    delete!(lb_lrp_per_route_copy, min_idx)
end


global num_iter = 1
global upperBound = Inf
global optimalSolution = nothing
global routes_2e

# ## Initial Feasible Routes
routes_2e = generate2eInitialRoutes()

execution_time = @elapsed begin
    while !isempty(lb_lrp_per_route) && num_iter < 2
        min_value, min_idx = findmin(lb_lrp_per_route)
        if min_value > upperBound   break   end

        route_1e = Vector{Route}()
        push!(route_1e, routes_1e_complete[min_idx])
        # push!(route_1e, generate1eRoute([1,2,6,4,1]))

        # branchAndPricePer1eSubproblem(route_1e, routes_2e)
        branchAndPriceWithScore(route_1e, routes_2e)

        delete!(lb_lrp_per_route, min_idx)
        # @info "current upper bound is $(round(upperBound,digits=2)) "

        global num_iter
        num_iter += 1
    end
end

if !isnothing(optimalSolution)
    for route in optimalSolution 
        println(route.sequence)
    end   
end


println("Execution time = $execution_time")


# solveMasterProblem()