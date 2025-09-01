using Plots, Random, DataStructures, Combinatorics, Printf, 
    HiGHS, SparseArrays, Test, DataFrames, CPLEX, JuMP

include("Utiles.jl")
include("BranchAndPrice/Utiles.jl")
include("BranchAndPrice/branchAndPrice.jl")
include("CompactModel/compactModel.jl")
include("LrpLowerBound/solveLRP.jl")


global root = "$(pwd())/TEST/"
# global root = "/gpfs/workdir/tangj/2EVRPMM/Code/Code/"

generateData()
# readData("E-n33-k4.txt", ARGS)

# routes_1e = Vector{Route}()
# push!(routes_1e, generate1eRoute([1,3,1]))
# global routes_2e = generate2eInitialRoutes()
# routes_2e_pool = filter2eRoute([3])
# routes_originated_p = Vector{Vector{Int}}()
# for s in satellites
#     routes = Vector{Int}()
#     for (r,route) in enumerate(routes_2e_pool)
#         if route.sequence[1] == s
#             push!(routes,r)
#         end
#     end
#     push!(routes_originated_p, routes)
# end

# model = Model(CPLEX.Optimizer)

# @variable(model, 1 >= x[1:length(routes_1e)] >= 0)
# @variable(model, 1 >= y[1:length(routes_2e_pool)] >= 0)

# @constraint(model, sync[s in satellites], sum(route.b2out[s] * y[r] for (r, route) in enumerate(routes_2e_pool))
#     -nb_vehicle_per_satellite * sum(route.b1[s] * x[r] for (r, route) in enumerate(routes_1e))<=0)
# @constraint(model, custVisit[i in customers], 1 - sum(route.a[i-1-length(satellites)] * y[r] for (r,route) in enumerate(routes_2e_pool)) <= 0 )
# @constraint(model, number2evfixe[s in satellites], sum(route.b2in[s] * y[r] for (r, route) in enumerate(routes_2e_pool)) == sum(route.b2out[s] * y[r] for (r, route) in enumerate(routes_2e_pool)))
# @constraint(model, maxVolumnMM[s in satellites], sum( routes_2e_pool[r].a[i-1-length(satellites
# )]*demands[i]*y[r] for r in routes_originated_p[s-1] for i in customers) - capacity_microhub <= 0)
# @constraint(model, single1eV, sum(x[r] for (r,_) in enumerate(routes_1e))>=1)
# @constraint(model, min2eRoute, sum(y[r] for (r,_) in enumerate(routes_2e_pool))>=ceil(sum(demands)/capacity_2e_vehicle))

# @objective(model, Min, sum(y[r] * route.cost for (r,route) in enumerate(routes_2e_pool)) + 
#                     sum(x[r] * route.cost for (r,route) in enumerate(routes_1e)))

# optimize!(model)

# println("Objective value = ", round(objective_value(model), digits=2))

# for (idx, route) in enumerate(routes_1e)
#     if round(value(x[idx]), digits=2) != 0
#         println("1e route: ", route.sequence, " cost = ", round(route.cost, digits=2), " x[$idx] = ", round(value(x[idx]), digits=2))
#     end 
# end
# for (idx, route) in enumerate(routes_2e_pool)
#     if round(value(y[idx]), digits=2) != 0
#         println("2e route: ", route.sequence, " cost = ", round(route.cost, digits=2), " y[$idx] = ", round(value(y[idx]), digits=2))
#     end 
# end

# getdual(model)


#=========================================================#

# solveCompactModelDisplayResult() 
# # 40 customers takes 470.89 sec to find optimal

#=========================================================#

# solveMasterProblem()

#=========================================================#

lb_lrp_per_route = calculateLRPLowerBoundByParking()
displayLRPLowerBound(deepcopy(lb_lrp_per_route))

global routes_2e = generate2eInitialRoutes()
global num_iter_global = 1
global upperBound = Inf
global optimalSolution = nothing

global optimal_found_iteration = 0
global execution_time_subproblem = 0
global execution_time_rmp = 0
global solving_rmp_time = 0
global filtering_time = 0
global deepest_level = 0
global optimal_found_in = 0

execution_time = @elapsed begin
    while !isempty(lb_lrp_per_route)  && num_iter_global < 2
        min_value, min_route = findmin(lb_lrp_per_route)
        if min_value > upperBound   break   end

        route_1e = Vector{Route}()
        push!(route_1e, min_route)
        branchAndPriceWithScore(route_1e)
        delete!(lb_lrp_per_route, min_route)
        @info "current upper bound is $(round(upperBound,digits=2))"
        global num_iter_global
        num_iter_global += 1
    end
end



# if !isnothing(optimalSolution)
#     @info "Current optimal solution found in interation $optimal_found_iteration:"
#     for route in optimalSolution 
#         println(route.sequence)
#     end   
# end
# println("Execution time = $(round(execution_time, digits=2)), time spent in solving subproblem = $(round(execution_time_subproblem, digits=2)), takes percentage of $(round(execution_time_subproblem/execution_time,digits=2)*100)%, deepest node dived to level $deepest_level")

#===============================================================================================#

# execution_time = @elapsed begin
#         route_1e = Vector{Route}()
#         push!(route_1e, generate1eRoute([1,3,4,5,2,1]))
#         branchAndPriceWithScore(route_1e)
#         @info "current upper bound is $(round(upperBound,digits=2)) "
#         global num_iter
#         num_iter += 1
#     end
if !isnothing(optimalSolution)
    println("\nExecution time = $(round(execution_time, digits=2))") 
    println("time spent in solving subproblem = $(round(execution_time_subproblem, digits=2)), takes percentage of $(round(execution_time_subproblem/execution_time,digits=2)*100)%")
    println("time spent in filtering = $(round(filtering_time, digits=2)), takes percentage of $(round(filtering_time/execution_time, digits =2)*100)%")
    println("time spent in total RMP = $(round(execution_time_rmp, digits = 2)), takes percentage of $(round(execution_time_rmp/execution_time, digits =2)*100)%")
    println("time spent in soving RMP = $(round(solving_rmp_time, digits = 2)), takes percentage of $(round(solving_rmp_time/execution_time, digits =2)*100)%")

    println("deepest node dived to level $deepest_level\n")
    # println("Current optimal solution found in interation $optimal_found_iteration in level $optimal_found_in:")
    for route in optimalSolution 
        println(route.sequence)
    end   
end

# solveRMP(generate1eRoute([1,3,6,2,5,1]))