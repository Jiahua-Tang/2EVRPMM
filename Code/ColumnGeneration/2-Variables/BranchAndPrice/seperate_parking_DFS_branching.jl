using JuMP, CPLEX, Plots, Random, DataStructures, Combinatorics
import DataFrames
import HiGHS
import Plots
import SparseArrays
import Test  #src
include("Utiles.jl")
include("column_generation.jl")
include("branching_strategy.jl")

# ==================================================================================== #

# solveMasterProblem() 

function plotOriginal()
    x_vals = [coord[1] for coord in coor]
    y_vals = [coord[2] for coord in coor]
    gr()
    plt = scatter(x_vals[2+nb_parking:end], y_vals[2+nb_parking:end], markershape=:circle, label = "Customers")
    scatter!(plt, x_vals[2:1+nb_parking][parking_availability[2:1+nb_parking] .== 1], 
                y_vals[2:1+nb_parking][parking_availability[2:1+nb_parking] .== 1], 
                markershape=:utriangle, label="available")

    scatter!(plt, x_vals[2:1+nb_parking][parking_availability[2:1+nb_parking] .!= 1], 
                y_vals[2:1+nb_parking][parking_availability[2:1+nb_parking] .!= 1], 
                markershape=:utriangle, color=:white, label="unavailable")

    scatter!(plt, x_vals[1:1], y_vals[1:1], markershape=:square, label = "depot")
    node_labels = [string("N.",i) for i in 1:length(coor)]
    for i in 1:length(coor)
        annotate!(x_vals[i], y_vals[i]+0.3, text(node_labels[i],6))
    end
    display(plt)
end

function filter_2e_routes(routes, branch_info)
    # println("in filter function, show branching rule: ")
    # displayBranchingRule(branch_info)

    result = Vector{Route}()
    for route in routes
        valid = true

        for must_include in branch_info.must_include_combinations 
        # a route include a must-combination on the branch can be added into result
            if route.sequence[1] != must_include[1] && must_include[2] in route.sequence
                valid = false
                break
            end
        end    

        for forbidden in branch_info.forbidden_combinations 
        # a route nclude a forbidden-combination on the branch cannot be added into result
            if route.sequence[1] == forbidden[1] && forbidden[2] in route.sequence
                valid = false
                break
            end
        end

        for customers in branch_info.must_served_together
        # if a route include only one of must serve together customers cannot be added into result
            if (Int(customers[1] in route.sequence) + Int(customers[2] in route.sequence)) == 1
                valid = false
                break
            end
        end

        for customers in branch_info.forbidden_served_together
        # if a route include both of must serve together customers cannot be added into result
            if (customers[1] in route.sequence) && (customers[2] in route.sequence)
                valid = false
                break
            end
        end

        for parking in branch_info.forbidden_parkings
        # if a route include a forbidden parking cannot be added into result
            if route.sequence[1] == parking || route.sequence[end] == parking
                valid = false
                break
            end
        end

        if valid == true
            push!(result, route)
        end
    end

    return result
end

function filter_1e_routes(routes, branch_info)
    result = Vector{Route}()
    # result = routes

    # branch 1e route by special order set
    for (idx, route) in enumerate(routes)
        valide = true
        for sos in branch_info.special_order_set_1e 
            if sos.sequence == route.sequence
                valide = false
                break
            end
        end
        
        if valide
            push!(result, route)
        end
    end

    # ## branch 1e route by specific parking
    # for (idx, route) in enumerate(routes)
    #     if route.sequence[1] in branch_info.forbidden_parkings
            
    #     end 
    # end

    return result
end

function display2eRoutes(routes_2e_pool)
    # Display all routes grouped by depart parking
    println("size of 2e routes pool:  ",length(routes_2e_pool))
    for s in satellites
        println("Route starts from satellite $s: ")
        for r in routes_2e_pool
            if r.sequence[1] == s
                println("     ",r.sequence, "   Cost=", round(r.cost, digits=2))
            end
        end
    end
end

function branchingStrategyParking(x, y, routes_1, routes_2, branchingInfo::BranchingInfo)

    sorted_fractional_y = sort([r for r in 1:length(y) if 0 < y[r] < 1], by = r -> y[r] * (1 - y[r]), rev = true)
    reverse_route = checkExistanceReversedRoute(sorted_fractional_y, routes_2)
    continue_status = false

    used_parkings = getUsedParkings(y, routes_2)
    nb_mm_exceed = getNbUsedParkings(used_parkings) > nb_microhub

    if nb_mm_exceed
    ## Total number of microhub exceed the limit
        # result = branchOn1eRoute(branchingInfo, x, routes_1)
        result = branchOnParking(branchingInfo, x, y, routes_1, routes_2)
        if !isnothing(result)
            return false, result[1], result[2]
        else
            continue_status = true
        end
    end
    if !nb_mm_exceed || continue_status
        if !isempty([r for r in 1:length(x) if 0 < x[r] < 1])
            result = branchOnSpecialOrderSet(branchingInfo, x, routes_1)
            if !isnothing(result)
                return false, result[1], result[2]  
            end
        else
            if !isnothing(reverse_route) && length(reverse_route) > 3    
                ## if route A -> custs -> B and route B -> custs -> A exist at the same time
                result = branchOnReverseRoute(branchingInfo, reverse_route)
                if !isnothing(result)
                    ## branch on arc in reverse route
                    return true, result[1], result[2]
                else
                    ## no more arc in reverse route can be branched on
                    ## branch on assignment of parking-customer on reverse route
                    result = branchOnAssignmentParkingCustomer(branchingInfo, reverse_route)
                    if !isnothing(result)
                        return true, result[1], result[2]   
                    else
                        ## random branch   
                        @info "No more possible parking-customer assignment decision: Random branch on combination customers" 
                        result = branchOnRandomCombinationCustomers(branchingInfo, sorted_fractional_y, routes_2)          
                    end
                end
            else
            ## no reverse route exist
                if isempty(sorted_fractional_y)
                ## if solution is integral but not feasible, branch on usage of parking
                    result = branchOnArcParkingCustomer(branchingInfo, [r for r in 1:length(y) if y[r] == 1], routes_2)
                    if !isnothing(result)
                        return true, result[1], result[2]     
                    else
                        ## random branch   
                        @info "No more possible arc parking-customer combination decision: Random branch on combination customers"            
                        result = branchOnRandomCombinationCustomers(branchingInfo, sorted_fractional_y, routes_2)          
                    end
                else
                ## solution is fractional
                ## find most fractional route and branch on most visited customer
                    result = branchOnMostFractionalAndVisitedPair(branchingInfo, sorted_fractional_y, routes_2)
                    if !isnothing(result)
                        return true, result[1], result[2]     
                    else
                        ## random branch   
                        @info "No more possible fractional route's parking-customer combination decision: Random branch on combination customers"               
                        result = branchOnRandomCombinationCustomers(branchingInfo, sorted_fractional_y, routes_2)          
                    end
                end
            end
        end
    end
    return nothing, nothing
end

function branchAndPriceOnlyParking(routes_1e_pool, routes_2e_pool)

    result_optimal_solution = Vector{Float64}()
    result_branching_iter = Vector{Int}() # 0 stands for leaf, 1 stands for parking leaf

    root_branch = BranchingInfo(Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Int}(), Set{Int}(), Set{Int}(), Set{Int}(),Set{Route}(), 0)
    node_stack = [root_branch]
    
    optimal_solution_value = Inf
    optimal_solution = nothing

    num_iter = 1
    cg_status = true
    while !isempty(node_stack) && num_iter <= 10
        println("\n---------------------------------ITER B&P $num_iter---------------------------------")
        for v in node_stack
            displayBranchingRule(v,routes_1e_pool)
        end

        branchingInfo = pop!(node_stack) 
        print("\nCurrent stack")
        displayBranchingRule(branchingInfo, routes_1e_pool)

        ## start restricted linear column generation 
        filtered_1e_routes_pool = filter_1e_routes(routes_1e_pool, branchingInfo)
        filtered_2e_routes_pool = filter_2e_routes(routes_2e_pool, branchingInfo)

        println("\nNumber of all 2e routes: ", length(routes_2e_pool), ", number of filtered 2e routes: ", length(filtered_2e_routes_pool))
        
        if !cg_status
            rlmpResult = solveRestrictedMasterProblem(filtered_1e_routes_pool, filtered_2e_routes_pool, branchingInfo)
            x, y = rlmpResult[5], rlmpResult[6]
        else
            rlmpResult = column_generation(filtered_1e_routes_pool, filtered_2e_routes_pool, branchingInfo)           
            new_columns_generated, filtered_2e_routes_pool = rlmpResult[1], rlmpResult[2]
            for route in new_columns_generated
                push!(routes_2e_pool, route)
            end
            x, y = rlmpResult[3], rlmpResult[4]
        end
        
        if !isnothing(rlmpResult)
            ## start branching
            ## check integrality
            if isempty([r for r in 1:length(y) if 0 < y[r] < 1])&& isempty([r for r in 1:length(x) if 0 < x[r] < 1])
                @info "Integer 2e Solution Found!"
                optimal_solution, optimal_solution_value = checkOptimal(filtered_1e_routes_pool, filtered_2e_routes_pool, x, y, optimal_solution, optimal_solution_value)  
                cg_status = true
            else
                cg_status, left_branch, right_branch = branchingStrategyParking(x, y, filtered_1e_routes_pool, filtered_2e_routes_pool, branchingInfo)
                if !isnothing(left_branch)
                    push!(node_stack, left_branch)
                    push!(node_stack, right_branch) 
                end
            end

        else
            push!(result_branching_iter, 0)
            @info "Current Restricted Linear Master Problem Infeasible"
        end

        println("\nCurrent optimal solution: with objective value = $optimal_solution_value")
        if !isnothing(optimal_solution)
            for route in optimal_solution
                print("   ", route.sequence)
            end
            println("")
        end
        push!(result_optimal_solution, optimal_solution_value)
        num_iter += 1
    end

    # println("\nOptimal solution obtained by B&P with objective value $optimal_solution_value: ")
    # for route in optimal_solution
    #     if route == optimal_solution[1]
    #         print("     1e Route ")
    #     else
    #         print("     2e Route ")
    #     end
    #     println(route.sequence, "   Cost= ", round(route.cost, digits=2))
    # end

    println("\n==============================================================================\n")
    # # optimal_value = solveMasterProblem() 
    # optimal_value = 140.53629640
    # iteration = 1:length(result_optimal_solution)
    # # Choose colors for each marker label (you can customize this)

    # iteration = 1:length(result_optimal_solution)

    # # Marker colors and labels
    # colors = [:white, :green, :pink] 
    # labels = ["leaf node", "branch on parking"]
    
    # # Base line plot
    # lineplot = plot(
    #     iteration,
    #     result_optimal_solution,
    #     title = "Objective value",
    #     xlabel = "Iteration",
    #     ylabel = "Objective Value",
    #     linewidth = 2,
    #     linecolor = :black,
    #     ylims = (optimal_value - 5, maximum(result_optimal_solution) + 5),
    #     label = false,
    #     ,m. = :outertopright,
    #     legendfontsize = 6
    # )
    
    # # Plot each type separately to get legend entries
    # for t in 0:1
    #     idxs = findall(x -> x == t, result_branching_iter)
    #     scatter!(
    #         iteration[idxs],
    #         result_optimal_solution[idxs],
    #         marker = (:circle, 3),
    #         markercolor = colors[t+1],
    #         label = labels[t+1]
    #     )
    # end
    # hline!(lineplot, [result_optimal_solution[end]], linestyle = :dash, label = false, linewidth = 1, color = :black)
    # hline!([optimal_value], linestyle = :dash, label = "optimal master problem", linewidth = 2)
    # display(lineplot)
    # display2eRoutes(routes_2e_pool)
end

# ============================================================================== #

const data = initializeData(5,3,8)

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

## Initial Feasible Routes
routes_1 = generateBest1eRoutes(minimum_parkings_required)
routes_2 = generate2eInitialRoutes()
# for route in routes_1
#     println(route.sequence)
# end


# routes_1, routes_2 = generateAllRoutes()

# # Display initial setting
# for (idx, v) in enumerate(parking_availability)
#     println("parking_availability[$idx] = $v")    
# end

# for route in routes_1 
#     println(route.sequence, "   ", getServedParking1eRoute(route))
# end


# println("")
# for route in routes_1 
#     println(route.sequence)
# end
# println("")
# for route in routes_2
#     println(route.sequence)
# end
# plotOriginal()


branchAndPriceOnlyParking(routes_1, routes_2)

# solveMasterProblem()

