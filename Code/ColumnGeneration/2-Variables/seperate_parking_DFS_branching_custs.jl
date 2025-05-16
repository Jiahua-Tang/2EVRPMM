using JuMP, CPLEX
import DataFrames
import HiGHS
import Plots
using Plots
import SparseArrays
using Random
using DataStructures
using Combinatorics
import Test  #src
include("BranchAndPrice/Utiles.jl")
include("BranchAndPrice/column_generation.jl")
include("BranchAndPrice/branching_strategy.jl")


# ============================================================================== #

const data = initializeData(2)

const coor = data[1]
const nb_parking = data[2]
const nb_microhub = data[3]
const parking_availability = data[4]
const demands = data[5]
const arc_cost = data[6]

# for (idx, p) in enumerate(parking_availability) 
#     println("parking_availability[$idx] = ",Int(p))
# end

# Define parameters
const nb_vehicle_per_satellite = 10
const capacity_2e_vehicle = 30
const capacity_microhub = 35
const maximum_duration_2e_vehicle = 50

# Define set
const points = 1:length(demands)
const customers = 2 + nb_parking : length(coor)
const satellites = 2:1 + nb_parking
const A2 = 2:length(coor)
const A1 = 1:1+length(satellites)

# Initial Feasible Routes
# routes_1 = generate1eInitialSolition()
routes_1 = generateBest1eRoutes(Int(ceil(sum(demands)/capacity_microhub)))
routes_2 = generate2eInitialRoutes()

# ==================================================================================== #

# solveMasterProblem() 

# ==================================================================================== #

# for (r, route) in enumerate(routes_1)
#     print(route.sequence,"   Available parking:  ")
#     for (idx, availability) in enumerate(route.b1)
#         if availability==1
#             print(idx," ")
#         end
#     end
#     println("")
# end


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

scatter!(plt, x_vals[1:1], y_vals[1:1], markershape=:circle, label = "depot")
node_labels = [string("N.",i) for i in 1:length(coor)]
for i in 1:length(coor)
    annotate!(x_vals[i], y_vals[i]+0.3, text(node_labels[i],6))
end
# display(plt)


# ==================================================================================== #

function filter_2e_routes(routes, branch_info)
    # println("in filter function, show branching rule: ")
    # displayBranchingRule(branch_info)

    result = Vector{Route}()
    for route in routes
        valid = true

        for must_include in branch_info.must_include_combinations 
            # for a route with a must include combination on the branch
            if route.sequence[1] != must_include[1] && must_include[2] in route.sequence
                valid = false
                break
            end
        end    

        for forbidden in branch_info.forbidden_combinations 
            if route.sequence[1] == forbidden[1] && forbidden[2] in route.sequence
                valid = false
                break
            end
        end

        for customers in branch_info.must_served_together
            # if a route include only one of must serve together customers
            if (Int(customers[1] in route.sequence) + Int(customers[2] in route.sequence)) == 1
                valid = false
            end
        end

        for customers in branch_info.forbidden_served_together 
            if (customers[1] in route.sequence) && (customers[2] in route.sequence)
                valid = false
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
    for route in routes 
        valide = true
        for shorterRule in branch_info.shorter_than 
            if sum(route.b1) >= shorterRule
                valide = false
                break
            end
        end
        
        for longerRule in branch_info.longer_than
            if sum(route.b1) <= longerRule
                valide = false
                break
            end            
        end

        if valide == true
            push!(result, route)
        end
    end
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

function branchAndPriceOnlyCustomers(routes_1e_pool, routes_2e_pool)
    root_branch = BranchingInfo(Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Int}(), Set{Int}(), 0)
    node_stack = [root_branch]
    
    optimal_solution_value = Inf
    optimal_solution = nothing

    num_iter = 1
    while !isempty(node_stack) && num_iter <= 5
        println("\n---------------------------------ITER B&P $num_iter---------------------------------")
        for v in node_stack
            displayBranchingRule(v)
        end

        branchingInfo = pop!(node_stack)  # DFS: pop from end
        println("\nCurrent stack")
        displayBranchingRule(branchingInfo)

        filtered_1e_routes_pool = routes_1e_pool
        filtered_2e_routes_pool = filter_2e_routes(routes_2e_pool, branchingInfo)

        println("number of all 2e routes: ", length(routes_2e_pool), ", number of filtered 2e routes: ", length(filtered_2e_routes_pool))
        
        rlmpResult = solve_1e2e_iter_loop(filtered_1e_routes_pool, filtered_2e_routes_pool, branchingInfo)
        if !isnothing(rlmpResult)
            new_columns_generated = rlmpResult[1]
            filtered_2e_routes_pool = rlmpResult[2]
            for route in new_columns_generated
                push!(routes_2e_pool, route)
            end
            x, y = rlmpResult[3], rlmpResult[4]

            # feasible solution found, check and update optimal solution
            selected_y =  [r for r in 1:length(y) if y[r] == 1]
            if isempty([r for r in 1:length(y) if 0 < y[r] < 1]) && length(selected_y) <= nb_microhub
                @info "Integer 2e Solution Found!"
                optimal_1e_route = findOptimal1eRoute(filtered_1e_routes_pool, filtered_2e_routes_pool, selected_y)
                selected_parking = Set{Int}()
                for y_value in selected_y
                    push!(selected_parking, filtered_2e_routes_pool[y_value].sequence[1])
                end
                current_solution_value = optimal_1e_route.cost
                current_solution = Vector{Route}()
                push!(current_solution, optimal_1e_route)
                for v in selected_y
                    current_solution_value += filtered_2e_routes_pool[v].cost
                    push!(current_solution, filtered_2e_routes_pool[v])
                end
                println("Ongoing solution value = ", current_solution_value)
                println("Ongoing solution 1e route= ", optimal_1e_route.sequence)

                if current_solution_value <= optimal_solution_value
                    optimal_solution_value = current_solution_value  # Fixed variable name
                    optimal_solution = current_solution
                end          
            end             

            branchingDecision = branchingStrategy1(y, filtered_2e_routes_pool, branchingInfo)
            
            if !isnothing(branchingDecision)
                node_stack = inStack(branchingInfo, branchingDecision, node_stack) 
            end
        else
            @info "Current Restricted Linear Master Problem Infeasible"
        end

        println("\nCurrent optimal solution: with objective value = $optimal_solution_value")
        if !isnothing(optimal_solution)
            for route in optimal_solution
                println("   ", route.sequence)
            end
        end
        num_iter += 1
    end

    println("\nOptimal solution obtained by B&P with objective value $optimal_solution_value: ")
    for route in optimal_solution
        if route == optimal_solution[1]
            print("     1e Route ")
        else
            print("     2e Route ")
        end
        println(route.sequence, "   Cost= ",round(route.cost, digits=2))
    end
end


# branchAndPrice(routes_1, routes_2)
branchAndPriceOnlyCustomers(routes_1, routes_2)
println("")

# solveMasterProblem() 
