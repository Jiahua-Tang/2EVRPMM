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
include("../Utiles_copy.jl")

function solveMasterProblem()
    feasible_1e_routes, feasible_2e_routes = generateAllRoutes()
    # a_test, b1_test, b2in_test, b2out_test = initializeParameters(feasible_1e_routes, feasible_2e_routes, customers, satellites, PI)
    routes_originated_p = Vector{Vector{Int}}()
    for s in satellites 
        routes = Vector{Int}()
        for (r,route) in enumerate(feasible_2e_routes)
            if route.sequence[1] == s
                push!(routes,r)
            end
        end
        push!(routes_originated_p, routes)
    end

    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, 1>=x[1:length(feasible_1e_routes)]>=0, Int)
    @variable(model, 1>=y[1:length(feasible_2e_routes)]>=0, Int)

    @constraint(model, sync[s in satellites], sum(route.b2out[s] * y[r] for (r, route) in enumerate(feasible_2e_routes))
    -nb_vehicle_per_satellite*sum(route.b1[s] * x[r] for (r, route) in enumerate(feasible_1e_routes))<=0)
    @constraint(model, custVisit[i in customers], 1 - sum(route.a[i-1-length(satellites)] * y[r] for (r, route) in enumerate(feasible_2e_routes)) <= 0 )
    @constraint(model, number2evfixe[s in satellites], sum(route.b2in[s] * y[r] for (r, route) in enumerate(feasible_2e_routes)) == sum(route.b2out[s] * y[r] for (r, route) in enumerate(feasible_2e_routes)))
    @constraint(model, maxVolumnMM[s in satellites], sum( feasible_2e_routes[r].a[i-1-length(satellites
    )]*demands[i]*y[r] for r in routes_originated_p[s-1] for i in customers) - capacity_microhub <= 0)
    @constraint(model, single1eV, sum(x[r] for (r,_) in enumerate(feasible_1e_routes))==1)

    @objective(model, Min, sum(y[r] * route.cost for (r, route) in enumerate(feasible_2e_routes)) + sum(x[r] * route.cost for (r, route) in enumerate(feasible_1e_routes)))

    optimize!(model)

    println("Objective value of master problem is: ",value(objective_value(model)))

    println("Status of 1e route: ")
    for (i,r) in enumerate(feasible_1e_routes)
        if value(x[i]) != 0
            print("   x=",round(value(x[i]),digits=2),"   1e Route ", r.sequence, "   Load= ", r.load ,  "   Cost= ", round(r.cost,digits=2), "   Available parking:")
            for s in satellites
                if r.b1[s] == 1
                    print(" ",s)
                end
            end
            println("")
        end 
    end

    println("Status of 2e route: ")
    for (i,r) in enumerate(feasible_2e_routes)
        if value(y[i]) != 0
            println("   y=",round(value(y[i]),digits=2),"   2e Route ", r.sequence, "   Load= ", r.load , "   Cost= ", round(r.cost,digits=2))
            # tc += routes_2[r].cost
        end
    end
    println("Status of satellite")
    for s in satellites
        freight = 0
        for (r,route) in enumerate(feasible_2e_routes)
            if route.sequence[1] == s && value(y[r])!=0
                freight += route.load
            end
        end
        if freight != 0
            println("   parking $s stores $freight freight")
        end
    end

end

function displayDualValue(π1, π2, π3, π4, π5)
    print("Dual price π1= [  ")
    for x in π1
        print(round(x, digits=2),"  ")
    end
    print("]\nDual price π2= [  ")
    for x in π2[customers]
        print(round(x, digits=2),"  ")
    end
    print("]\nDual price π3= [  ")
    for x in π3
        print(round(x, digits=2),"  ")
    end
    print("]\nDual price π4= [  ")
    for x in π4
        print(round(x, digits=2),"  ")
    end
    print("]\nDual price π5= [  ")
    for x in π5
        print(round(x, digits=2),"  ")
    end
    println("]")
end

function solve_1e2e_iter_loop(filtered_1e_routes, filtered_2e_routes, branchingInfo)
    stopStatus = false
    num_iter = 1
    reduced_cost_2e = zeros(length(satellites)+1)
    reduced_cost_1e = 0
    π1, π2, π3, π4, π5 = 0, 0, 0, 0, 0
    new_columns_generated = Vector{Route}()
    x = nothing
    y = nothing

    while !(stopStatus)
        println("\n======================Iteration $num_iter======================")

        rlmpResult = solveRestrictedMasterProblem(filtered_1e_routes, filtered_2e_routes)
        if !isnothing(rlmpResult)
            ##  if RLMP has feasible solution
            π1, π2 = rlmpResult[1], rlmpResult[2]
            π3 = rlmpResult[3]
            π4 = rlmpResult[4]
            π5 = rlmpResult[5]
            x = rlmpResult[6]
            y = rlmpResult[7]
            # displayDualValue(π1, π2, π3, π4, π5)

            status_2e_subproblem_ternimate = true
            println("")
            for s in satellites
                subproblem_2e_result, reduced_cost = solve_2e_MILP(π1, π2, π3, π4, s, branchingInfo)
                ## if 2e subproblem from parking s has feasible solution, (generate new 2e route)
                if !isnothing(subproblem_2e_result)
                    # If any 2e route generated, new result stored
                    if reduced_cost_2e[s] != reduced_cost
                        # new route generated has a different reduced cost

                        reduced_cost_2e[s] = reduced_cost
                        # push!(result_2e, subproblem_2e_result)
                        push!(filtered_2e_routes, subproblem_2e_result)
                        push!(new_columns_generated, subproblem_2e_result)
                        # println("   ", "rc= ", round(reduced_cost_2e[s],digits=2), "  new route from parking ", s,": ", subproblem_2e_result.sequence, "    cost=", round(subproblem_2e_result.cost,digits=2))    
                        status_2e_subproblem_ternimate = false
                    end
                end
            end
            # If no new 2e route generated or reduced cost converged, run 1e subproblem
            if status_2e_subproblem_ternimate
                stopStatus = true
                println("Number of new columns generated=  ", length(new_columns_generated))
                return new_columns_generated, filtered_2e_routes, value.(x), value.(y)
                @info "Column Generation terminated"        
            end
        else
            stopStatus = true
            return nothing
        end 

        num_iter += 1
    end   

    # # Display all routes grouped by depart parking
    # for s in satellites
    #     for r in routes_2
    #         if r.sequence[1] == s
    #             println(r.sequence, "   Cost=", round(r.cost, digits=2))
    #         end
    #     end
    # end
end

function solve_1e_MILP(pi1, pi5)
    model = Model(CPLEX.Optimizer)
    set_silent(model)

    @variable(model, 1>=x[A1, A1]>=0, Int)
    @variable(model, u[satellites],Int)

    @constraint(model, [i in A1], sum(x[i,j] for j in A1) == sum(x[j,i] for j in A1))
    @constraint(model, sum(x[1,i] for i in A1)==1)
    @constraint(model, [i in satellites, j in satellites], u[i] + 1 <= u[j] + length(satellites)*(1-x[i,j]))
    @constraint(model, [i in A1, j in A1], parking_availability[i]+parking_availability[j]>=x[i,j])

    @objective(model, Min, pi5 + sum(x[i,j] * arc_cost[i,j] for i in A1 for j in A1)
                        -nb_vehicle_per_satellite*sum( pi1[s] * ((1-parking_availability[s])*(parking_availability[s]+x[s,j]) 
                        +parking_availability[s]*(parking_availability[j]*x[s,j])) for s in satellites for j in A1)
    )
    
    set_optimizer_attribute(model, "CPX_PARAM_TILIM", 120)
    optimize!(model)
    # for i in A1
    #     for j in A1
    #         if value(x[i,j])!=0
    #             println("   x[$i $j]=",value(x[i,j]))
    #         end
    #     end
    # end
    println("Objective value of 1e Subproblem is: ",round(objective_value(model), digits=2))
    
    if value(objective_value(model)) < 0
        new_route = generateRoute(transformRoute(x))
        # display(new_route)
        return new_route, objective_value(model)
    else
        return nothing, objective_value(model)
    end
end

function solve_2e_MILP(pi1, pi2, pi3, pi4, startParking, branchingDecision)
    # println("During 2e pricing problem, branchingDecision\n    ",branchingDecision)

    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, y[A2, A2], Bin)
    @variable(model, 1 <= u[A2] <= length(customers), Int)
    # displayBranchingRule(branchingDecision)
    if !isnothing(branchingDecision)
        for combination in branchingDecision.must_include_combinations
            if startParking != combination[1]
                @constraint(model, sum(y[combination[2], i] for i in A2) == 0)
            end
       end

        for combination in branchingDecision.forbidden_combinations
            if startParking == combination[1]
                @constraint(model, sum( y[combination[2], i] for i in A2) == 0)
            end
        end

        # for combination in branchingDecision.must_served_together
        #     combination = collect(combination)
        #     @constraint(model, sum(y[combination[1],i] for i in A2) 
        #                     == sum(y[combination[2],i] for i in A2))
        # end

        # for combination in branchingDecision.forbidden_served_together 
        #     combination = collect(combination)
        #     @constraint(model, sum(y[combination[1],i] for i in A2) + 
        #                        sum(y[combination[2],i] for i in A2) <= 1)
        # end
    end

    @constraint(model, flowCustomer[i in customers], sum(y[i,j] for j in A2) == sum(y[j,i] for j in A2))
    @constraint(model, beginParking, sum( y[startParking,i] for i in customers)==1)
    @constraint(model, endParking, sum( y[i,s] for s in satellites for i in customers)==1)
    @constraint(model, vehicleCapacity, sum(y[i,j]*demands[i] for i in A2 for j in A2)<=capacity_2e_vehicle)
    @constraint(model, MTZ[i in customers, j in customers], u[i] + 1 <= u[j] + length(customers)*(1-y[i,j]))

    @objective(model, Min, sum(y[i,j] * arc_cost[i,j] for i in A2 for j in A2)
                         + pi1[startParking] * sum(y[startParking,j] for j in A2)
                         - sum(pi2[i] * y[i,j] for i in customers for j in A2)
                         + pi3[startParking] * (sum(y[i, startParking] for i in customers) - sum(y[startParking, i] for i in customers))
                         + pi4[startParking] * sum(y[i,j] * demands[i] for i in customers for j in A2)
    )
    optimize!(model)
    # println("Objective value of 2e Subproblem is: ",round(objective_value(model), digits=2))
    # for i in A2
    #     for j in A2
    #         if value(y[i,j])!=0
    #             println("y[$i $j]=", Int(value(y[i,j])))
    #         end
    #     end
    # end

    status = termination_status(model)
    if status == MOI.OPTIMAL || status == MOI.FEASIBLE_POINT 
        reduced_cost = value(objective_value(model))
        # println("   result of 2e subproblem:  $reduced_cost")
        if reduced_cost < -1e-8
            # println("new route=",transformRoute(y))
            new_route = generate2eRoute(transformRoute(y))
            return new_route, reduced_cost
        else
            return nothing, reduced_cost
        end      
    else
        return nothing, nothing
    end
end

function solveRestrictedMasterProblem(routes_1e, routes_2e)
    ## Relaxed Restricted Master Problem
    routes_originated_p = Vector{Vector{Int}}()
    for s in satellites 
        routes = Vector{Int}()
        for (r,route) in enumerate(routes_2e)
            if route.sequence[1] == s
                push!(routes,r)
            end
        end
        push!(routes_originated_p, routes)
    end

    @info ("Number of 2e routes: ", length(routes_2e))
    @info ("Number of 1e routes: ", length(routes_1e))
    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, 1>=x[1:length(routes_1e)]>=0)
    @variable(model, 1>=y[1:length(routes_2e)]>=0)

    @constraint(model, sync[s in satellites], sum(route.b2out[s] * y[r] for (r, route) in enumerate(routes_2e))
        -nb_vehicle_per_satellite*sum(route.b1[s] * x[r] for (r, route) in enumerate(routes_1e))<=0)
    @constraint(model, custVisit[i in customers], 1 - sum(route.a[i-1-length(satellites)] * y[r] for (r,route) in enumerate(routes_2e)) <= 0 )
    @constraint(model, number2evfixe[s in satellites], sum(route.b2in[s] * y[r] for (r, route) in enumerate(routes_2e)) == sum(route.b2out[s] * y[r] for (r, route) in enumerate(routes_2e)))
    @constraint(model, maxVolumnMM[s in satellites], sum( routes_2e[r].a[i-1-length(satellites
    )]*demands[i]*y[r] for r in routes_originated_p[s-1] for i in customers) - capacity_microhub <= 0)
    @constraint(model, single1eV, sum(x[r] for (r,_) in enumerate(routes_1e))>=1)
    @constraint(model, maxMMnumber, sum(x[r]*sum(route.b1) for (r,route) in enumerate(routes_1e)) <= nb_microhub)

    @objective(model, Min, sum(y[r] * route.cost for (r,route) in enumerate(routes_2e)) + 
                        sum(x[r] * route.cost for (r,route) in enumerate(routes_1e)))
    
    optimize!(model)
  
    status = termination_status(model)
    if status == MOI.OPTIMAL || status == MOI.FEASIBLE_POINT
        println("Objective value of master problem is: ",value(objective_value(model)))
        # println("Status of 1e route: ")
        # for (r,route) in enumerate(routes_1)
        #     if value(x[r]) != 0
        #         print("   x=",round(value(x[r]),digits=2),"   1e Route ", route.sequence, "   Load= ", route.load ,  "   Cost= ", round(route.cost,digits=2), "   Available parking: ")
        #         for (idx, availability) in enumerate(route.b1)
        #             if availability == 1
        #                 print(" ", idx)
        #             end
        #         end
        #         println("")
        #     end 
        # end
        println("Status of 2e route: ")
        for (r,route) in enumerate(routes_2e)
            if value(y[r]) != 0
                println("   y=",round(value(y[r]),digits=2),"   2e Route ", route.sequence, "   Load= ", route.load , "   Cost= ", round(route.cost,digits=2))
            end
        end
        # println("Status of satellite")
        # for s in satellites
        #     freight = 0
        #     for (r,route) in enumerate(routes_2)
        #         if route.sequence[1] == s && value(y[r])!=0
        #             freight += route.load
        #         end
        #     end
        #     if freight != 0
        #         println("   parking $s stores $freight freight")
        #     end
        # end
        # println("   Total 2nd route cost = $tc")

        π1 = collect(dual.(sync))
        π2 = collect(dual.(custVisit))
        π3 = collect(dual.(number2evfixe))
        π4 = collect(dual.(maxVolumnMM))
        π5 = collect(dual.(single1eV))
        
        π1 = abs.(vcat(0,π1))
        π2 = abs.(vcat(zeros(1+length(satellites)),π2))
        π3 = abs.(vcat(0,π3))
        π4 = abs.(vcat(0,π4))

        x_value = []
        y_value = []

        for (r, _) in enumerate(routes_1e) 
            push!(x_value, value(x[r]))
        end
        for (r,_) in enumerate(routes_2e)
            push!(y_value, value(y[r]))
        end
        
        return π1, π2, π3, π4, π5, x_value, y_value
    else
        return nothing
    end
end

function node_most_used(route_to_branch, routes)
    node_to_branch = route_to_branch[2]
    apperance_max = 1
    
    for node in route_to_branch 
        # println("node_to_branch: $node_to_branch,   apperance_max: $apperance_max")
        flag = 1
        for r in routes 
            if node in r.sequence[2:end-1] && r != route_to_branch
                flag += 1
            end
        end
        if flag > apperance_max
            apperance_max = flag
            node_to_branch = node
        end
    end
    return node_to_branch
end

# ============================================================================== #

const data = initializeData()

const coor = data[1]
const nb_parking = data[2]
const nb_microhub = data[3]
const parking_availability = data[4]
const demands = data[5]
const arc_cost = data[6]

for (idx, p) in enumerate(parking_availability) 
    println("parking_availability[$idx] = ",Int(p))
end

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
function generate2eInitialRoutes()
    result = Vector{Route}()
    for cust in customers
        for parking in satellites
            if arc_cost[parking, cust] <= maximum_duration_2e_vehicle/2
                push!(result, generate2eRoute([parking, cust, parking]))
            end
        end
    end
    return result
end
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


# x_vals = [coord[1] for coord in coor]
# y_vals = [coord[2] for coord in coor]
# gr()
# plt = scatter(x_vals[2+nb_parking:end], y_vals[2+nb_parking:end], markershape=:circle, label = "Customers")
# scatter!(plt, x_vals[2:1+nb_parking], y_vals[2:1+nb_parking], markershape=:utriangle, label = "Parkings")
# scatter!(plt, x_vals[1:1], y_vals[1:1], markershape=:circle, label = "depot")
# node_labels = [string("N.",i) for i in 1:length(coor)]
# for i in 1:length(coor)
#     annotate!(x_vals[i], y_vals[i]+0.3, text(node_labels[i],6))
# end
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

function displayBranchingRule(branchingInfo::BranchingInfo)
    print("   Must include combination:  ")
    for value in branchingInfo.must_include_combinations 
        print(value, "  ")
    end
    print("\n   Forbidden combination:   ")
    for value in branchingInfo.forbidden_combinations 
        print(value, "  ")
    end
    println("depth:  ", branchingInfo.depth)
    # print("\n   Customers must be served together:   ")
    # for value in branchingInfo.must_served_together
    #     print(value, "  ")
    # end
    # print("\n   Customers forbidden served together:   ")
    # for value in branchingInfo.forbidden_served_together 
    #     print(value, "  ")
    # end
    println("\n")
end

function most_costive_points(optimal_1e_route, served_mm::Vector{Int})
    # println("TESTTEST ", optimal_1e_route)
    full_route = optimal_1e_route.sequence
    point_costs = Dict{Int, Float64}()

    # Map each node to its index in full route
    full_index = Dict(n => i for (i, n) in enumerate(full_route))

    # Loop through internal nodes (excluding depots)
    for i in 2:length(served_mm)-1
        node = served_mm[i]
        idx_in_full = full_index[node]

        # Find previous served node in full route
        prev_node = nothing
        for j = idx_in_full-1:-1:1
            if full_route[j] in served_mm
                prev_node = full_route[j]
                break
            end
        end

        # Find next served node in full route
        next_node = nothing
        for j = idx_in_full+1:length(full_route)
            if full_route[j] in served_mm
                next_node = full_route[j]
                break
            end
        end

        # If both neighbors are found, sum arc costs in full route
        if prev_node !== nothing && next_node !== nothing
            idx_start = full_index[prev_node]
            idx_end = full_index[next_node]
            cost = 0.0
            for k in idx_start:idx_end-1
                from = full_route[k]
                to = full_route[k+1]
                cost += arc_cost[from, to]
            end
            point_costs[node] = cost
        end
    end

    # Sort the point_costs dict by descending cost
    sorted_nodes = sort(collect(point_costs), by = x -> -x[2])

    # Separate keys and values if needed
    nodes_sorted = [x[1] for x in sorted_nodes]
    # println("TESTTEST ", nodes_sorted)
    return nodes_sorted
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

# function log_branch_node(io, branchingInfo::BranchingInfo, decision, status, value)
#     depth = branchingInfo.depth
#     indent = join(["│   " for _ in 1:depth], "")
#     prefix = depth == 0 ? "└── Root" : indent * "├──"
#     println(io, prefix * " Branch: $(decision === nothing ? "None" : decision)  [$status]" *
#         (value !== nothing ? ", Value: $(round(value, digits=2))" : ""))
# end

function branchAndPrice(routes_1e_pool, routes_2e_pool)
    root_branch = BranchingInfo(Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), 0)
    node_stack = [root_branch]
    
    optimal_solution_value = Inf
    optimal_solution = nothing

    num_iter = 1
    while !isempty(node_stack) && num_iter <= 20
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
            x = rlmpResult[3]
            y = rlmpResult[4]

            fractional_y = [r for r in 1:length(filtered_2e_routes_pool) if 0 < y[r] < 1]
            selected_y = [r for r in 1:length(filtered_2e_routes_pool) if y[r] == 1]

            if isempty(fractional_y)
                @info "Integer 2e Solution Found!"
                optimal_1e_route = findOptimal1eRoute(filtered_1e_routes_pool, filtered_2e_routes_pool, selected_y)
                selected_parking = Set{Int}()
                for y_value in selected_y
                    push!(selected_parking, filtered_2e_routes_pool[y_value].sequence[1])
                end
                # println("selected_parking= $selected_parking")
                if length(selected_parking) <= nb_microhub
                    current_solution_value = optimal_1e_route.cost
                    current_solution = Vector{Route}()
                    push!(current_solution, optimal_1e_route)
                    for v in selected_y
                        current_solution_value += filtered_2e_routes_pool[v].cost
                        push!(current_solution, filtered_2e_routes_pool[v])
                    end
                    println("Ongoing solution value = ", current_solution_value)
                    println("Ongoing solution = ", optimal_1e_route.sequence)

                    if current_solution_value <= optimal_solution_value
                        optimal_solution_value = current_solution_value  # Fixed variable name
                        optimal_solution = current_solution
                    end                    
                end

                branchDecisionFound = true
                branchingDecision = nothing
                sorted_selected_routes = sort(
                    [filtered_2e_routes_pool[y] for y in selected_y],
                    by = r -> r.cost
                )
                while branchDecisionFound
                    for route in sorted_selected_routes
                        branch_point_1 = rand([route.sequence[1], route.sequence[end]])
                        branch_point_2 = rand(route.sequence[2:end-1])
                        branchingDecision = (branch_point_1, branch_point_2)
                        existance_must = branchingDecision in branchingInfo.must_include_combinations
                        existance_forbidden = branchingDecision in branchingInfo.forbidden_combinations
                        # println(route.sequence, "    ", branchingDecision, "   ", existance_must, "   ",existance_forbidden)
                        if !existance_must && !existance_forbidden
                            branchDecisionFound = false
                            break
                        end
                        if route == sorted_selected_routes[end]
                            branchingDecision = nothing
                            branchDecisionFound = false   
                            @info "Leaf node"                 
                        end
                    end
                end

                if !isnothing(branchingDecision)
                    println("Branching Decision: parking-customer: ", branchingDecision)
                    left_branch = deepcopy(branchingInfo)
                    right_branch = deepcopy(branchingInfo)
                    push!(left_branch.must_include_combinations, branchingDecision)
                    push!(right_branch.forbidden_combinations, branchingDecision)
                    left_branch.depth  += 1
                    right_branch.depth += 1
                    push!(node_stack, right_branch)
                    push!(node_stack, left_branch)
                end

            else
                @info "Fractional 2e solution found"
                sorted_fractional_y = sort(fractional_y, by = r -> y[r] * (1 - y[r]), rev = true)
                # println("sorted fractional y = $sorted_fractional_y")

                branchingDecisionFound = true
                while branchingDecisionFound
                    # println("TEST POINT  $branchingDecisionFound")
                    for y_value in sorted_fractional_y
                        route_to_branch = filtered_2e_routes_pool[y_value].sequence
                        branchingDecision = (rand([route_to_branch[1],route_to_branch[end]]), rand(route_to_branch[2:end-1]))
                        existance_must = branchingDecision in branchingInfo.must_include_combinations
                        existance_forbidden = branchingDecision in branchingInfo.forbidden_combinations
                        # println(route_to_branch, "    ", branchingDecision, "   ", existance_must, "   ",existance_forbidden)
                        if !existance_must && !existance_forbidden
                            branchingDecisionFound = false
                            break
                        end
                        if y_value == sorted_fractional_y[end]
                            branchingDecision = nothing
                            branchingDecisionFound = false   
                            @info "Leaf node"                 
                        end
                    end
                end
                if !isnothing(branchingDecision)
                    println("Combination of parking - customer need to branch on = ", branchingDecision)
                    left_branch = deepcopy(branchingInfo)
                    right_branch = deepcopy(branchingInfo)
                    push!(left_branch.must_include_combinations, branchingDecision)
                    push!(right_branch.forbidden_combinations, branchingDecision)
                    left_branch.depth  += 1
                    right_branch.depth += 1
                    push!(node_stack, right_branch)
                    push!(node_stack, left_branch)                       
                end
             
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

branchAndPrice(routes_1, routes_2)

println("")

# solveMasterProblem() 
