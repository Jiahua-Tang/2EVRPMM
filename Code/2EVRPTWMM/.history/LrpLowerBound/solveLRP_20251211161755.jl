include("../BranchAndPrice/columnGeneration.jl")

function calculateTSP1e(selected_parkings)
    # println("Calculate TSP 1e : $selected_parkings\n")
    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, x[A1, A1], Bin)
    @variable(model, y[A1, A1], Bin)
    @variable(model, u[A1], Int)
    for i in A1 
        @constraint(model, x[i,i] == 0)
        @constraint(model, y[i,i] == 0)
    end
    ## Flow conservation at parking for FEV and microhub
    @constraint(model, [i in satellites], sum(x[j,i] for j in A1) == sum(x[i,j] for j in A1))
    # @constraint(model, [i in satellites], sum(x[i,j] for j in A1) <= 1)
    @constraint(model, [i in selected_parkings], sum(x[j,i] for j in A1) == 1)
    @constraint(model, [i in selected_parkings], parking_availability[i] + sum(y[j,i] for j in A1) + sum(y[i,j] for j in A1) == 1)
    ## Flow conservation at depot for FEV and microhub
    @constraint(model, sum(x[1,j] for j in A1) == 1)
    @constraint(model, sum(x[j,1] for j in A1) == 1)
    @constraint(model, [i in A1], y[1,i] ==0)
    @constraint(model, [i in A1], y[i,1] ==0)
    ## Microhub moving rule
    @constraint(model, [i in A1, j in A1], y[i,j] <= x[i,j])
    @constraint(model, [j in satellites], sum(y[i,j] for i in A1) <= 1-parking_availability[j])
    @constraint(model, [i in satellites], sum(y[i,j] for j in A1) <= parking_availability[i])
    ## Subtour elimination
    @constraint(model, [i in satellites, j in satellites], u[i] + 1 <= u[j] + length(A1)*(1-x[i,j]))

    @objective(model, Min, sum(arc_cost[i,j] * x[i,j] for i in A1 for j in A1))

    optimize!(model)

    println("CPLEX solve MILP 1e TSL time = ", round(solve_time(model),digits=3), " seconds")

    # for i in A1 
    #     for j in A1 
    #         if value(x[i,j]) != 0
    #             println("x[$i,$j]=$(round(value(x[i,j])))")
    #         end
    #     end 
    # end
    # for i in A1 
    #     for j in A1 
    #         if value(y[i,j]) != 0
    #             println("- y[$i,$j]=$(round(value(y[i,j])))")
    #         end
    #     end 
    # end
    # for i in satellites
    #     println("u[$i] = ", value(u[i]))
    # end

    return generate1eRoute(transformRoute(x))
end

function calculateLRP2eCG(route_1e::Route)

    result = solve_root_node(route_1e)
    if !isnothing(result)
        return result[1][1].cgLowerBound
    else
        return nothing
    end

end

function calculateLRPLowerBoundByParking()
    # display("$minimum_parkings_required")
    lowerbound_1e_routes = Dict{Route, Float64}()

    for (idx, parking) in enumerate(parking_availability)   
        println("parking availability[$idx] = ", parking)
    end

    parkingCombination = Dict{Route, Float64}()
    for num_parking in minimum_parkings_required:nb_microhub
        for parking_subset in combinations(satellites, num_parking)
            # println("\n================================Parking subset = ", parking_subset,"================================")
            route_1e = calculateTSP1e(parking_subset)
            lowerbound_2e = calculateLRP2eCG(route_1e)
            lowerbound_1e_routes[route_1e] = route_1e.cost + lowerbound_2e
            # @info "$(route_1e.sequence)  Lower bound found: $(round(route_1e.cost, digits=2))  $(round(lowerbound_2e, digits=2))  Total: $(round(route_1e.cost + lowerbound_2e, digits=2))"
        end
    end 
    return lowerbound_1e_routes
    # display(lowerbound_1e_routes)
end

function calculateLRPLowerBoundCG()
    lowerbound_1e_routes = Dict{Route, Float64}()

    for (idx, parking) in enumerate(parking_availability)   
        println("parking availability[$idx] = ", parking)
    end

    parkingCombination = Dict{Route, Float64}()
    for num_parking in minimum_parkings_required:nb_microhub
        for parking_subset in combinations(satellites, num_parking)
            # println("\n================================Parking subset = ", parking_subset,"================================")
            route_1e = calculateTSP1e(parking_subset)

            lowerbound_2e = calculateLRP2eCG(route_1e)
            if !isnothing(lowerbound_2e)
                lowerbound_1e_routes[route_1e] = lowerbound_2e
            end
            # @info "$(route_1e.sequence)  Lower bound found: $(round(route_1e.cost, digits=2))  $(round(lowerbound_2e, digits=2))  Total: $(round(route_1e.cost + lowerbound_2e, digits=2))"
        end
    end 
    return lowerbound_1e_routes

end

function displayLRPLowerBound(lb_lrp_per_route)
    count = 1
    while !isempty(lb_lrp_per_route)
        min_value, min_route = findmin(lb_lrp_per_route)
        println("Lower bound= ", round(min_value, digits=2), "   ", min_route.sequence, "    parkings: $(getServedParking1eRoute(min_route))" )
        # println(count, ". ",routes_1e_complete[min_idx].sequence,"  ",getServedParking1eRoute(routes_1e_complete[min_idx]),"   lower bound= $(round(min_value, digits=2))")
        delete!(lb_lrp_per_route, min_route)
        count += 1
    end
end

function get_sorted_2e_subproblems(theta)
    lrp_subproblems = PriorityQueue()

    for (idx, parking) in enumerate(parking_availability)   
        println("parking availability[$idx] = ", parking)
    end
    println("")

    global routes_1e_complete = Vector{Route}()

    for num_parking in minimum_parkings_required:nb_microhub
        for parking_subset in combinations(satellites, num_parking)
            println("\n================================Parking subset = ", parking_subset,"================================")
            flush(stdout)
            route_1e = calculateTSP1e(parking_subset)
            push!(routes_1e_complete, route_1e)
            # println(route_1e.sequence)
            # println("start solving lp")
            flush(stdout)
            lower_bound_subproblem = route_1e.cost
            # lower_bound_subproblem += solve_location_allocation(parking_subset)
            lower_bound_subproblem += solve_LRP_LP(parking_subset)

            enqueue!(lrp_subproblems, route_1e, lower_bound_subproblem)
        end
    end 

    # for (route, solution) in lrp_subproblems
    #     println(route.sequence, "   $(getServedParking1eRoute(route)),   $(round(solution, digits=2))")
    # end

    # for (route, _) in Iterators.take(lrp_subproblems, theta)
    #     lrp_solution_value = calculate_LRP_LP(getServedParking1eRoute(route))
    #     println(round(lrp_solution_value + route.cost, digits=2))
    # end
    
    return lrp_subproblems

end

function solve_LRP_LP(selected_parkings)
    model = Model(CPLEX.Optimizer)
    set_silent(model)
    # set_optimizer_attribute(model, "CPX_PARAM_TILIM", 60)
    # println("solve lrp lp of subproblem $selected_parkings ")

    @variable(model, x[A2, A2]>=0)
    @variable(model, capacity_2e_vehicle>=f[A2, A2]>=0)

    @objective(model, Min, sum(arc_cost[i,j] * x[i,j] for i in A2, j in A2))

    # self-loop
    @constraint(model, [i in A2], x[i,i] == 0)

    # conservation
    @constraint(model, [i in A2], sum(x[i,j] for j in A2) == sum(x[j,i] for j in A2))
    @constraint(model, [i in satellites], sum(x[i,j] for j in customers)<= nb_vehicle_per_satellite)

    # cov - customer
    @constraint(model, [i in customers], sum(x[i,j] for j in A2) == 1)
    # cov - depot
    @constraint(model, [i in selected_parkings], sum(x[i,j] for j in A2) >= 1)
    @constraint(model, [i in setdiff(satellites, selected_parkings)], sum(x[i,j] for j in A2) == 0)

    # capacity 2e vehicle
    @constraint(model, [i in customers], sum(f[j,i] for j in A2) - sum(f[i,j] for j in A2) == demands[i])
    @constraint(model, [i in A2, j in A2], f[i,j] <= capacity_2e_vehicle * x[i,j])

    optimize!(model)


    println("CPLEX solve LP 2e MDVRP time = ", round(solve_time(model),digits=3), " seconds")

    #region: print lp result
    # for i in A2, j in A2
    #     if value(x[i,j]) != 0 
    #         println("x[$i,$j] = $(round(value(x[i,j]), digits=2))")
    #     end
    # end

    # for i in A2, j in A2
    #     if value(f[i,j]) != 0 
    #         println("f[$i,$j] = $(round(value(f[i,j]), digits=2))")
    #     end
    # end

    # for customer in selected_parkings ∪ customers
    #     if customer in customers
    #         println("t[$(customer-1-length(satellites))] = $(round(value(t[customer]),digits=2))")
    #     else
    #         println("t[$customer] = $(round(value(t[customer]),digits=2))")
    #     end
    # end
    #endregion
    return objective_value(model)
end

function solve_location_allocation(selected_parkings)

    model = Model(CPLEX.Optimizer)
    set_silent(model)

    @variable(model, x[selected_parkings, customers], Bin)

    @objective(model, Min, sum(arc_cost[i,j] * x[i,j] for i in selected_parkings, j in customers))

    @constraint(model, [i in customers], sum(x[j,i] for j in selected_parkings)>=1)

    optimize!(model)

    # for i in selected_parkings, j in customers
    #     if value(x[i,j]) != 0 
    #         println("x[$i,$j] = $(round(value(x[i,j])))")
    #     end
    # end

    return objective_value(model)

end