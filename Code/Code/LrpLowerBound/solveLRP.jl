include("../BranchAndPrice/columnGeneration.jl")

function calculateTSP1e(selected_parkings)
    # println("Calculate TSP 1e\n")
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
    @constraint(model, [i in satellites], sum(x[i,j] for j in A1) <= 1)
    @constraint(model, [i in selected_parkings], sum(x[j,i] for j in A1) == 1)
    @constraint(model, [i in selected_parkings], parking_availability[i] + sum(y[j,i] for j in A1) + sum(y[i,j] for j in A1) ==1 )
    ## Flow conservation at depot for FEV and microhub
    @constraint(model, sum(x[1,j] for j in A1) == 1)
    @constraint(model, sum(x[j,1] for j in A1) == 1)
    @constraint(model, [i in A1], y[1,i] ==0)
    @constraint(model, [i in A1], y[i,1] ==0)
    ## Microhub moving rule
    @constraint(model, [i in A1, j in A1], y[i,j] <= x[i,j])
    @constraint(model, [i in satellites], sum(y[i,j] for j in A1) <= parking_availability[i])
    @constraint(model, [j in satellites], sum(y[i,j] for i in A1) <= 1-parking_availability[j])
    ## Subtour elimination
    @constraint(model, [i in satellites, j in satellites], u[i] + 1 <= u[j] + length(A1)*(1-x[i,j]))

    @objective(model, Min, sum(arc_cost[i,j] * x[i,j] for i in A1 for j in A1))

    optimize!(model)

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

function calculateLRP2eMILP(route_1e::Route)

    result = solve_root_node(route_1e)
    if !isnothing(result)
        return result[1][1].cgLowerBound
    else
        return nothing
    end

end

function calculateLRP2eSimplfied(route_1e::Route)
    selected_parkings = getServedParking1eRoute(route_1e)

    ## Solve a multi depot VRP
    empty_parkings = setdiff!(collect(satellites), selected_parkings)
    available_points = sort(collect(union(selected_parkings, customers)))

    model = Model(CPLEX.Optimizer)
    set_silent(model)

    # set_optimizer_attribute(model, "CPX_PARAM_TILIM", 60)

    @variable(model, 1 >= z[A2, A2] >= 0)
    @variable(model, u[customers]>=0)

    @constraint(model, [i in customers], sum(z[i,j] for j in A2)==1)
    @constraint(model, [p in empty_parkings], sum(z[p,i] for i in A2) == 0)
    @constraint(model, [p in selected_parkings], sum(z[p,j] for j in A2)>=1)
    @constraint(model, [i in selected_parkings, j in selected_parkings], z[i,j]==0)
    @constraint(model, [i in available_points], sum(z[i,j] for j in A2) == sum(z[j,i] for j in A2))
    @constraint(model, [i in customers, j in customers], u[i] + 1 <= u[j] + length(customers)*(1-z[i,j]))
    # @constraint(model, )

    @objective(model, Min, sum(arc_cost[i,j] * z[i,j] for i in A2 for j in A2))

    optimize!(model)
    # println("Objective value = ", objective_value(model))

    status = termination_status(model)
    # println("Termination status = ", status)
    if status == MOI.OPTIMAL || primal_status(model) == MOI.FEASIBLE_POINT
        # for i in A2
        #     for j in A2 
        #         value(z[i,j]) != 0 && println("z[$i,$j]=$(value(z[i,j]))")
        #     end
        # end
        return objective_value(model)
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
            # lowerbound_2e = calculateLRP2eMILP(route_1e)
            lowerbound_2e = calculateLRP2eSimplfied(route_1e)
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

            lowerbound_2e = calculateLRP2eMILP(route_1e)
            if !isnothing(lowerbound_2e)
                lowerbound_1e_routes[route_1e] = route_1e.cost + lowerbound_2e
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
