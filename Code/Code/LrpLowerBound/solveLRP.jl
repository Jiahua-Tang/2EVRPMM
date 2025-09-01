# include("labelling.jl")

function calculateLRPLowerBound(routes_1e_complete)
    lb_per_route = Dict{Int, Float64}()

    for (r, route) in enumerate(routes_1e_complete)
        available_parkings = getServedParking1eRoute(route)
        empty_parkings = setdiff!(collect(satellites), available_parkings)
        available_points = sort(collect(union(available_parkings, customers)))
    
        model = Model(CPLEX.Optimizer)
        set_silent(model)

        @variable(model, x[A2, A2], Bin)
        @variable(model, u[customers]>=0,Int)
    
        @constraint(model, [i in customers], sum(x[i,j] for j in A2)==1)
        @constraint(model, [p in empty_parkings], sum(x[p,i] for i in A2) == 0)
        @constraint(model, [p in available_parkings], sum(x[p,j] for j in A2)>=1)
        @constraint(model, [i in available_parkings, j in available_parkings], x[i,j]==0)
        @constraint(model, [i in available_points], sum(x[i,j] for j in A2) == sum(x[j,i] for j in A2))
        @constraint(model, [i in customers, j in customers], u[i] + 1 <= u[j] + length(customers)*(1-x[i,j]))
        
        @objective(model, Min, route.cost + sum(arc_cost[i,j]*x[i,j] for i in A2 for j in A2))
    
        optimize!(model)
    
        status = termination_status(model)
        if status == MOI.OPTIMAL || status == MOI.FEASIBLE_POINT 
            lb_per_route[r] = objective_value(model)
        end
    end

    return lb_per_route
end

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


function solveLRP_RLMP(selected_parkings, routes_pool)
    # println("Solve LRP 2e RLMP  ", selected_parkings)
    model = Model(CPLEX.Optimizer)
    set_silent(model)

    @variable(model, 0 <= x[1:length(routes_pool)] <= 1)

    @constraint(model, activeParking[p in selected_parkings], sum(route.b2out[p] * x[r] for (r, route) in enumerate(routes_pool)) >= 1)
    # @constraint(model, forbiddenParking[p in setdiff(satellites, selected_parkings)], sum(route.b2out[p] * x[r] for (r, route) in enumerate(routes_2e)) == 0)
    @constraint(model, cov[c in customers], sum(route.a[c-length(A1)] * x[r] for (r, route) in enumerate(routes_pool)) >= 1)
    @constraint(model, balance[p in selected_parkings], sum(route.b2out[p] * x[r] for (r, route) in enumerate(routes_pool)) == sum(route.b2in[p] * x[r] for (r, route) in enumerate(routes_pool)))
    
    @objective(model, Min, sum(route.cost * x[r] for (r, route) in enumerate(routes_pool)))
    
    optimize!(model)    

    # for (r, route) in enumerate(routes_pool)
    #     if round(value(x[r])) != 0
    #         println(route.sequence, " cost = ", round(route.cost, digits=2), " x[$r] = ", round(value(x[r]), digits=2))
    #     end 
    # end

    π1 = zeros(length(A1))
    π3 = zeros(length(A1))
    i = 1
    for node in selected_parkings
        π1[node] = abs.([dual(activeParking[i]) for i in keys(activeParking)])[i]
        π3[node] = abs.([dual(balance[i]) for i in keys(balance)])[i]
        i += 1
    end
    π2 = abs.(vcat(zeros(length(A1)), [dual(cov[i]) for i in keys(cov)]))

    println("\nπ1 = ", round.(π1, digits=2))
    println("π2 = ", round.(π2[customers], digits=2))
    println("π3 = ", round.(π3, digits=2))
    println("Objective value = ", round(objective_value(model), digits=2))
    return π1, π2, π3, objective_value(model)
end

function filter2eRoute(selected_parkings)
    result = Vector{Route}()
    forbidden_parking = setdiff!(collect(satellites), selected_parkings)
    for route in routes_2e 
        if !(route.sequence[1] in forbidden_parking || route.sequence[end] in forbidden_parking)
            push!(result, route)
        end
    end
    return result
end

function calculateLRP2eCG(selected_parkings)
    println("Calculate LRP 2e Column Generation")
    global routes_2e = generate2eInitialRoutes()
    num_iter_cg = 1
    new_columns_generated = []
    lower_bound = Inf
    routes_pool = filter2eRoute(selected_parkings)

    while num_iter_cg < 5
        π1, π2, π3, cgLB = solveLRP_RLMP(selected_parkings, routes_pool)
        lower_bound = min(lower_bound, cgLB)
        result = solveLRP_SP_Labelling(π1, π2, π3, selected_parkings)
        if isnothing(result) || isempty(result)
            println("No more routes can be generated")
            break
        end
        if (length(result) == length(new_columns_generated) && result[1].visitedNodes == new_columns_generated[1].visitedNodes)
            println("Column Generation converged")
            break           
        end

        new_columns_generated = result
        for route in result
            global routes_2e
            push!(routes_2e, generate2eRoute(route.visitedNodes))
            push!(routes_pool, generate2eRoute(route.visitedNodes))
        end
        for route in routes_pool 
            println("Route: ", route.sequence, " cost = ", round(route.cost, digits=2))
        end
        num_iter_cg += 1
    end
    return lower_bound
end

function calculateLRP2eMILP(selected_parkings)
    # println("Calculate LRP 2e")
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
            lowerbound_2e = calculateLRP2eMILP(parking_subset)
            lowerbound_1e_routes[route_1e] = route_1e.cost + lowerbound_2e
            # @info "$(route_1e.sequence)  Lower bound found: $(round(route_1e.cost, digits=2))  $(round(lowerbound_2e, digits=2))  Total: $(round(route_1e.cost + lowerbound_2e, digits=2))"
        end
    end 
    return lowerbound_1e_routes
    # display(lowerbound_1e_routes)
end

function displayLRPLowerBound(lb_lrp_per_route)
    count = 1
    while !isempty(lb_lrp_per_route)
        min_value, min_route = findmin(lb_lrp_per_route)
        println("Lower bound= ", round(min_value, digits=2), "   ", min_route.sequence )
        # println(count, ". ",routes_1e_complete[min_idx].sequence,"  ",getServedParking1eRoute(routes_1e_complete[min_idx]),"   lower bound= $(round(min_value, digits=2))")
        delete!(lb_lrp_per_route, min_route)
        count += 1
    end
end
