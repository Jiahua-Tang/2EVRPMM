function solveRMP(route_1)
    feasible_1e_routes = Vector{Route}()
    push!(feasible_1e_routes, route_1)
    feasible_2e_routes = generateAllFeasible2eRoute()

    @info ("Number of 1e routes: ", length(feasible_1e_routes))
    @info ("Number of 2e routes: ", length(feasible_2e_routes))

    routes_originated_p = Vector{Vector{Int}}()
    for s in satellites 
        routes = Vector{Int}()
        for (r,route) in enumerate(feasible_2e_routes)
            if route.sequence[1] == s
                push!(routes, r)
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
    return objective_value(model)
end

function solveMasterProblem()
    feasible_1e_routes, feasible_2e_routes = generateAllRoutes()
    
    @info ("Number of 1e routes: ", length(feasible_1e_routes))
    @info ("Number of 2e routes: ", length(feasible_2e_routes))

    routes_originated_p = Vector{Vector{Int}}()
    for s in satellites 
        routes = Vector{Int}()
        for (r,route) in enumerate(feasible_2e_routes)
            if route.sequence[1] == s
                push!(routes, r)
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
    return objective_value(model)
end

function displayDualValue(π1, π2, π3, π4)
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
    # print("]\nDual price π5= [  ")
    # for x in π5
    #     print(round(x, digits=2),"  ")
    # end
    println("]")
end

function column_generation(filtered_1e_routes, filtered_2e_routes, branchingInfo)
    stopStatus = false
    num_iter = 1
    reduced_cost_2e = zeros(length(satellites)+1)
    reduced_cost_1e = 0
    π1, π2, π3, π4, π5 = 0, 0, 0, 0, 0
    new_columns_generated = Vector{Route}()
    x = nothing
    y = nothing

    while !(stopStatus)
        println("\n======================Iter CG $num_iter======================")

        rlmpResult = solveRestrictedMasterProblem(filtered_1e_routes, filtered_2e_routes, branchingInfo)
        if !isnothing(rlmpResult)
            ##  if RLMP has feasible solution
            π1, π2 = rlmpResult[1], rlmpResult[2]
            π3 = rlmpResult[3]
            π4 = rlmpResult[4]
            x = rlmpResult[5]
            y = rlmpResult[6]
            # displayDualValue(π1, π2, π3, π4, π5)

            status_2e_subproblem_ternimate = true
            println("")
            for s in satellites
                if !(s in branchingInfo.forbidden_parkings)
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

function solve_2e_MILP(pi1, pi2, pi3, pi4, startParking, branchingInfo)
    # println("During 2e pricing problem, branchingDecision\n    ",branchingDecision)

    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, y[A2, A2], Bin)
    @variable(model, 1 <= u[A2] <= length(customers), Int)
    # displayBranchingRule(branchingDecision)

    available_parkings = setdiff(Set(satellites), branchingInfo.forbidden_parkings)
    # display(available_parkings)

    if !isnothing(branchingInfo)
        for combination in branchingInfo.must_include_combinations
            # if startParking != combination[1]
            #     @constraint(model, sum(y[combination[2], i] for i in A2) == 0)
            # end
            @constraint(model, sum(y[combination[1], i] for i in A2) + sum(y[i, combination[1]] for i in A2) + sum(y[combination[2],i] for i in A2) >= 2)
       end

        for combination in branchingInfo.forbidden_combinations
            # if startParking == combination[1]
            #     @constraint(model, sum(y[combination[2], i] for i in A2) == 0)
            # end
            @constraint(model, sum(y[combination[1], i] for i in A2) + sum(y[i, combination[1]] for i in A2) + sum(y[combination[2],i] for i in A2) <= 1)

        end

        for combination in branchingInfo.must_served_together
            combination = collect(combination)
            @constraint(model, sum(y[combination[1],i] for i in A2) 
                            == sum(y[combination[2],i] for i in A2))
        end

        for combination in branchingInfo.forbidden_served_together 
            combination = collect(combination)
            @constraint(model, sum(y[combination[1],i] for i in A2)  
                             + sum(y[combination[2],i] for i in A2) <= 1)
        end

        for parking in branchingInfo.forbidden_parkings 
            @constraint(model, sum(y[i, parking] for i in A2)==0)
            @constraint(model, sum(y[parking, i] for i in A2)==0)
        end
    end
    
    @constraint(model, [i in satellites, j in satellites], y[i,j]==0)
    @constraint(model, flowCustomer[i in customers], sum(y[i,j] for j in A2) == sum(y[j,i] for j in A2))
    @constraint(model, beginParking, sum( y[startParking,i] for i in customers)==1)
    @constraint(model, endParking, sum( y[i,s] for s in satellites for i in customers)==1)
    @constraint(model, vehicleCapacity, sum(y[i,j]*demands[i] for i in A2 for j in A2)<=capacity_2e_vehicle)
    @constraint(model, maxDuration, sum(y[i,j]*arc_cost[i,j] for i in A2 for j in A2)<=maximum_duration_2e_vehicle)
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

function solveRestrictedMasterProblem(routes_1e, routes_2e, branchingInfo)

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

    # println("Number of 2e routes: ", length(routes_2e))
    # println("Number of 1e routes: ", length(routes_1e))
    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, 1>=x[1:length(routes_1e)]>=0)
    @variable(model, 1>=y[1:length(routes_2e)]>=0)

    forbidden_2e_routes = Vector{Int}()
    for (idx,route) in enumerate(routes_2e) 
        for parking in branchingInfo.forbidden_parkings 
            if parking in route.sequence
                push!(forbidden_2e_routes, idx)
            end
        end
    end

    # println("size of forbidden 2e routes is:  $(length(forbidden_2e_routes))")
    if !isempty(branchingInfo.forbidden_parkings)
        @constraint(model, forbidden_parkings, sum(y[r] for r in (forbidden_2e_routes)) == 0)
    end

    if !isempty(branchingInfo.lower_bound_number_2e_routes)
        lower_bound = maximum(branchingInfo.lower_bound_number_2e_routes) 
        @constraint(model, sum(y[r] for (r, route) in enumerate(routes_2e))>=lower_bound)
    end
        if !isempty(branchingInfo.upper_bound_number_2e_routes)
        upper_bound = minimum(branchingInfo.upper_bound_number_2e_routes) 
        @constraint(model, sum(y[r] for (r, route) in enumerate(routes_2e))<=upper_bound)
    end

    # @constraint(model, microhubUB, sum(route.b1[s] * x[r] for (r, route) in enumerate(routes_1e) for s in satellites)<=nb_microhub)
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
        # println("Objective value of master problem is: ",value(objective_value(model)))
        # println("Status of 1e route: ")
        # for (r,route) in enumerate(routes_1e)
        #     if value(x[r]) != 0
        #         print("   x=", round(value(x[r]),digits=2), "   1e Route ", route.sequence, "   Load= ", route.load ,  "   Cost= ", round(route.cost,digits=2), "   Available parking: ")
        #         for (idx, availability) in enumerate(route.b1)
        #             if availability == 1
        #                 print(" ", idx)
        #             end
        #         end
        #         println("")
        #     end 
        # end

        # println("Status of 2e route: ")
        # for (r,route) in enumerate(routes_2e)
        #     if value(y[r]) != 0
        #         println("   y=",round(value(y[r]),digits=2),"   2e Route ", route.sequence, "   Load= ", route.load , "   Cost= ", round(route.cost,digits=2))
        #     end
        # end

        # println("\n   Total number of active satellites: ",sum( route.b1[s] * value(x[r]) for s in satellites for (r,route) in enumerate(routes_1e)))
        # println("   Total number of 2e routes: ", sum(value(y[r]) for r in 1:length(routes_2e)))
        
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
        
        return π1, π2, π3, π4, x_value, y_value, objective_value(model)
    else
        return nothing
    end
end
