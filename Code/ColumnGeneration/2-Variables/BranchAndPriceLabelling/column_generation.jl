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
    π1, π2, π3, π4 = 0, 0, 0, 0
    new_columns_generated = Dict{Int, Vector{Label}}()
    for s in satellites 
        new_columns_generated[s] = []
    end
    x = nothing
    y = nothing

    while !(stopStatus) # && num_iter < 71
        # println("\n======================Iter CG $num_iter======================")

        rlmpResult = solveRestrictedMasterProblem(filtered_1e_routes, filtered_2e_routes, branchingInfo)
        if !isnothing(rlmpResult)
            ##  if RLMP has feasible solution
            π1, π2 = rlmpResult[1], rlmpResult[2]
            π3, π4 = rlmpResult[3], rlmpResult[4]
            x = rlmpResult[5]
            y = rlmpResult[6]
            # for (_, y_value) in enumerate([r for r in 1:length(y) if 0 < y[r]]) 
            #     println("   $(filtered_2e_routes[y_value].sequence)  $(round(y[y_value],digits=2))")
            # end
            # displayDualValue(π1, π2, π3, π4)

            status_2e_subproblem_ternimate = true

            ## Solve subproblem by each parking
            for s in satellites
                if !(s in branchingInfo.forbidden_parkings)
                    execution_time = @elapsed begin
                        subproblem_2e_result = solve_2e_labelling(π1, π2, π3, π4, s, branchingInfo)
                    end
                    global execution_time_subproblem += execution_time
                    if length(subproblem_2e_result) != 0
                        # @info "Number of new routes generated of subproblem parking $s: $(length(subproblem_2e_result))"
                        if !(length(subproblem_2e_result) == length(new_columns_generated[s]) && subproblem_2e_result[1].visitedNodes == new_columns_generated[s][1].visitedNodes)
                            new_columns_generated[s] = subproblem_2e_result
                            for label in subproblem_2e_result 
                                global routes_2e
                                push!(routes_2e, generate2eRoute(label.visitedNodes))
                                push!(filtered_2e_routes, generate2eRoute(label.visitedNodes))
                            end
                            status_2e_subproblem_ternimate = false
                        else
                            # println("Subproblem for parking $s converge (not 0)")
                        end
                    else
                        # println("Subproblem for parking $s converge")
                    end
                end
            end
            ## All subproblems yiled no more negative reduced cost
            if status_2e_subproblem_ternimate
                stopStatus = true
                return nothing , filtered_2e_routes, value.(x), value.(y), rlmpResult[7]
            end
        else
            # @info "RLMP infeasible"
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

mutable struct Label
    origin_node::Int
    current_node::Int
    reduced_cost::Float64
    accumulated_capacity::Int
    accumulated_duration::Float64
    visitedNodes::Vector{Int}
end

function displayLabel(label::Label)
    println("- Origin parking: $(label.origin_node)")
    println("  Current node: $(label.current_node)")
    println("  Reduced cost: $(round(label.reduced_cost, digits=2))")
    println("  Accumulated capacity: $(label.accumulated_capacity)")
    println("  Accumulated duration: $(round(label.accumulated_duration, digits=2))")
    println("  Visit sequence: $(label.visitedNodes)\n")
end

function printLabel(label::Label)
    @printf("  Label(%d,  %2d,  RC: %7.2f,  C: %2d,  D:%7.2f,  %s)\n",
    label.origin_node,
    label.current_node,
    label.reduced_cost,
    label.accumulated_capacity,
    label.accumulated_duration,
    label.visitedNodes
    )
end

function extendLabel(pi1, pi2, pi3, pi4, label::Label, next_node::Int, branchingInfo::BranchingInfo)
    # displayLabel(label)
    # Update reduced cost
    reduced_cost = label.reduced_cost + arc_cost[label.current_node, next_node]
    if label.current_node in satellites
        reduced_cost += pi1[label.current_node] - pi3[label.current_node]
    end
    if next_node in customers
        reduced_cost = reduced_cost - pi2[next_node] + pi4[label.origin_node] * demands[next_node]
    end
    if next_node in satellites
        reduced_cost += pi3[next_node]       
    end
    # Update accumulated capacity
    accumulated_capacity = label.accumulated_capacity + demands[next_node]
    if accumulated_capacity > capacity_2e_vehicle
        return nothing
    end
    # Update accumulated duration
    accumulated_duration = label.accumulated_duration + arc_cost[label.current_node, next_node]
    if accumulated_duration > maximum_duration_2e_vehicle
        return nothing
    end
    visitedNodes = push!(deepcopy(label.visitedNodes), next_node)
    new_label = Label(label.origin_node, next_node, reduced_cost, accumulated_capacity, accumulated_duration, visitedNodes)
    # displayLabel(new_label)
    return new_label
end

function dominanceRule(label1, label2)
    rcBool = label1.reduced_cost <= label2.reduced_cost
    capBool = label1.accumulated_capacity <= label2.accumulated_capacity
    durationBool = label1.accumulated_duration <= label2.accumulated_duration
    ineBool = label1.reduced_cost == label2.reduced_cost && label1.accumulated_capacity == label2.accumulated_capacity && label1.accumulated_duration == label2.accumulated_duration
    if rcBool && capBool && durationBool && !ineBool
        # @info "Dominance relation found: "
        # printLabel(label1)
        # println("dominates")
        # printLabel(label2)
        return label2
    else
        return nothing
    end
end

function dominanceCheckSingle(l1, l2)
    result = dominanceRule(l1, l2)
    if !isnothing(result)
        return 2
    else
        result = dominanceRule(l2, l1)
        if !isnothing(result)
            return 1
        else
            return nothing
        end
    end
end

function dominanceCheck(unprocessedLabelsList, processedLabelsList)
    unprocessedLabels = deepcopy(unprocessedLabelsList)
    processedLabels = deepcopy(processedLabelsList)
    idx_dominated = []
    for (idx1, label1) in enumerate(unprocessedLabels) 
        for (idx2, label2) in enumerate(unprocessedLabels)
            if idx2 > idx1
                result = dominanceCheckSingle(label1, label2)
                if !isnothing(result)
                    result == 1 && push!(idx_dominated, idx1)
                    result == 2 && push!(idx_dominated, idx2)
                end
            end
        end

        for (idx3, label3) in enumerate(processedLabels) 
            result = dominanceCheckSingle(label1, label3)
            if !isnothing(result)
                result == 1 && push!(idx_dominated, idx1)
                result == 2 && deleteat!(processedLabels, idx3)
            end
        end
    end
    deleteat!(unprocessedLabels, unique(sort(idx_dominated)))
    return unprocessedLabels, processedLabels
end

function solve_2e_labelling(pi1, pi2, pi3, pi4, startParking, branchingInfo)

    ## prep
    active_nodes = []
    for parking in satellites 
        if !(parking in branchingInfo.forbidden_parkings)
            push!(active_nodes, parking)
        end
    end
    active_nodes = vcat(active_nodes, customers)

    ## Line 1
    unprocessedLabels = Dict{Int, Vector{Label}}()
    processedLabels = Dict{Int, Vector{Label}}()
    depotLabels = Dict{Int, Vector{Label}}()

    for node in active_nodes 
        unprocessedLabels[node] = Vector{Label}()
        processedLabels[node] = Vector{Label}()
        # if node in satellites
            depotLabels[node] = Vector{Label}()
        # end
    end

    initial_label = Label(startParking, startParking, 0, 0, 0, [startParking])

    ## To deal with the branching rule in this node:
    ## must parking-customer combination: if start parking is not the parking, delete the customer
    ## forbidden parking-customer combination: if start parking is the parking, delete the customer

    active_customers = collect(deepcopy(customers))
    if !isempty(branchingInfo.must_include_combinations)
        for ele in branchingInfo.must_include_combinations 
            if ele[1] != startParking
                if ele[2] in active_customers
                    deleteat!(active_customers, findfirst(==(ele[2]), active_customers))
                end
            end
        end
    end

    if !isempty(branchingInfo.forbidden_combinations)
        for ele in branchingInfo.forbidden_combinations
            if ele[1] == startParking
                if ele[2] in active_customers
                    deleteat!(active_customers, findfirst(==(ele[2]), active_customers))
                end
            end
        end
    end
    
    # @info "Display active customers for sp parking $startParking : $(active_customers)"
    # First propagation from origin node    
    for cust in active_customers
        result = extendLabel(pi1, pi2, pi3, pi4, initial_label, cust, branchingInfo)
        !isnothing(result) && push!(unprocessedLabels[cust], result)
    end

    ## Line 2
    num_iter_labelling = 1
    # num_iter_labelling < 2 &&
    while !isempty(collect(Iterators.flatten(values(unprocessedLabels)))) 
        # println("\n===================Iter $num_iter_labelling===================")
        # println("Display Unprocessed Labels")
        # for node in active_nodes
        #     if !isempty(unprocessedLabels[node])
        #         print("-")
        #     end
        #     for (idx, ele) in enumerate(unprocessedLabels[node])
        #         if idx > 1
        #             print(" ")
        #         else
        #             print("")
        #         end
        #         printLabel(ele)
        #     end
        # end        
        # println("Display Depot Labels")
        # for node in active_nodes
        #     if !isempty(depotLabels[node])
        #         print("-")
        #     end
        #     for (idx, ele) in enumerate(depotLabels[node])
        #         if idx > 1
        #             print(" ")
        #         else
        #             print("")
        #         end
        #         printLabel(ele)
        #     end
        # end
        # println("Display Processed Labels")
        # for node in active_nodes
        #     if !isempty(processedLabels[node])
        #         print("-")
        #     end
        #     for (idx, ele) in enumerate(processedLabels[node])
        #         if idx > 1
        #             print(" ")
        #         else
        #             print("")
        #         end
        #         printLabel(ele)
        #     end
        # end
        ## Line 3
        all_labels = collect(Iterators.flatten(values(unprocessedLabels)))
        min_label = all_labels[findmin(l -> l.reduced_cost, all_labels)[2]]
        # @info "Selected label:"
        # displayLabel(min_label)
        min_idx = findfirst(==(min_label), unprocessedLabels[min_label.visitedNodes[end]])
        deleteat!(unprocessedLabels[min_label.visitedNodes[end]], min_idx)
        ## Line 9
        push!(processedLabels[min_label.visitedNodes[end]], min_label)
        ## Line 4
        # @info "Propagate to new node"
        ## Line 5
        for node in setdiff(active_nodes, min_label.current_node)
            new_label = extendLabel(pi1, pi2, pi3, pi4, min_label, node, branchingInfo)
            ## Line 6
            if !isnothing(new_label)
                ## Line 7
                node in satellites && push!(depotLabels[node], new_label)
                node in active_customers && push!(unprocessedLabels[node], new_label)

                ## Line 8
                if node in active_customers
                    unprocessedLabels[node], processedLabels[node] = dominanceCheck(unprocessedLabels[node], processedLabels[node])
                end
            end
        end
        num_iter_labelling += 1
    end

    result = []
    for node in intersect(satellites, active_nodes)
        for label in depotLabels[node]
            # if node == 5
            #     printLabel(label)
            #     println(label.visitedNodes, "   ", unique(label.))
            # end
            if label.reduced_cost < -1e-8 && length(label.visitedNodes[2:end-1]) == length(unique(label.visitedNodes[2:end-1]))
                valide = true
                for combination in branchingInfo.must_served_together
                    if Int(combination[1] in label.visitedNodes) + Int(combination[2] in label.visitedNodes) == 1
                        valide = false
                        break
                    end
                end

                for combination in branchingInfo.forbidden_served_together 
                    if Int(combination[1] in label.visitedNodes) + Int(combination[2] in label.visitedNodes) == 2
                        valide = false
                        break
                    end
                end
                
                valide && push!(result, label)
            end
        end
        # for label in result
        #     printLabel(label)
        #     # if label.visitedNodes == [5,17,20,5] || label.visitedNodes == reverse([5,17,20,5])
        #     #     println("[5,17,20,5]")
        #     # end
        # end
    end

    return result

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
            if startParking != combination[1]
                @constraint(model, sum(y[combination[2], i] for i in A2) == 0)
            end
            # @constraint(model, sum(y[combination[1], i] for i in A2) + sum(y[i, combination[1]] for i in A2) + sum(y[combination[2],i] for i in A2) >= 2)
       end

        for combination in branchingInfo.forbidden_combinations
            if startParking == combination[1]
                @constraint(model, sum(y[combination[2], i] for i in A2) == 0)
            end
            # @constraint(model, sum(y[combination[1], i] for i in A2) + sum(y[i, combination[1]] for i in A2) + sum(y[combination[2],i] for i in A2) <= 1)
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

function solveRestrictedMasterProblem(routes_1e, routes_2e_pool, branchingInfo)

    ## Relaxed Restricted Master Problem
    routes_originated_p = Vector{Vector{Int}}()
    for s in satellites 
        routes = Vector{Int}()
        for (r,route) in enumerate(routes_2e_pool)
            if route.sequence[1] == s
                push!(routes,r)
            end
        end
        push!(routes_originated_p, routes)
    end

    # println("Number of 2e routes: ", length(routes_2e_pool))
    # println("Number of 1e routes: ", length(routes_1e))
    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, 1>=x[1:length(routes_1e)]>=0)
    @variable(model, 1>=y[1:length(routes_2e_pool)]>=0)

    forbidden_2e_routes = Vector{Int}()
    for (idx,route) in enumerate(routes_2e_pool) 
        for parking in branchingInfo.forbidden_parkings 
            if parking in route.sequence
                push!(forbidden_2e_routes, idx)
            end
        end
    end

    if !isempty(branchingInfo.lower_bound_number_2e_routes)
        lower_bound = maximum(branchingInfo.lower_bound_number_2e_routes) 
        @constraint(model, sum(y[r] for (r, route) in enumerate(routes_2e_pool))>=lower_bound)
    end
        if !isempty(branchingInfo.upper_bound_number_2e_routes)
        upper_bound = minimum(branchingInfo.upper_bound_number_2e_routes) 
        @constraint(model, sum(y[r] for (r, route) in enumerate(routes_2e_pool))<=upper_bound)
    end
    
    special_order_set_must = Set{Int}()
    special_order_set_forbidden = Set{Int}()

    for (idx, route) in enumerate(routes_2e_pool) 
        for r in branchingInfo.special_order_set_must_include 
            if route.sequence == r.sequence
                push!(special_order_set_must, idx)
            end
        end
        for r in branchingInfo.special_order_set_forbidden_include 
            if route.sequence == r.sequence
                push!(special_order_set_forbidden, idx)
            end
        end
    end
    # display(special_order_set_must)
    # display(special_order_set_forbidden)

    if !isempty(special_order_set_must)
        @constraint(model, [r in special_order_set_must], y[r] == 1)
    end
    if !isempty(special_order_set_forbidden)
        @constraint(model, sum(y[r] for r in special_order_set_forbidden) == 0)
    end

    # @constraint(model, microhubUB, sum(route.b1[s] * x[r] for (r, route) in enumerate(routes_1e) for s in satellites)<=nb_microhub)
    @constraint(model, sync[s in satellites], sum(route.b2out[s] * y[r] for (r, route) in enumerate(routes_2e_pool))
        -nb_vehicle_per_satellite * sum(route.b1[s] * x[r] for (r, route) in enumerate(routes_1e))<=0)
    @constraint(model, custVisit[i in customers], 1 - sum(route.a[i-1-length(satellites)] * y[r] for (r,route) in enumerate(routes_2e_pool)) <= 0 )
    @constraint(model, number2evfixe[s in satellites], sum(route.b2in[s] * y[r] for (r, route) in enumerate(routes_2e_pool)) == sum(route.b2out[s] * y[r] for (r, route) in enumerate(routes_2e_pool)))
    @constraint(model, maxVolumnMM[s in satellites], sum( routes_2e_pool[r].a[i-1-length(satellites
    )]*demands[i]*y[r] for r in routes_originated_p[s-1] for i in customers) - capacity_microhub <= 0)
    @constraint(model, single1eV, sum(x[r] for (r,_) in enumerate(routes_1e))>=1)
    # @constraint(model, maxMMnumber, sum(x[r]*sum(route.b1) for (r,route) in enumerate(routes_1e)) <= nb_microhub)
    @constraint(model, min2eRoute, sum(y[r] for (r,_) in enumerate(routes_2e_pool))>=ceil(sum(demands)/capacity_2e_vehicle))

    @objective(model, Min, sum(y[r] * route.cost for (r,route) in enumerate(routes_2e_pool)) + 
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
        # for (r,route) in enumerate(routes_2e_pool)
        #     if value(y[r]) != 0
        #         println("   y=",round(value(y[r]),digits=2),"   2e Route ", route.sequence, "   Load= ", route.load , "   Cost= ", round(route.cost,digits=2))
        #     end
        # end

        # println("\n   Total number of active satellites: ",sum( route.b1[s] * value(x[r]) for s in satellites for (r,route) in enumerate(routes_1e)))
        # println("   Total number of 2e routes: ", sum(value(y[r]) for r in 1:length(routes_2e_pool)))
        
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
        for (r,_) in enumerate(routes_2e_pool)
            push!(y_value, value(y[r]))
        end
        
        return π1, π2, π3, π4, x_value, y_value, objective_value(model)
    else
        return nothing
    end
end
