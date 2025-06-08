using  JuMP, CPLEX, DataFrames, DataStructures

using JuMP, CPLEX, Plots, Random, DataStructures, Combinatorics, Printf
import DataFrames
import HiGHS
import Plots
import SparseArrays
import Test  #src
include("Utiles.jl")


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
    # println("Start labelling algo")
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
        ## Propagate to new node
        ## Line 5
        node_pool = setdiff(active_nodes, min_label.current_node)
        # for forbidden_combination in branchingInfo.forbidden_served_together
        #     if forbidden_combination[1] in min_label.visitedNodes
        #         node_pool = setdiff(node_pool, forbidden_combination[2])
        #     end
        #     if forbidden_combination[2] in min_label.visitedNodes
        #         node_pool = setdiff(node_pool, forbidden_combination[1])
        #     end
        # end
        for node in node_pool
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
    end

    return result

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
        π1 = [dual(sync[i]) for i in keys(sync)]
        π2 = [dual(custVisit[i]) for i in keys(custVisit)]
        π3 = [dual(number2evfixe[i]) for i in keys(number2evfixe)]
        π4 = [dual(maxVolumnMM[i]) for i in keys(maxVolumnMM)]

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