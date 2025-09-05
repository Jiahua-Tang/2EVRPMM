using JuMP, CPLEX, Plots, Random, DataStructures, Combinatorics, Printf
import DataFrames
import HiGHS
import Plots
import SparseArrays
import Test  #src
include("Utiles.jl")
# include("labelling.jl")
include("../Utiles.jl")
# include("ngPathLabelling.jl")

generateData()
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

function solveColumnGeneration(filtered_1e_routes, filtered_2e_routes, branchingInfo)
    # println

    selected_parkings = getServedParking1eRoute(filtered_1e_routes[1])

    #region : create model
    model = Model(CPLEX.Optimizer)
    set_silent(model)
    set_optimizer_attribute(model, "CPXPARAM_Threads", 1)
    set_optimizer_attribute(model, "CPXPARAM_MIP_Display", 0)

    y_vars = Dict{Int, VariableRef}()

    @objective(model, Min, 0.0)

    sync = Vector{ConstraintRef}(undef, length(satellites))
    for (k,s) in enumerate(satellites)
        # println("$k $s")
        sync[k] = @constraint(model, -nb_vehicle_per_satellite <= 0.0)
    end

    custVisit = Vector{ConstraintRef}(undef, length(customers))
    for (k,s) in enumerate(customers) 
        custVisit[k] = @constraint(model, 1.0 <= 0.0)
    end

    number2evfixe = Vector{ConstraintRef}(undef, length(satellites))
    for (k,s) in enumerate(satellites)
        number2evfixe[k] = @constraint(model, 0.0 == 0.0)
    end

    maxVolumnMM = Vector{ConstraintRef}(undef, length(satellites))
    for (k,s) in enumerate(satellites) 
        maxVolumnMM[k] = @constraint(model, -capacity_microhub <= 0.0)
    end

    lower_bound = minimum_2e_vehicle_required
    if !isempty(branchingInfo.lower_bound_number_2e_routes)
        lower_bound = maximum(branchingInfo.lower_bound_number_2e_routes) 
    end
    upper_bound = nb_parking * nb_vehicle_per_satellite
    if !isempty(branchingInfo.upper_bound_number_2e_routes)
        upper_bound = minimum(branchingInfo.upper_bound_number_2e_routes) 
    end
    globalLowerBound = @constraint(model, lower_bound <= 0)
    globalUpperBound = @constraint(model, -upper_bound <= 0)

    #endregion

    # ## Initial columns
    for (idx, route) in enumerate(filtered_2e_routes)
        add_2eroute!(model, idx, route, 
            sync, custVisit, number2evfixe, maxVolumnMM, globalLowerBound, globalUpperBound,
            demands, y_vars)
    end
    
    lpObjValue = 0.0
    # y = nothing

    for it in 1:20
        println("\n==$it==")

        #region : display routes pool info  
        # println("length of routes pool = ",length(filtered_2e_routes))
        # for route in filtered_2e_routes 
        #     println(route.sequence)
        # end
        #endregion
        # println("Test")
        execution_time = @elapsed begin
            optimize!(model)
        end
        global solving_rmp_time += execution_time
        println("LP Objective Value = $(round(objective_value(model), digits=2)),   sum y = $(round(sum(value.(values(y_vars))), digits=2))")
        lpObjValue = objective_value(model)
        for (rid, y) in y_vars
            if value(y)!=0
                # println("Route $(initial_2e_routes[rid].sequence): y = ", value(y))
                println("y$(filtered_2e_routes[rid].sequence) = ", round(value(y),digits=2), "  ", round(filtered_2e_routes[rid].cost, digits=2))
            end
        end

        #region : retrieve and display dual multiplier
        # Keep JuMP's original dual signs
        π1 = vcat(0, abs.(shadow_price.(sync)))
        π2 = vcat(zeros(1+length(satellites)), abs.(shadow_price.(custVisit)))
        π3 = vcat(0, abs.(shadow_price.(number2evfixe)))
        π4 = vcat(0, abs.(shadow_price.(maxVolumnMM)))

        # display(π1)
        # println("")
        # display(π2)
        # println("")
        # display(π3)
        # println("")
        # display(π4)
        # println("")
        #endregion

        filtered_2e_routes, new_routes_from = pricing(selected_parkings, filtered_2e_routes, π1, π2, π3, π4, branchingInfo)

        if new_routes_from == false
            break
        end

        # println(length(new_routes_2e))
        r_id = new_routes_from
        for route in filtered_2e_routes[new_routes_from:end]
            add_2eroute!(model, r_id, route,
                            sync, custVisit, number2evfixe, 
                            maxVolumnMM, globalLowerBound, globalUpperBound,demands,y_vars)
            r_id += 1
        end
        it += 1
    end



    return nothing, filtered_2e_routes, nothing, value.(values(y_vars)), lpObjValue 

end

function add_2eroute!(model::Model,
                      r_id::Int, route::Route,
                      sync::Vector{ConstraintRef},
                      custVisit::Vector{ConstraintRef},
                      number2evfixe::Vector{ConstraintRef},
                      maxVolumnMM::Vector{ConstraintRef},
                      globalLowerBound::ConstraintRef,
                      globalUpperBound::ConstraintRef,
                      demands::AbstractVector,
                      y_vars::Dict{Int, VariableRef})
    # println("add column $(route.sequence)")
    ## create column variable
    y = @variable(model, lower_bound = 0.0)
    y_vars[r_id] = y

    ## objective coefficients
    JuMP.set_objective_coefficient(model, y, route.cost)

    ## sync constraint : + b2out[s] * y
    @inbounds for s in eachindex(sync) 
        b = route.b2out[s+1]
        # println("$(route.sequence) b2out[$(s+1)] = $b")
        if b != 0
            JuMP.set_normalized_coefficient(sync[s], y, b)
        end
    end

    ## customer-coverage: 1 - sum(a_i * y) <= 0  → coefficient is -a_i
    # println("")
    @inbounds for i in eachindex(custVisit)
        ai = route.a[i+length(A1)]
        # ai == 1 && println(route.sequence, " a[$(i+length(A1))] = $ai")
        if ai != 0
            set_normalized_coefficient(custVisit[i], y, -ai)
        end
    end

    ## 2E flow balance per satellite: sum(b2in) - sum(b2out) == 0
    @inbounds for s in eachindex(number2evfixe)
        coeff = route.b2in[s+1] - route.b2out[s+1]
        if coeff != 0
            set_normalized_coefficient(number2evfixe[s], y, coeff)
        end
    end

    ## microhub capacity at origin s0: ∑ a_i * demand_i * y - cap ≤ 0  (only for origin)
    load = 0
    for i in route.sequence 
        load += demands[i]
    end

    @inbounds for s in eachindex(maxVolumnMM) 
        b = route.b2out[s+1]
        if b != 0
            JuMP.set_normalized_coefficient(maxVolumnMM[s], y, load)
        end
    end

    JuMP.set_normalized_coefficient(globalLowerBound, y, -1)
    JuMP.set_normalized_coefficient(globalUpperBound, y, 1)

    return y
end

 function pricing(selected_parkings, routes_2e, π1, π2, π3, π4, branchingInfo::BranchingInfo) 
    #region : dual multiplier verification
    # for route in routes_2e
    #     r = route.sequence
    #     rc = 0
        
    #     # 1. Original arc costs
    #     rc += route.cost
        
    #     # 2. Customer coverage: 1 - sum(a[i] * y) <= 0
    #     # Coefficient: -a[i] = -1 for visited customers  
    #     # RC contribution: π2[i] * (-1) = -π2[i]
    #     l = 0
    #     for cust in r[2:end-1]
    #         rc -= π2[cust]  # Keep original: -π2[cust]
    #         l += demands[cust]
    #     end

    #     # 3. Sync constraint: sum(b2out[s] * y) - nb_vehicle <= 0  
    #     # Coefficient: +b2out[s] for origin satellite
    #     # RC contribution: π1[s] * b2out[s]
    #     origin_s = r[1]
    #     rc += π1[origin_s] * route.b2out[origin_s]
        
    #     # 4. Flow balance: sum((b2in[s] - b2out[s]) * y) == 0
    #     # Coefficient: (b2in[s] - b2out[s]) for each satellite
    #     # RC contribution: π3[s] * (b2in[s] - b2out[s])
    #     for s in satellites
    #         flow_coeff = route.b2in[s] - route.b2out[s]
    #         rc += π3[s] * flow_coeff
    #     end
        
    #     # 5. Capacity: sum(a[i] * demand[i] * y) - capacity <= 0
    #     # Coefficient: +sum(a[i] * demand[i]) for origin
    #     # RC contribution: π4[s] * total_demand
    #     rc += π4[origin_s] * l

    #     if rc <= -1e-8
    #         val_2 = 0
    #         for cust in r[2:end-1]
    #             val_2 += π2[cust]
    #         end            
    #         println("rc of $r : $(round(rc,digits=2)) \n     cost: $(round(route.cost, digits=2))   val_2: $(round(val_2,digits=2))")
    #     end
    # end
    #endregion
    execution_time = @elapsed begin
        subproblem_2e_result = labelling(π1, π2, π3, π4, selected_parkings, branchingInfo)
    end
    global execution_time_subproblem += execution_time
    # println("TEST return result from labelling")
    new_routes_from = length(routes_2e) + 1
    new_routes_generated = false
    for label in subproblem_2e_result
        
        route_existed = false
        for route in routes_2e 
            if route.sequence == label.visitedNodes
                route_existed = true
                break
            end
        end

        if !route_existed
            new_route = generate2eRoute(label.visitedNodes)
            if !(new_route in routes_2e)
                # println(label.visitedNodes, "  ", round(label.reduced_cost, digits=2))
                push!(routes_2e, new_route)
                new_routes_generated = true
            end
        end


    end
    println(new_routes_generated, "  ", length(routes_2e[new_routes_from:end]))
    if new_routes_generated
        return routes_2e, new_routes_from
    else
        return routes_2e, false
    end
    

end

function dominanceRule(label1, label2)
    rcBool = label1.reduced_cost <= label2.reduced_cost
    capBool = label1.accumulated_capacity <= label2.accumulated_capacity
    # durationBool = label1.accumulated_duration <= label2.accumulated_duration
    ineBool = label1.reduced_cost == label2.reduced_cost && label1.accumulated_capacity == label2.accumulated_capacity # && label1.accumulated_duration == label2.accumulated_duration
    if rcBool && capBool&& !ineBool # && durationBool 
        # @info "Dominance relation found: "
        # println("$(label1.visitedNodes) $(round(label1.reduced_cost, digits=2)) dominates $(label2.visitedNodes)  $(round(label2.reduced_cost, digits=2))")
        return label2
    else
        return nothing
    end
end

function dominanceCheckSingle(l1, l2)
    result = dominanceRule(l1, l2)
    if !isnothing(result)
        ## l1 dominate l2
        return 2
    else
        result = dominanceRule(l2, l1)
        if !isnothing(result)
            ## l2 dominate l1
            return 1
        else
            ## no dominance relation exsit
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

function extendLabel_v2(π2, π3, π4, label::Label, next_node::Int)
    # displayLabel(label)

    ## Update reduced cost
    reduced_cost = label.reduced_cost + arc_cost[label.current_node, next_node]

    if next_node in customers
        reduced_cost = reduced_cost - π2[next_node] +
                         π4[label.origin_node] * demands[next_node]
    end

    if next_node in satellites
        reduced_cost += π3[next_node]       
    end

    ## Update accumulated capacity
    accumulated_capacity = label.accumulated_capacity + demands[next_node]
    if accumulated_capacity > capacity_2e_vehicle
        return nothing
    end

    ## Update accumulated duration
    accumulated_duration = label.accumulated_duration + arc_cost[label.current_node, next_node]
    if accumulated_duration > maximum_duration_2e_vehicle
        return nothing
    end

    visitedNodes = push!(deepcopy(label.visitedNodes), next_node)
    new_label = Label(label.origin_node, next_node, reduced_cost, accumulated_capacity, accumulated_duration, visitedNodes)
    # displayLabel(new_label)
    return new_label

end

function labelling(π1, π2, π3, π4, selected_parkings, branchingInfo)

    ## Branching strategies include: 
    ## Case A : total number of 2e routes (constraint added in master problem)
    ## Case B : special order set (constraint added in master problem) TODO 
    ## Case C : combination of parking - customer
    ## Case D : combination of customer - customer

    unprocessedLabels = Dict{Int, Vector{Label}}()
    processedLabels = Dict{Int, Vector{Label}}()
    depotLabels = Vector{Label}()
    
    active_nodes = vcat(collect(selected_parkings), customers)

    # println(active_nodes)
    for node in active_nodes
        unprocessedLabels[node] = Vector{Label}()
        processedLabels[node] = Vector{Label}()
    end

    for parking in selected_parkings
        rc = π1[parking] - π3[parking] 
        l = Label(parking, parking, rc, 0, 0, [parking])
        push!(unprocessedLabels[parking], l)
    end

    result = []

    num_iter_labelling = 1 #  num_iter_labelling < 1001 &&
    while !isempty(collect(Iterators.flatten(values(unprocessedLabels))))
        #region : Display labels
        # println("\n===================Iter $num_iter_labelling===================")
        # println("Display $(length(collect(Iterators.flatten(values(unprocessedLabels))))) Unprocessed Labels")
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
        #        displayLabel(ele)
        #     end
        # end

        # println("Display $(length(depotLabels)) Depot Labels")
        # for label in depotLabels
        #    displayLabel(label) 
        # end
        # println("")
        #endregion

        ## Line 3
        all_labels = collect(Iterators.flatten(values(unprocessedLabels)))
        min_label = all_labels[findmin(l -> l.reduced_cost, all_labels)[2]]
        # @info "Selected label:"
        # displayLabel(min_label)

        min_idx = findfirst(==(min_label), unprocessedLabels[min_label.visitedNodes[end]])
        deleteat!(unprocessedLabels[min_label.visitedNodes[end]], min_idx)

        ## Line 9
        push!(processedLabels[min_label.visitedNodes[end]], min_label)

        ## Line 4 : Propagation to new node
        ## Line 5
        # @info "Propagate labels:"
        current_node = min_label.visitedNodes[end]
        for node in active_nodes
            #region
            ## Combination parking - customer
            ## must include : customer cannot be served by other parking
            for combination in branchingInfo.must_include_combinations 
                if combination[1] != min_label.visitedNodes[1]
                    filter!(x -> x!= combination[2], active_nodes)
                end
            end
            ## mustn't include : customer cannot be served by selected parking
            for combination in branchingInfo.forbidden_combinations 
                if combination[1] == min_label.visitedNodes[1]
                    filter!(x -> x!= combination[2], active_nodes)
                end
            end

            ## mustn't include : two customers cannot exist together
            for combination in branchingInfo.forbidden_served_together
                if combination[1] in min_label.visitedNodes
                    filter!(x -> x != combination[2], active_nodes)
                end
                if combination[2] in min_label.visitedNodes
                    filter!(x -> x != combination[1], active_nodes)
                end
            end

            #endregion

            # println(active_nodes)
            # Set node in visiting sequence as unreachable
            if node in selected_parkings || (!(node in min_label.visitedNodes) && !(node in selected_parkings))
                # println("Propagate from $(min_label.visitedNodes[end]) to $node")
                new_label = extendLabel_v2(π2, π3, π4, min_label, node)
                # println(new_label)
                if !isnothing(new_label)
                    ## Combination customer - customer
                    ## must include : two customers both exist or not exist
                    for combination in branchingInfo.must_served_together
                        if Int(combination[1] in new_label.visitedNodes) + 
                           Int(combination[2] in new_label.visitedNodes) == 1
                            new_label = nothing
                            break
                        end
                    end         
                end       
                ## Line 6
                if !isnothing(new_label)
                    ## Line 7
                    if node in selected_parkings
                        push!(depotLabels, new_label)
                        if new_label.reduced_cost < -1e-8 && length(new_label.visitedNodes)>2
                            push!(result, new_label)
                        end
                    else
                        new_label_is_dominated = false                            
                        ## Line 8
                        ## Check if new label is dominated, if yes, do not add it in
                        for label in processedLabels[node]
                            # if label.visitedNodes[1] == new_label.visitedNodes[1]
                            if dominanceCheckSingle(new_label, label) == 1
                                ## new label is dominated by a processed label
                                new_label_is_dominated = true
                                break
                            end                               
                            # end
                        end
                    
                        ## Check if new label is dominated by or dominates a unprocessed label
                        if !new_label_is_dominated
                            for label in unprocessedLabels[node]
                                # if label.visitedNodes[1] == new_label.visitedNodes[1]
                                if dominanceCheckSingle(new_label, label) == 1
                                    ## new label is dominated by a unprocessed label
                                    new_label_is_dominated = true
                                    break
                                elseif dominanceCheckSingle(new_label,label) == 2
                                    # println("a unprocessed label is dominated")
                                    min_idx = findfirst(==(label), unprocessedLabels[node])
                                    deleteat!(unprocessedLabels[node], min_idx)
                                end
                                # end
                            end
                        end
                        # println(new_label_is_dominated)
                        if !new_label_is_dominated 
                            push!(unprocessedLabels[node], new_label)
                        end
                    end
                end    
            end
        end
        num_iter_labelling += 1
    end

    return result
end

function displayLabel(label::Label)
    println("- Origin parking: $(label.origin_node)")
    println("  Current node: $(label.current_node)")
    println("  Reduced cost: $(round(label.reduced_cost, digits=2))")
    println("  Accumulated capacity: $(label.accumulated_capacity)")
    println("  Accumulated duration: $(round(label.accumulated_duration, digits=2))")
    println("  Visit sequence: $(label.visitedNodes)\n")
end
