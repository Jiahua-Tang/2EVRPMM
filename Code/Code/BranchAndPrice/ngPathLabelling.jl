include("Utiles.jl")


function displayLabel(label::Label)
    println("- Origin parking: $(label.origin_node)")
    println("  Current node: $(label.current_node)")
    println("  Reduced cost: $(round(label.reduced_cost, digits=2))")
    println("  Accumulated capacity: $(label.accumulated_capacity)")
    println("  Accumulated duration: $(round(label.accumulated_duration, digits=2))")
    println("  Visit sequence: $(label.visitedNodes)\n")
end

function displayNgLabel(label::ngLabel)
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
    println("Start labelling algo for parking $startParking")
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
    # 
    while num_iter_labelling < 11 && !isempty(collect(Iterators.flatten(values(unprocessedLabels)))) 
        println("\n===================Iter $num_iter_labelling===================")
        println("Display $(length(unprocessedLabels)) Unprocessed Labels")
        for node in active_nodes
            if !isempty(unprocessedLabels[node])
                print("-")
            end
            for (idx, ele) in enumerate(unprocessedLabels[node])
                if idx > 1
                    print(" ")
                else
                    print("")
                end
                printLabel(ele)
            end
        end        
        println("Display Depot Labels")
        for node in active_nodes
            if !isempty(depotLabels[node])
                print("-")
            end
            for (idx, ele) in enumerate(depotLabels[node])
                if idx > 1
                    print(" ")
                else
                    print("")
                end
                printLabel(ele)
            end
        end
        println("Display Processed Labels")
        for node in active_nodes
            if !isempty(processedLabels[node])
                print("-")
            end
            for (idx, ele) in enumerate(processedLabels[node])
                if idx > 1
                    print(" ")
                else
                    print("")
                end
                printLabel(ele)
            end
        end

        ## Line 3
        all_labels = collect(Iterators.flatten(values(unprocessedLabels)))
        min_label = all_labels[findmin(l -> l.reduced_cost, all_labels)[2]]
        @info "Selected label:"
        displayLabel(min_label)
        min_idx = findfirst(==(min_label), unprocessedLabels[min_label.visitedNodes[end]])
        deleteat!(unprocessedLabels[min_label.visitedNodes[end]], min_idx)
        ## Line 9
        push!(processedLabels[min_label.visitedNodes[end]], min_label)
        ## Line 4
        ## Propagate to new node
        ## Line 5
        node_pool = setdiff(active_nodes, min_label.current_node)
        if length(min_label.visitedNodes)>2 
            node_pool = setdiff(node_pool, min_label.visitedNodes[end-1])
        end
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

function solve_2e_ng_labelling(pi1, pi2, pi3, pi4, startParking, branchingInfo)
    println("Start ng-path labelling algo")
    
    ## Initialization
    selected_parkings = Vector{Int}()
    for parking in satellites 
        if !(parking in branchingInfo.forbidden_parkings)
            push!(selected_parkings, parking)
        end
    end
    active_nodes = vcat(selected_parkings, customers)

    processedLabels = Dict{Int, Vector{ngLabel}}()
    unprocessedLabels = Dict{Int, Vector{ngLabel}}()
    depotLabels = Dict{Int, Vector{ngLabel}}()
    for node in selected_parkings
        depotLabels[node] = Vector{Label}()
    end
    for node in customers   
        unprocessedLabels[node] = Vector{Label}()
        processedLabels[node] = Vector{Label}()
    end
    E0 = ngLabel(startParking, startParking, pi1[startParking]-pi3[startParking], 0, 0, zeros(Int, length(customers)) , [startParking])
    unprocessedLabels[startParking] = [E0]
    size_neighbour = 3
    neighbours = generateNeighbour(active_nodes, size_neighbour)
    display(neighbours)

    num_iter_labelling = 1
    # # 
    while num_iter_labelling < 11 && !isempty(collect(Iterators.flatten(values(unprocessedLabels)))) 
        ## Pick the label with minimum reduced cost
        min_label = selectMinLabel(unprocessedLabels)

        push!(processedLabels[min_label.visitedNodes[end]], min_label)
        
        node_pool = setdiff(active_nodes, min_label.current_node)
        if length(min_label.visitedNodes)>2 
            node_pool = setdiff(node_pool, min_label.visitedNodes[end-1])
        end

        for node in node_pool 
            new_label = extendLabel(pi1, pi2, pi3, pi4, min_label, node, branchingInfo)
            
        
        end


        num_iter_labelling += 1
    end
end

function generateNeighbour(selected_nodes, size_neighbour)
    result = Dict{Int, Vector{Int}}()
    for node in selected_nodes
        # Get all possible destinations except the node itself
        candidates = [(other, arc_cost[node, other]) for other in selected_nodes if other != node]
        
        # Sort by cost
        sorted_candidates = sort(candidates, by = x -> x[2])
        
        # Take top size_neighbour neighbors
        neighbours = [x[1] for x in Iterators.take(sorted_candidates, size_neighbour)]
        
        result[node] = neighbours
    end
    return result
end

function selectMinLabel(unprocessedLabels)
    all_labels = collect(Iterators.flatten(values(unprocessedLabels)))
    min_label = all_labels[findmin(l -> l.reduced_cost, all_labels)[2]]
    @info "Selected label:"
    displayNgLabel(min_label)
    min_idx = findfirst(==(min_label), unprocessedLabels[min_label.visitedNodes[end]])
    deleteat!(unprocessedLabels[min_label.visitedNodes[end]], min_idx)
    return min_label
end
