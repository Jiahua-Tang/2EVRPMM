
function printLabelLRP(label::LabelLRP)
    @printf("  Label(%d,  %2d,  RC: %7.2f,  %s)\n",
    label.origin_node,
    label.current_node,
    label.reduced_cost,
    label.visitedNodes
    )
end

function dominanceRule(label1::LabelLRP, label2::LabelLRP)
    rcBool = label1.reduced_cost < label2.reduced_cost
    if rcBool
        # @info "Dominance relation found: "
        printLabelLRP(label1)
        @info "dominates"
        printLabelLRP(label2)
        println("")
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


function extendLabel(π1, π2, π3, label::LabelLRP, next_node::Int)
    # displayLabel(label)
    # Update reduced cost
    reduced_cost = label.reduced_cost + arc_cost[label.current_node, next_node]
    # if label.current_node in satellites
    #     reduced_cost += pi1[label.current_node] - pi3[label.current_node]
    # end
    if next_node in customers
        reduced_cost = reduced_cost - π2[next_node]
    end
    if next_node in satellites
        reduced_cost -= π3[next_node]       
    end

    visitedNodes = push!(deepcopy(label.visitedNodes), next_node)
    new_label = LabelLRP(label.origin_node, next_node, reduced_cost, visitedNodes)
    # displayLabel(new_label)
    return new_label
end

function solveLRP_SP_Labelling(π1, π2, π3, selected_parkings)
    println("Solve LRP 2e Labeling")

    active_nodes = vcat(selected_parkings, customers)

    ## Line 1
    unprocessedLabels = Dict{Int, Vector{LabelLRP}}()
    processedLabels = Dict{Int, Vector{LabelLRP}}()
    depotLabels = Dict{Int, Vector{LabelLRP}}()

    for node in active_nodes 
        unprocessedLabels[node] = Vector{Label}()
        processedLabels[node] = Vector{Label}()
        if node in selected_parkings
            depotLabels[node] = Vector{Label}()
        end
    end

    for node in selected_parkings 
        label = LabelLRP(node, node, -π1[node]+π3[node], [node])
        push!(unprocessedLabels[node], label)
    end
    
    ## Line 2
    num_iter_labelling = 1
    # 
    while num_iter_labelling < 3 && !isempty(collect(Iterators.flatten(values(unprocessedLabels))))
        println("\n===================Iter $num_iter_labelling Labelling===================")
        println("Display Unprocessed Labels")
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
                printLabelLRP(ele)
            end
        end        
        println("Display Depot Labels")
        for node in selected_parkings
            if !isempty(depotLabels[node])
                print("-")
            end
            for (idx, ele) in enumerate(depotLabels[node])
                if idx > 1
                    print(" ")
                else
                    print("")
                end
                printLabelLRP(ele)
            end
        end
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
        #         printLabelLRP(ele)
        #     end
        # end


        ## Line 3
        all_labels = collect(Iterators.flatten(values(unprocessedLabels)))
        min_label = all_labels[findmin(l -> l.reduced_cost, all_labels)[2]]
        # @info "Selected label:"
        # printLabelLRP(min_label)
        min_idx = findfirst(==(min_label), unprocessedLabels[min_label.visitedNodes[end]])
        deleteat!(unprocessedLabels[min_label.visitedNodes[end]], min_idx)

        ## Line 9
        push!(processedLabels[min_label.visitedNodes[end]], min_label)
        ## Line 4
        ## Propagate to new node
        ## Line 5
        for node in setdiff(active_nodes, min_label.visitedNodes)
            new_label = extendLabel(π1, π2, π3, min_label, node)
            ## Line 6
            if !isnothing(new_label)
                ## Line 7
                node in satellites && push!(depotLabels[node], new_label)
                node in customers && push!(unprocessedLabels[node], new_label)

                # Line 8
                if node in customers
                    unprocessedLabels[node], processedLabels[node] = dominanceCheck(unprocessedLabels[node], processedLabels[node])
                end
            end
        end

        num_iter_labelling += 1
    end

    result = []
    for node in selected_parkings 
        for label in depotLabels[node]
            if label.reduced_cost < -1e-8 && length(label.visitedNodes) > 2
                # printLabelLRP(label)
                push!(result, label)
            end
        end        
    end
    println("Number of labels found: $(length(result))")
    return result  

end