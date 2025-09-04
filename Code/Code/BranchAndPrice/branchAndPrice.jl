include("branchingStrategies.jl")
include("Utiles.jl")
include("columnGeneration.jl")
include("columnGeneration_v2.jl")


function filter_2e_routes(branchingInfo::BranchingInfo, routes::Vector{Route})
    result = Vector{Route}()
    # displayBranchingRule(branchingInfo)
    
    for route in routes
        valide = true

        if route.sequence[1] in branchingInfo.forbidden_parkings || route.sequence[end] in branchingInfo.forbidden_parkings
            valide = false
        end

        # for must_include in branchingInfo.must_include_combinations 
        # ## a route include a must-combination on the branch can be added into 
        # ## target parking and target customer must be in same route 
        #     if must_include[2] in route.sequence && !(must_include[1] in route.sequence)
        #         # println("TEST1  ")
        #         valide = false
        #         break            
        #     end
        # end    

        # for forbidden in branchingInfo.forbidden_combinations 
        # # a route include a forbidden-combination on the branch cannot be added into result
        # ## target parking and target customer cannnot be in same route
        #     if Int(forbidden[1] in route.sequence) + Int(forbidden[2] in route.sequence) == 2
        #         # println("TEST2  ")
        #         valide = false
        #         break
        #     end
        # end

        for must_include in branchingInfo.must_include_combinations 
        ## a route include a must-combination on the branch can be added into 
        ## target parking and target customer must be in same route 
            if must_include[2] in route.sequence && !(must_include[1] == route.sequence[1])
                # println("TEST1  ")
                valide = false
                break            
            end
        end    

        for forbidden in branchingInfo.forbidden_combinations 
        # a route include a forbidden-combination on the branch cannot be added into result
        ## target parking and target customer cannnot be in same route
            if Int(forbidden[1] == route.sequence[1]) + Int(forbidden[2] in route.sequence) == 2
                # println("TEST2  ")
                valide = false
                break
            end
        end

        for customers in branchingInfo.must_served_together
        # if a route include only one of must serve together customers cannot be added into result
            if (Int(customers[1] in route.sequence) + Int(customers[2] in route.sequence)) == 1
                # println("TEST3  ")
                # println(route.sequence, "  ", customers, "   ",(Int(customers[1] in route.sequence) + Int(customers[2] in route.sequence)))
                valide = false
                break
            end
        end

        for customers in branchingInfo.forbidden_served_together
        # if a route include both of must serve together customers cannot be added into result
            if (customers[1] in route.sequence) && (customers[2] in route.sequence)
                # println("TEST4  ")
                valide = false
                break
            end
        end

        if valide
            push!(result, route)
        end
        # print("$(route.sequence)   $valide")
    end

    return result
end



function createBranchingNode(route_1e, routes_2e_pool, branchingInfo, cgLowerBound, fs, num_iter_sp, id, parent_id)
## given a branchingInfo, transform it into a branchingNode structure (add cg result)
    execution_time = @elapsed begin
        routes_2e_pool = filter_2e_routes(branchingInfo, routes_2e_pool)
    end
    global filtering_time += execution_time
    
    # TODO
    ## Before start column generation, check branching rules conflic
    @info "Start column generation for node N_$id, parent node N_$parent_id, depth $(branchingInfo.depth)"
    displayBranchingRule(branchingInfo)
    # result = column_generation(route_1e, routes_2e_pool, branchingInfo)
    result = solveColumnGeneration(route_1e, routes_2e_pool, branchingInfo)
    
    # println(result)
    # for route in routes_2e_pool 
    #     println(route.sequence)
    # end        

    if !isnothing(result)
        # println("teset")
        y_value = result[4]
        fractionalScore = 0
        # for (_, y) in enumerate([r for r in 1:length(y_value) if 0 < y_value[r]]) 
        #     println("   $(routes_2e_pool[y].sequence)  $(round(y_value[y],digits=2))")
        # end
        routes_2e_pool = result[2]
        if checkExistanceDummyRoute(y_value, routes_2e_pool)
        ## Dummy route used at the end of column generation, branch can be pruned
            @info "Dummy routes used, exceed upper bound, prune"
            return nothing

        elseif isempty([r for r in 1:length(y_value) if 0 < y_value[r] < 1])
        ## Integer solution found, note as a leaf node
            ## display CG result
            # println("   Total number of 2e routes: ", sum(y_value) , ", lb of cg = $(round(result[5],digits=2))")
            # for (_, y) in enumerate([r for r in 1:length(y_value) if 0 < y_value[r]]) 
            #     println("   $(routes_2e_pool[y].sequence)  $(round(y_value[y],digits=2))")
            # end
            @info "Integer Solution Found  $(result[5])"
            isLeaf = true
            if result[5] < upperBound
                global  upperBound
                upperBound = result[5]
                global optimalSolution
                optimalSolution = Vector{Route}()
                global optimal_found_in
                optimal_found_in = branchingInfo.depth
                global optimal_found_iteration = num_iter_sp
                push!(optimalSolution, route_1e[1])
                for (_, y) in enumerate([r for r in 1:length(y_value) if y_value[r]==1]) 
                    push!(optimalSolution, routes_2e_pool[y])
                end             
            end
            gradientLB = 0
            gradientFS = 0   
        else
        ## Create a child node
            # println("Column generation lower bound is $(round(result[5],digits=2))")
            for value in y_value
                if value <= 0.5
                    fractionalScore += value
                else
                    fractionalScore += 1 - value
                end
            end
            gradientLB = result[5] - cgLowerBound
            gradientFS = fractionalScore - fs
            # println("Fractional score: $(round(fractionalScore, digits = 3))\n")
            isLeaf = false
        end
        branchingNode = BranchingNode(branchingInfo, result[5], y_value, isLeaf, fractionalScore, routes_2e_pool, gradientLB, gradientFS, id, parent_id)
        return branchingNode
    else
        @info "RLMP infeasible, prune "
        return nothing
    end
end


function branchAndPriceWithScore(route_1e::Vector{Route})
    root_branch = BranchingInfo(Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Int}(), Set{Int}(), Set{Int}(), Set{Int}(),Set{Route}(),Set{Route}(), 0)
    root_branch.forbidden_parkings = setdiff(Set(satellites), getServedParking1eRoute(route_1e[1]))
    current_id = 0
    # CG for root node
    println("\n================Iteration 0 of B&P for SP$num_iter_global $(route_1e[1].sequence)================")
    branchingNode = createBranchingNode(route_1e, routes_2e, root_branch, 0,0,0,0,0 )
    node_stack = [branchingNode]
    if isnothing(branchingNode)
        return
    end

    println("Branching stack contains now $(length(node_stack)) nodes, current upper bound is $(round(upperBound,digits=2))")

    num_iter_sp = 1
    while !isempty(node_stack) && num_iter_sp < 3
        println("\n================Iteration $num_iter_sp of B&P for SP$num_iter_global $(route_1e[1].sequence)================")

        #region : Different node selection strategies
        ## Display current node stack
        # @info "Node stack contains $(length(node_stack)) elements:"
        # for deepestNode in node_stack
        #     displayBranchingNode(deepestNode)
        # end        
        
        ## Strategy 1. Depth first search
        # node = pop!(node_stack)
 
        ## Strategy 2. Select from deepest nodes
        # deepest_nodes = Vector{BranchingNode}()
        # max_depth = 0
        # for (_, value) in enumerate(node_stack) 
        #     if value.branchingInfo.depth == max_depth
        #         push!(deepest_nodes, value)
        #     elseif value.branchingInfo.depth > max_depth
        #         max_depth = value.branchingInfo.depth
        #         deepest_nodes = Vector{BranchingNode}()
        #         push!(deepest_nodes, value)
        #     end
        # end
        # println("")
        # @info "List of deepest nodes with $(length(deepest_nodes)) elements: "
        # for deepestNode in deepest_nodes
        #     displayBranchingNode(deepestNode)
        # end
        # node = deepest_nodes[1]
        # if length(deepest_nodes) > 1
        #     for (_, deepest_node) in enumerate(deepest_nodes[2:end])
        #         # println(node.cgLowerBound ,"  ", deepest_node.cgLowerBound)
        #         node = nodeSelection(node, deepest_node)
        #     end
        # end
        #endregion

        ## Strategy 3. Select node with lowest score
        node = node_stack[1]
        score = (node.gradientLB + node.gradientFS)/node.branchingInfo.depth
        for node_iter in node_stack 
            score_iter = (node_iter.gradientLB + node_iter.gradientFS)/(1+log(node_iter.branchingInfo.depth))
            if score_iter < score
                node = node_iter
                score = score_iter
            end
        end

        println("")
        @info "Display selected node:"
        displayBranchingNode(node)
        deleteat!(node_stack, findfirst(==(node), node_stack))
        println("")

        # @info "Node stack contains $(length(node_stack)) elements:"
        # for deepestNode in node_stack
        #     displayBranchingNode(deepestNode)
        # end
        # println("")
    
        # @info "Display branching rule:"
        # displayBranchingRule(node.branchingInfo)
        if node.cgLowerBound > upperBound
            @info "$(round(node.cgLowerBound, digits=2)), Exceed Upper Bound, prune"
        elseif node.isLeaf
            @info "Leaf Node already"
        else
        ## start branching
            # println("   Total number of 2e routes: ", sum(node.y_value) , ", lb of cg = $(round(node.cgLowerBound,digits=2))")
            # for (_, y) in enumerate([r for r in 1:length(node.y_value) if 0 < node.y_value[r]]) 
            #     println("   $(node.routes_pool[y].sequence)  $(round(node.y_value[y],digits=2))")
            # end

            ## BranchAndPrice
            result = branchingStrategy(node.y_value, node.routes_pool, node.branchingInfo)
            if !isnothing(result)
                leftBranchingNode = createBranchingNode(route_1e, node.routes_pool, result[1], node.cgLowerBound, node.fractionalScore, num_iter_sp, current_id + 1, node.id)
                rightBranchingNode = createBranchingNode(route_1e, node.routes_pool, result[2], node.cgLowerBound, node.fractionalScore, num_iter_sp, current_id + 2, node.id)
                current_id += 2
                if !isnothing(leftBranchingNode)
                    if leftBranchingNode.branchingInfo.depth > deepest_level
                        global deepest_level = leftBranchingNode.branchingInfo.depth
                    end
                    push!(node_stack, leftBranchingNode)
                end
                if !isnothing(rightBranchingNode)
                    if rightBranchingNode.branchingInfo.depth > deepest_level
                        global deepest_level = rightBranchingNode.branchingInfo.depth
                    end
                    push!(node_stack, rightBranchingNode)
                end
                
            else
                println("No branching decision made")
            end

            # println(test)
        end        
 
        println("Branching stack contains now $(length(node_stack)) nodes, current upper bound is $(round(upperBound,digits=2))")
        # for node in node_stack 
        #     displayBranchingNode(node)
        # end
        num_iter_sp += 1    
    end
end


function branchingStrategy(y, routes_pool, branchingInfo::BranchingInfo)

    left_branch = deepcopy(branchingInfo)
    right_branch = deepcopy(branchingInfo)

    ## Case A: total number of 2e route is fractional
    if !(abs(sum(y)-round(sum(y)))<1e-8)
        @info "Branch on total number of 2e routes:  $(floor(sum(y))), $(ceil(sum(y)))"

        push!(left_branch.lower_bound_number_2e_routes, ceil(sum(y)))
        push!(right_branch.upper_bound_number_2e_routes, floor(sum(y)))
        left_branch.depth += 1
        right_branch.depth += 1

        return left_branch, right_branch
    end

    ## Case B: reversed routes exist
    reversed_route = checkExistanceReversedRoute(sort([r for r in 1:length(y) if 0 < y[r] < 1], by = r -> y[r] * (1 - y[r]), rev = true), routes_pool)
    if !isnothing(reversed_route)
        return branchOnReverseRoute(branchingInfo, reversed_route)
    end

    # Case C: combination of customers
    result = branchOnCombinationParkingCustomer(branchingInfo, y, routes_pool)
    if !isnothing(result)
        return result
    end

    # ## Case D: combination of customers
    # result = branchOnArc(branchingInfo, y, routes_pool)
    # return result

end