include("branchingStrategies.jl")
include("Utiles.jl")
include("columnGeneration.jl")


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

function solve_MILP_model(route_1e::Route)
    selected_parkings = getServedParking1eRoute(route_1e)

    #region : construct model
    milpModel = Model(CPLEX.Optimizer)
    routes_2e_pool = Vector{Route}()
    for route in routes_2e 
        if route.sequence[1] in selected_parkings && route.sequence[end] in selected_parkings
            push!(routes_2e_pool, route)
        end
    end

    set_silent(milpModel)

    @variable(milpModel, y[1:length(routes_2e_pool)], Bin)

    @constraint(milpModel, [p in selected_parkings], sum(y[idx]*r.b2out[p] for (idx,r) in enumerate(routes_2e_pool))<=nb_vehicle_per_satellite)
    @constraint(milpModel, [i in customers], sum(y[idx]*r.a[i] for (idx,r) in enumerate(routes_2e_pool)) >=1)
    @constraint(milpModel, [p in selected_parkings], sum(y[idx]*r.b2in[p] for (idx,r) in enumerate(routes_2e_pool))== sum(y[idx]*r.b2out[p] for (idx,r) in enumerate(routes_2e_pool)))
    @constraint(milpModel, [p in selected_parkings], sum(y[idx]*r.load*r.b2out[p] for (idx,r) in enumerate(routes_2e_pool))<= capacity_microhub)

    @objective(milpModel, Min, sum(y[idx]*r.cost for (idx,r) in enumerate(routes_2e_pool)))
    #endregion

    optimize!(milpModel)

    println("MILP Result:\n",round(objective_value(milpModel)+route_1e.cost,digits=2))
    println(route_1e.sequence, "    $(round(route_1e.cost, digits=2))")
    for (idx,r) in enumerate(routes_2e_pool) 
        if value(y[idx]) != 0
            println(r.sequence, "   $(round(r.cost, digits=2))")
        end
    end

    if objective_value(milpModel)+route_1e.cost < upperBound
        global upperBound = objective_value(milpModel)+route_1e.cost
        global optimalSolution = Vector{Route}()

        push!(optimalSolution, route_1e)
        for (idx,r) in enumerate(routes_2e_pool) 
            if value(y[idx]) != 0
        #         println(r.sequence, "   $(round(route_1e.cost, digits=2))")
                push!(optimalSolution, r)
            end
        end
    end
end

function createRootNode(route_1e::Route)
    root_branch_info = BranchingInfo(Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Int}(), Set{Int}(), Set{Int}(), Set{Int}(),Set{Route}(),Set{Route}(), 0)
    root_branch_info.forbidden_parkings = setdiff(Set(satellites), getServedParking1eRoute(route_1e))
        
    routes_2e_pool = filter_2e_routes(root_branch_info, routes_2e)

    @info "--------Start column generation for root node of $(route_1e.sequence)--------"
    println("\n--------Start column generation for root node of $(route_1e.sequence)--------")
    println("Current global upper bound = $upperBound")

    result = solveColumnGeneration(route_1e, routes_2e_pool, root_branch_info)

    if !isnothing(result)
        routes_2e_pool = result[1]
        y_value = result[2]
        lpObjValue = result[3]
        println("lp value of column generation : $(round(lpObjValue, digits=2))")
        if checkExistanceDummyRoute(y_value, routes_2e_pool)
        ## Dummy route used at the end of column generation, branch can be pruned
            @info "Dummy routes used, exceed upper bound, prune"
            return nothing

        elseif isempty([r for r in 1:length(y_value) if 0 < y_value[r] < 1])
        ## Integer solution found, note as a leaf node
            @info "Integer Solution Found  $lpObjValue"
            println("Integer Solution Found  $lpObjValue")
            isLeaf = true
            if lpObjValue < upperBound
                global  upperBound = lpObjValue
                global optimalSolution = Vector{Route}()
                global optimal_found_in = 0
                # global optimal_found_iteration = num_iter_sp
                @info "Update upper bound"
                println("Update upper bound")
                
                push!(optimalSolution, route_1e)
                for (_, y) in enumerate([r for r in 1:length(y_value) if y_value[r]==1]) 
                    println(routes_2e_pool[y].sequence, "   ", round(routes_2e_pool[y].cost, digits=2))
                    push!(optimalSolution, routes_2e_pool[y])
                end             
            end
            return nothing
        else
            if lpObjValue >= upperBound
                @info "lp result exceed global upper bound, prune"
                println("lp result exceed global upper bound, prune")
                return nothing
            else
            ## * Create a node
                # println("Column generation lower bound is $(round(result[5],digits=2))")
                fractionalScore = 0
                for value in y_value
                    fractionalScore += value <= 0.5 ? value : 1 - value
                end
                gradientLB = lpObjValue
                gradientFS = fractionalScore
                isLeaf = false
                solve_MILP_model(route_1e)
                branchingNode = BranchingNode(root_branch_info, lpObjValue, y_value, isLeaf, fractionalScore, routes_2e_pool, gradientLB, gradientFS, 0,0)
                return branchingNode        
            end
       
        end
    else
        @info "RLMP infeasible, prune "
        return nothing
    end
end

function createBranchingNode(route_1e, routes_2e_pool, branchingInfo, cgLowerBound, fs, num_iter_sp, id, parent_id)
## given a branchingInfo, transform it into a branchingNode structure (add cg result)
    
    execution_time = @elapsed begin
        routes_2e_pool = filter_2e_routes(branchingInfo, routes_2e_pool)
    end
    global filtering_time += execution_time
    
    # TODO : Before start column generation, check branching rules conflic
    @info "Start column generation for node N_$id, parent node N_$parent_id, depth $(branchingInfo.depth)"
    println("Start column generation for node N_$id, parent node N_$parent_id, depth $(branchingInfo.depth)")
    displayBranchingRule(branchingInfo)
    # result we expect from a column generation: y values and index
    execution_time = @elapsed begin
        result = solveColumnGeneration(route_1e, routes_2e_pool, branchingInfo)
    end
    global execution_time_column_generation += execution_time

    if !isnothing(result)
        routes_2e_pool = result[1]
        y_value = result[2]
        lpObjValue = result[3]

        if checkExistanceDummyRoute(y_value, routes_2e_pool)
        ## Dummy route used at the end of column generation, branch can be pruned
            @info "Dummy routes used, exceed upper bound, prune"
            return nothing

        elseif isempty([r for r in 1:length(y_value) if 0 < y_value[r] < 1])
        ## Integer solution found, note as a leaf node
            @info "Integer Solution Found  $lpObjValue"
            println("Integer Solution Found  $lpObjValue")
            isLeaf = true
            if lpObjValue < upperBound
                global upperBound = lpObjValue
                global optimalSolution = Vector{Route}()
                global optimal_found_in = branchingInfo.depth
                @info "Update upper bound"
                println("Update upper bound")
                
                push!(optimalSolution, route_1e)
                for (_, y) in enumerate([r for r in 1:length(y_value) if y_value[r]==1]) 
                    println(routes_2e_pool[y].sequence, "   ", round(routes_2e_pool[y].cost, digits=2))
                    push!(optimalSolution, routes_2e_pool[y])
                end             
            end
            return nothing
        else
            if lpObjValue >= upperBound
                @info "lp result exceed global upper bound, prune"
                println("lp result exceed global upper bound, prune")
                return nothing
            else
            ## Create a child node
                println("value of column generation : $(round(lpObjValue, digits=2))")
                fractionalScore = sum(value <= 0.5 ? value : 1 - value for value in y_value)
                gradientLB = - lpObjValue + cgLowerBound
                gradientFS = fs - fractionalScore
                isLeaf = false
                branchingNode = BranchingNode(branchingInfo, lpObjValue, y_value, isLeaf, fractionalScore, routes_2e_pool, gradientLB, gradientFS, id, parent_id)
                return branchingNode
            end
        end
    else
        @info "RLMP infeasible, prune "
        return nothing
    end
end

function branchAndPriceWithScore(route_1e::Route)

    execution_time = @elapsed begin
        root_node = createRootNode(route_1e)
        if isnothing(root_node)
            return
        end
        node_stack = [root_node]

        println("Branching stack contains now $(length(node_stack)) nodes, current upper bound is $(round(upperBound,digits=2))")
    end
    global execution_time_root_node += execution_time

    num_iter_sp = 1
    current_id = 0
    while !isempty(node_stack) # && num_iter_sp < 1
        println("\n================Iteration $num_iter_sp of B&P for SP$num_iter_global $(route_1e.sequence)================")

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
        score = (node.gradientLB + node.gradientFS)/(1+log(node.branchingInfo.depth))
        for node_iter in node_stack 
            score_iter = (node_iter.gradientLB + node_iter.gradientFS)/(1+log(node_iter.branchingInfo.depth))
            if score_iter < score
                node = node_iter
                score = score_iter
            end
        end

        println("")
        @info "Display selected node $(node.id): from $(length(node_stack)) nodes"
        println("Display selected node $(node.id): from $(length(node_stack)) nodes")
        displayBranchingNode(node)
        deleteat!(node_stack, findfirst(==(node), node_stack))
        println("")

        
        if node.cgLowerBound > upperBound
            @info "$(round(node.cgLowerBound, digits=2)), Exceed Upper Bound, prune"
            println("$(round(node.cgLowerBound, digits=2)), Exceed Upper Bound, prune")
        elseif node.isLeaf
            println("Leaf Node Already")
            @info "Leaf Node already"
        else
        ## start branching
            execution_time = @elapsed begin
                # TODO : if only one active parking: skip strategy parking-customer
                branching_decision = branchingStrategy(node.y_value, node.routes_pool, node.branchingInfo)
            end  
            global execution_time_branching += execution_time
                
            execution_time = @elapsed begin
                if !isnothing(branching_decision)
                    leftBranchingNode = createBranchingNode(route_1e, node.routes_pool, branching_decision[1], node.cgLowerBound, node.fractionalScore, num_iter_sp, current_id + 1, node.id)
                    # println()
                    rightBranchingNode = createBranchingNode(route_1e, node.routes_pool, branching_decision[2], node.cgLowerBound, node.fractionalScore, num_iter_sp, current_id + 2, node.id)
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
            end
            global execution_time_child_node += execution_time
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
        println("Branch on total number of 2e routes:  $(floor(sum(y))), $(ceil(sum(y)))")
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