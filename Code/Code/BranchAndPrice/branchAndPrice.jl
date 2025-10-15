include("branchingStrategies.jl")
include("Utiles.jl")
include("columnGeneration.jl")

function select_node_from_tree(node_stack)
    node = node_stack[1]
    score = (node.gradientLB + node.gradientFS) / node.branchingInfo.depth
    for node_iter in node_stack 
        score_iter = (node_iter.gradientLB + node_iter.gradientFS)/(1+log(node_iter.branchingInfo.depth))
        if score_iter < score
            node = node_iter
            score = score_iter
        end
    end

    @info "Display selected node $(node.id), parent node $(node.parent_id): from $(length(node_stack)) nodes"
    println("Display selected node $(node.id), parent node $(node.parent_id): from $(length(node_stack)) nodes")
    displayBranchingNode(node)

    deleteat!(node_stack, findfirst(==(node), node_stack))
    return node#, node_stack
end

function filter_2e_routes(branchingInfo::BranchingInfo, routes::Vector{Int})
    result = Vector{Int}()
    routes_to_delete = Vector{Int}()
    # displayBranchingRule(branchingInfo)
    
    for (idx, route_id) in enumerate(routes)
        valide = true
        route = routes_2e[route_id]
        if route.sequence[1] in branchingInfo.forbidden_parkings || route.sequence[end] in branchingInfo.forbidden_parkings
            valide = false
        end
        #region
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
        #endregion

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
            push!(result, route_id)
        else
            push!(routes_to_delete, route_id)
        end
        # print("$(route.sequence)   $valide")
    end

    return result, routes_to_delete
end

#region : solve column generation
function solve_column_generation(route_1e, routes_2e_pool, 
                                 model, y_vars, sync, custVisit, number2evfixe, maxVolumnMM, globalLowerBound, globalUpperBound,
                                 branchingInfo::BranchingInfo, cgLB, fs, id, parent_id)
    # * Column generation process:
    # *     - 1. solve formulation
    # *     - 2. get dual multiplier
    # *     - 3. execute labelling algorithm
    # *     - 4. check existance new routes

    selected_parkings = getServedParking1eRoute(route_1e)
    
    # @info "Start column generation for node N_$id, parent node N_$parent_id, depth $(branchingInfo.depth)"
    # println("Start column generation for node N_$id, parent node N_$parent_id, depth $(branchingInfo.depth)")
    
    num_iter_cg = 1
    while true # num_iter_cg < 16
        println("-------------Iter CG $num_iter_cg-------------")
        # * 1. solve formulation
        execution_time_lp = @elapsed begin
        optimize!(model)
        end
        global execution_time_rmp += execution_time_lp
        if has_values(model)
            # execution_time_op = @elapsed begin
                lpObjValue = objective_value(model) + route_1e.cost
                #region : display lp result
                println("LP Objective Value = $(round(lpObjValue, digits=2)),   sum y = $(round(sum(value.(values(y_vars))), digits=2))")
                # println("x$(route_1e.sequence),  $(round(route_1e.cost, digits=2))")
                # for (rid, y) in y_vars
                #     if value(y)!=0
                #         println("y$(routes_2e[rid].sequence) = ", round(value(y),digits=2), "  ", round(routes_2e[rid].cost, digits=2))
                #     end
                # end
                #endregion

            # end

            # global execution_time_output += execution_time_op
            
            # * 2. get dual multiplier
            #region : retrieve and display dual multiplier
            π1 = vcat(0, abs.(shadow_price.(sync)))
            π2 = vcat(zeros(1+length(satellites)), abs.(shadow_price.(custVisit)))
            π3 = vcat(0, abs.(shadow_price.(number2evfixe)))
            π4 = vcat(0, abs.(shadow_price.(maxVolumnMM)))

            # println("π1= $(round.(π1,digits=2))")
            # println("π2= $(round.(π2[customers],digits=2))")
            # println("π3= $(round.(π3,digits=2))")
            # println("π4= $(round.(π4,digits=2))")
            #endregion

            # * 3. execute labelling algorithm
            execution_time_p = @elapsed begin
                routes_2e_pool, new_routes_from = pricing(selected_parkings, collect(1:length(routes_2e)), π1, π2, π3, π4, branchingInfo)
            end
            global execution_time_pricing += execution_time_p

            # * 4. check existance new routes
            if new_routes_from == false
                break
            else
                # execution_time = @elapsed begin
                    for route in routes_2e_pool[new_routes_from:end]
                        add_2eroute!(model, route,
                                     sync, custVisit, number2evfixe, maxVolumnMM, 
                                     globalLowerBound, globalUpperBound, y_vars)
                    end
                # end    
            end
        else
            println("No feasible solution for LMP")
            return nothing
        end
        num_iter_cg += 1
    end

    if objective_value(model)+route_1e.cost > upperBound
        @info "Exceed Upper Bound, prune"
        println("Exceed Upper Bound, prune")
        return nothing
    end

    y_values = [value(y_vars[k]) for k in sort(collect(keys(y_vars)))]
    # Define isLeaf
    # Define fractional score
    fractionalScore = 0
    if isempty([r for r in 1:length(y_values) if 0 < y_values[r] < 1])
        @info "Integer solution found"
        println("Integer solution found")
        isLeaf = true
        if objective_value(model)+route_1e.cost < upperBound
            @info "Update upper bound"
            println("Update upper bound")

            global upperBound = objective_value(model)+route_1e.cost
            global optimalSolution = Vector{Route}()

            push!(optimalSolution, route_1e)
            for (_, y) in enumerate([r for r in 1:length(y_values) if y_values[r]==1]) 
                println(routes_2e[y].sequence, "   ", round(routes_2e[y].cost, digits=2))
                push!(optimalSolution, routes_2e[y])
            end
        end

        return nothing
    else
        isLeaf = false
        for value in y_values
            if value <= 0.5
                fractionalScore += value
            else
                fractionalScore += 1 - value
            end
        end
    end

    # Define gradientLB
    gradientLB = objective_value(model) + route_1e.cost - cgLB
    # Define gradientFS
    gradientFS = fractionalScore - fs
    result = BranchingNode(branchingInfo,
                        objective_value(model)+route_1e.cost,
                        y_values,
                        routes_2e_pool,
                        isLeaf,
                        fractionalScore,
                        gradientLB,
                        gradientFS,
                        id,
                        parent_id)


    return result
end
#endregion
function update_optimal_solution(lpValue, route_1e, routes_2e_pool, y_value)
    global upperBound= lpObjValue

    global optimalSolution = Vector{Route}()

    @info "Update upper bound"
    println("Update upper bound")

    push!(optimalSolution, route_1e)
    for (_, y) in enumerate([r for r in 1:length(y_value) if y_value[r]==1]) 
        println(routes_2e_pool[y].sequence, "   ", round(routes_2e_pool[y].cost, digits=2))
        push!(optimalSolution, routes_2e_pool[y])
    end
end
#endregion
#region : solve a root node
function solve_root_node(route_1e::Route)

    root_node_branching_info = BranchingInfo(Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Int}(), Set{Int}(), Set{Int}(), Set{Int}(),Set{Route}(),Set{Route}(), 0)
    root_node_branching_info.forbidden_parkings = setdiff(Set(satellites), getServedParking1eRoute(route_1e))
    
    #region : create model
    execution_time = @elapsed begin
        model = Model(CPLEX.Optimizer)
        set_silent(model)
        # set_optimizer_attribute(model, "CPXPARAM_Threads", 1)
        # set_optimizer_attribute(model, "CPXPARAM_MIP_Display", 0)

        y_vars = Dict{Int, VariableRef}()

        @objective(model, Min, 0.0)

        sync = Vector{ConstraintRef}(undef, length(satellites))
        for (k,_) in enumerate(satellites)
            # println("$k $s")
            sync[k] = @constraint(model, -nb_vehicle_per_satellite <= 0.0)
        end

        custVisit = Vector{ConstraintRef}(undef, length(customers))
        for (k,_) in enumerate(customers) 
            custVisit[k] = @constraint(model, 1.0 <= 0.0)
        end

        number2evfixe = Vector{ConstraintRef}(undef, length(satellites))
        for (k,_) in enumerate(satellites)
            number2evfixe[k] = @constraint(model, 0.0 == 0.0)
        end

        maxVolumnMM = Vector{ConstraintRef}(undef, length(satellites))
        for (k,_) in enumerate(satellites) 
            maxVolumnMM[k] = @constraint(model, -capacity_microhub <= 0.0)
        end

        global lower_bound_2e_routes = minimum_2e_vehicle_required
        global upper_bound_2e_routes = nb_parking * nb_vehicle_per_satellite

        globalLowerBound = @constraint(model, 0 <= -minimum_2e_vehicle_required) 
        globalUpperBound = @constraint(model, 0 <= upper_bound_2e_routes)
    end
    global execution_time_build_model += execution_time
    #endregion

    #region : initial columns
    execution_time = @elapsed begin
        _, columns_to_be_deleted = filter_2e_routes(root_node_branching_info, collect(1:length(routes_2e)))
    end
    global execution_time_filtering += execution_time

    execution_time = @elapsed begin
        for (route,_) in enumerate(routes_2e)
            add_2eroute!(model, route, sync, custVisit, number2evfixe, 
                        maxVolumnMM, globalLowerBound, globalUpperBound, y_vars)
        end

        for idx in columns_to_be_deleted
            if haskey(y_vars, idx)
                y = y_vars[idx]
                JuMP.set_upper_bound(y, 0.0)
                JuMP.set_lower_bound(y, 0.0)
            end
        end
    end
    global execution_time_build_model += execution_time
    #endregion

    execution_time = @elapsed begin
        root_node = solve_column_generation(route_1e, routes_2e, 
                                            model, y_vars, sync, custVisit, number2evfixe, maxVolumnMM,globalLowerBound,globalUpperBound,
                                            root_node_branching_info, 0,0,0,0)
    end
    global execution_time_column_generation += execution_time

    if !isnothing(root_node)
        if root_node.isLeaf
            println("Integer solution found in root node")
            if root_node.cgLowerBound < upperBound
                println("Update upper bound")
                global upperBound = root_node.cgLowerBound
                global optimalSolution = Vector{Route}()

                push!(optimalSolution, route_1e)
                for (_, y) in enumerate([r for r in 1:length(root_node.y_value) if root_node.y_value[r]==1]) 
                    push!(optimalSolution, routes_2e[y])
                end
            end
            return nothing
        else
            # TODO : if lp result of a root node exceed UB, prune subproblem
            node_stack = [root_node]
            solve_MILP_model(route_1e)    

            println("Branching stack contains now $(length(node_stack)) nodes, current upper bound is $(round(upperBound,digits=2))")  
            return node_stack, model, y_vars, sync, custVisit, number2evfixe, maxVolumnMM,globalLowerBound,globalUpperBound
        end       
    else
        return nothing 
    end


end
#endregion
#region : solve a child node
function solve_child_node(route_1e, model, y_vars, sync, custVisit, number2evfixe, maxVolumnMM, 
                          globalLowerBound, globalUpperBound, node::BranchingNode, branching_decision::BranchingInfo, id)
    println("")
    @info "Solve child node $id"
    println("Solve child node $id")
    displayBranchingRule(branching_decision)
    # * Instead of copying the model, just filter out routes and set bounds to 0
    execution_time = @elapsed begin
        routes_2e_pool, columns_to_be_deleted = filter_2e_routes(branching_decision, collect(1:length(routes_2e)))
    end
    global execution_time_filtering += execution_time

    execution_time = @elapsed begin
        # * Set bounds to 0 for deleted routes instead of actually deleting them
        for route_idx in columns_to_be_deleted
            if haskey(y_vars, route_idx)
                y = y_vars[route_idx]
                JuMP.set_upper_bound(y, 0.0)
                JuMP.set_lower_bound(y, 0.0)
            end
        end

        # * Global Lower bound
        if !isempty(branching_decision.lower_bound_number_2e_routes)
            lower_bound_number_2e_routes = maximum(branching_decision.lower_bound_number_2e_routes)
            set_normalized_rhs(globalLowerBound, -lower_bound_number_2e_routes)
        else
            set_normalized_rhs(globalLowerBound, -lower_bound_2e_routes)
        end
        
        # * Global Upper Bound
        if !isempty(branching_decision.upper_bound_number_2e_routes)
            upper_bound_number_2e_routes = minimum(branching_decision.upper_bound_number_2e_routes)
            set_normalized_rhs(globalUpperBound, upper_bound_number_2e_routes)
        else
            set_normalized_rhs(globalUpperBound, upper_bound_2e_routes)
        end
    end
    global execution_time_set_bound += execution_time
    global execution_time_build_model += execution_time
    
    execution_time = @elapsed begin
        child_node = solve_column_generation(route_1e, routes_2e_pool, model, y_vars,
                                            sync, custVisit, number2evfixe, maxVolumnMM, 
                                            globalLowerBound, globalUpperBound,
                                            branching_decision, node.cgLowerBound, node.fractionalScore, id, node.id) 

    end
    global execution_time_column_generation += execution_time

    execution_time = @elapsed begin
        # *  Reset bounds after solving
        for route_idx in columns_to_be_deleted
            if haskey(y_vars, route_idx)
                y = y_vars[route_idx]
                JuMP.set_upper_bound(y, 1.0)
                JuMP.set_lower_bound(y, 0.0)
            end
        end
    end
    global execution_time_set_bound += execution_time
    global execution_time_build_model += execution_time

    return child_node
end
#endregion
#region : solve branch and price 2e subproblem
function solve_branch_and_price_2e_subproblem(route_1e::Route)
    #region
    # * Each BranchNode contains :
    # *    - branchingInfo
    # *    - column generation result : rmp obj value
    # *    - column generation result : y value
    # *    - column generation result : routes pool
    # *    - whether it is a leaf node (integer)
    # *    - fractional score
    # *    - gradient lower bound
    # *    - gradient fractional score
    # *    - node id
    # *    - parent node id

    # * 1. Obtain root node : build model
    # * 2. Branch and Price chide nodes
        # * 2.1 Select a node from search tree
        # * 2.2 Obtain Branching strategy (delete columns)
        # * 2.3 Column generation two child nodes (add columns)
    #endregion

    # * 1. Obtain root node : build model
    execution_time = @elapsed begin
        println("\n================Iteration 0 of B&P for SP$num_iter_global $(route_1e.sequence) parkings$([r for r in getServedParking1eRoute(route_1e)])================")
        result_root_node = solve_root_node(route_1e)
    end
    global execution_time_root_node += execution_time

    if isnothing(result_root_node)
        return
    else
        #region : pass parameters
        node_stack = result_root_node[1]
        model = result_root_node[2]
        y_vars = result_root_node[3]
        sync = result_root_node[4]
        custVisit = result_root_node[5]
        number2evfixe = result_root_node[6]
        maxVolumnMM = result_root_node[7]
        globalLowerBound = result_root_node[8]
        globalUpperBound = result_root_node[9]
        #endregion

        # * 2. Branch and Price child nodes
        num_iter_sp = 1
        current_node_id = 0
        while !isempty(node_stack) # && num_iter_sp < 51
            println("\n================Iteration $num_iter_sp of B&P for SP$num_iter_global $(route_1e.sequence) parkings$([r for r in getServedParking1eRoute(route_1e)])================")
            
            # * 2.1 Select a node from search tree
            node = select_node_from_tree(node_stack)
            
            if node.cgLowerBound > upperBound
                @info "$(round(node.cgLowerBound, digits=2)), Exceed Upper Bound, prune"
                println("$(round(node.cgLowerBound, digits=2)), Exceed Upper Bound, prune")
            else
                # * 2.2 Obtain branching strategy
                execution_time = @elapsed begin
                    branching_decisions = branchingStrategy(node.y_value, route_1e, node.routes_pool, node.branchingInfo)
                end
                global execution_time_branching += execution_time

                execution_time = @elapsed begin
                    left_child_node  = solve_child_node(route_1e, model, y_vars, sync, custVisit, number2evfixe, 
                                                        maxVolumnMM, globalLowerBound, globalUpperBound,
                                                        node, branching_decisions[1], current_node_id + 1)
                    right_child_node = solve_child_node(route_1e, model, y_vars, sync, custVisit, number2evfixe, 
                                                        maxVolumnMM, globalLowerBound, globalUpperBound,
                                                        node, branching_decisions[2], current_node_id + 2)
                    current_node_id += 2
                    if !isnothing(left_child_node)
                        push!(node_stack, left_child_node)
                    end
                    if !isnothing(right_child_node)
                        push!(node_stack, right_child_node)
                    end
                end
                global execution_time_child_node += execution_time
            end
            num_iter_sp += 1
        end
    end

end
#endregion
function solve_MILP_model(route_1e)

    selected_parkings = getServedParking1eRoute(route_1e)

    milpModel = Model(CPLEX.Optimizer)
    routes_2e_pool = Vector{Route}()
    for route in routes_2e 
        if route.sequence[1] in selected_parkings && route.sequence[end] in selected_parkings
            push!(routes_2e_pool, route)
        end
    end
    # println(length(routes_2e_pool))

    set_silent(milpModel)

    @variable(milpModel, y[1:length(routes_2e_pool)], Bin)

    @constraint(milpModel, [p in selected_parkings], sum(y[idx]*r.b2out[p] for (idx,r) in enumerate(routes_2e_pool))<=nb_vehicle_per_satellite)
    @constraint(milpModel, [i in customers], sum(y[idx]*r.a[i] for (idx,r) in enumerate(routes_2e_pool)) >=1)
    @constraint(milpModel, [p in selected_parkings], sum(y[idx]*r.b2in[p] for (idx,r) in enumerate(routes_2e_pool))== sum(y[idx]*r.b2out[p] for (idx,r) in enumerate(routes_2e_pool)))
    @constraint(milpModel, [p in selected_parkings], sum(y[idx]*r.load*r.b2out[p] for (idx,r) in enumerate(routes_2e_pool))<= capacity_microhub)

    @objective(milpModel, Min, sum(y[idx]*r.cost for (idx,r) in enumerate(routes_2e_pool)))

    optimize!(milpModel)

    println("MILP Result:\n",round(objective_value(milpModel)+route_1e.cost,digits=2))
    println(route_1e.sequence, "    $(round(route_1e.cost, digits=2))")
    for (idx,r) in enumerate(routes_2e_pool) 
        if value(y[idx]) != 0
            println(r.sequence, "   $(round(r.cost, digits=2))")
        end
    end

    if objective_value(milpModel)+route_1e.cost < upperBound
        global upperBound= objective_value(milpModel)+route_1e.cost
        global optimalSolution = Vector{Route}()

        # @info "Update upper bound"
        # println("Update upper bound")

        push!(optimalSolution, route_1e)
        for (idx,r) in enumerate(routes_2e_pool) 
            if value(y[idx]) != 0
        #         println(r.sequence, "   $(round(route_1e.cost, digits=2))")
                push!(optimalSolution, r)
            end
        end
    end
end

function branchingStrategy(y, route_1e, routes_pool, branchingInfo::BranchingInfo)

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
    result = branchOnCombinationParkingCustomer(route_1e, branchingInfo, y, routes_pool)
    if !isnothing(result)
        return result
    end

    # ## Case D: combination of customers
    # result = branchOnArc(branchingInfo, y, routes_pool)
    # return result

end