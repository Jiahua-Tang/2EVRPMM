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
    println("Display selected node $(node.id), parent node $(node.parent_id): from $(length(node_stack)) nodes, current upper bound = $(round(upperBound, digits=2))")
    displayBranchingNode(node)

    deleteat!(node_stack, findfirst(==(node), node_stack))
    return node
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

function preparation_branch_and_price()
    #region B&P: Prep
    global num_iter_global = 1
    global upperBound = Inf
    global optimalSolution = nothing

    global optimal_found_iteration = 0
    global execution_time_total = 0
    global execution_time_branchandprice = 0
    global execution_time_root_node = 0
    global execution_time_child_node = 0
    global execution_time_branching = 0
    global execution_time_column_generation = 0
    global execution_time_set_bound = 0
    global execution_time_test = 0
    global execution_time_pricing = 0
    global execution_time_output = 0
    global execution_time_subproblem = 0
    global execution_time_build_model = 0
    global execution_time_add_columns = 0
    global execution_time_rmp = 0
    global execution_time_filtering = 0
    global deepest_level = 0
    global optimal_found_in = 0
    #endregion

    generate2eInitialRoutes()
    global neighbours = get_neighbours_optimized(10)

    execution_time = @elapsed begin
        lrp_subproblems = get_sorted_2e_subproblems(5)
    end
    println("time to get sorted 2e subproblem = $(round(execution_time, digits=2))s")

    return lrp_subproblems
end

function solve_virtual_root_node()
    println("\nSolve virtual root node")
    root_node_branching_info = BranchingInfo(Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Int}(), Set{Int}(), Set{Int}(), Set{Int}(), 0)
    
    #region : create model
    execution_time = @elapsed begin
        global model = Model(CPLEX.Optimizer)
        set_silent(model)
        # set_optimizer_attribute(model, "CPXPARAM_Threads", 1)
        # set_optimizer_attribute(model, "CPXPARAM_MIP_Display", 0)

        global y_vars = Dict{Int, VariableRef}()

        @objective(model, Min, 0.0)

        global sync = Vector{ConstraintRef}(undef, length(satellites))
        for (k,_) in enumerate(satellites)
            # println("$k $s")
            sync[k] = @constraint(model, -nb_vehicle_per_satellite <= 0.0)
        end

        global custVisit = Vector{ConstraintRef}(undef, length(customers))
        for (k,_) in enumerate(customers) 
            custVisit[k] = @constraint(model, 1.0 <= 0.0)
        end

        global number2evfixe = Vector{ConstraintRef}(undef, length(satellites))
        for (k,_) in enumerate(satellites)
            number2evfixe[k] = @constraint(model, 0.0 == 0.0)
        end

        global maxVolumnMM = Vector{ConstraintRef}(undef, length(satellites))
        for (k,_) in enumerate(satellites) 
            maxVolumnMM[k] = @constraint(model, -capacity_microhub <= 0.0)
        end

        global lower_bound_2e_routes = minimum_2e_vehicle_required
        global upper_bound_2e_routes = nb_parking * nb_vehicle_per_satellite

        global globalLowerBound = @constraint(model, 0 <= -minimum_2e_vehicle_required) 
        global globalUpperBound = @constraint(model, 0 <= upper_bound_2e_routes)
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
            add_2eroute!(route)
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

    root_node = solve_column_generation(generate1eRoute([1]), root_node_branching_info, 0,0,0,0)
end

#region : column generation
function solve_column_generation(route_1e, branchingInfo::BranchingInfo, cgLB, fs, id, parent_id)
    # * Column generation process:
    # *     - 1. solve formulation
    # *     - 2. get dual multiplier
    # *     - 3. execute labelling algorithm
    # *     - 4. check existence new routes

    if route_1e.sequence == [1]
        selected_parkings = collect(satellites)
    else
        selected_parkings = getServedParking1eRoute(route_1e)
    end
    
    num_iter_cg = 1
    is_virtual_root = (route_1e.sequence == [1])  # Check if called from virtual root node
    
    # Pre-allocate dual multiplier arrays to avoid reallocation each iteration
    n_satellites = length(satellites)
    n_customers = length(customers)
    π1 = Vector{Float64}(undef, n_satellites + 1)
    π2 = Vector{Float64}(undef, n_satellites + n_customers + 1)
    π3 = Vector{Float64}(undef, n_satellites + 1)
    π4 = Vector{Float64}(undef, n_satellites + 1)

    π1_stabilized = Vector{Float64}(undef, n_satellites + 1)
    π2_stabilized = Vector{Float64}(undef, n_satellites + n_customers + 1)
    π3_stabilized = Vector{Float64}(undef, n_satellites + 1)
    π4_stabilized = Vector{Float64}(undef, n_satellites + 1)
    
    while true # num_iter_cg < 2 # && true
        println("-------------Iter CG $num_iter_cg-------------")
        # * 1. solve formulation
        execution_time_lp = @elapsed begin
            optimize!(model)
        end
        global execution_time_rmp += execution_time_lp
        if has_values(model)
            # Cache objective value (used multiple times)
            obj_val = objective_value(model)
            lpObjValue = obj_val + route_1e.cost
            
            # * 2. get dual multiplier - optimized to avoid allocations
            #region : retrieve dual multiplier
            π1[1] = 0.0
            @inbounds for i in 1:n_satellites
                π1[i+1] = abs(shadow_price(sync[i]))
            end
            
            π2[1] = 0.0
            @inbounds for i in 1:n_satellites
                π2[i+1] = 0.0
            end
            @inbounds for i in 1:n_customers
                π2[n_satellites + i + 1] = abs(shadow_price(custVisit[i]))
            end
            
            π3[1] = 0.0
            @inbounds for i in 1:n_satellites
                π3[i+1] = abs(shadow_price(number2evfixe[i]))
            end
            
            π4[1] = 0.0
            @inbounds for i in 1:n_satellites
                π4[i+1] = abs(shadow_price(maxVolumnMM[i]))
            end
            #endregion

            #region : stabilization
            phi = 0
            if num_iter_cg <= 2
                π1_stabilized = π1
                π2_stabilized = π2
                π3_stabilized = π3
                π4_stabilized = π4
            else
                π1_stabilized = π1_stabilized * phi + π1 * (1-phi)
                π2_stabilized = π2_stabilized * phi + π2 * (1-phi)
                π3_stabilized = π3_stabilized * phi + π3 * (1-phi)
                π4_stabilized = π4_stabilized * phi + π4 * (1-phi)
            end
            #endregion

            # * 3. execute labelling algorithm
            # execution_time_p = @elapsed begin
                # routes_2e_pool, new_routes_from = 
                new_columns_found = pricing(selected_parkings, collect(1:length(routes_2e)), π1_stabilized, π2_stabilized, π3_stabilized, π4_stabilized, branchingInfo)
            # end
            # global execution_time_pricing += execution_time_p

            # * 4. check existence new routes
            # if new_routes_from == false
            #     break
            # else
            #     # Break after first iteration if this is the virtual root node (before adding new routes)
            #     if is_virtual_root && num_iter_cg >= 1
            #         println("Virtual root node: stopping after 1 iteration")
            #         break
            #     end
                
            #     # Add new routes
            #     @inbounds for i in new_routes_from:length(routes_2e_pool)
            #         add_2eroute!(routes_2e_pool[i])
            #     end
            # end
            if !new_columns_found
                break
            end
        else
            println("No feasible solution for LMP")
            return nothing
        end
        num_iter_cg += 1
    end

    # Virtual root node: just for generating routes, no need for final result
    if is_virtual_root
        println("Virtual root node: routes generated, returning without final result")
        for route in routes_2e
            println(route.sequence)
        end
        return nothing
    end

    # Cache objective value (used multiple times below)
    obj_val = objective_value(model)
    total_obj = obj_val + route_1e.cost
    
    if total_obj > upperBound
        @info "Exceed Upper Bound, prune"
        println("Exceed Upper Bound, prune")
        return nothing
    end

    # Extract y_values more efficiently - avoid sort and collect
    n_vars = length(y_vars)
    y_values = Vector{Float64}(undef, n_vars)
    sorted_keys = sort!(collect(keys(y_vars)))
    @inbounds for (idx, k) in enumerate(sorted_keys)
        y_values[idx] = value(y_vars[k])
        if y_values[idx] !=  0
            println(routes_2e[value(k)].sequence)
            println(round(y_values[idx],digits=2))
        end
    end
    
    # Check for integer solution and compute fractional score
    isLeaf = true
    fractionalScore = 0.0
    @inbounds for y_val in y_values
        if 0 < y_val < 1
            isLeaf = false
            if y_val <= 0.5
                fractionalScore += y_val
            else
                fractionalScore += 1 - y_val
            end
        end
    end
    
    if isLeaf
        @info "Integer solution found"
        println("Integer solution found")
        
        if total_obj < upperBound
            @info "Update upper bound"
            println("Update upper bound")

            global upperBound = total_obj
            println("Upper bound = $upperBound")
            global optimalSolution = Vector{Route}()
            sizehint!(optimalSolution, n_vars + 1)

            push!(optimalSolution, route_1e)
            println("$(route_1e.sequence)", "   ", round(route_1e.cost, digits=2))
            @inbounds for i in 1:length(y_values)
                if y_values[i] == 1.0
                    println(routes_2e[i].sequence, "   ", round(routes_2e[i].cost, digits=2))
                    push!(optimalSolution, routes_2e[i])
                end
            end
        end

        return nothing
    end

    println("LP result of column generation: $(round(total_obj, digits=2))")
    # Define gradientLB
    gradientLB = total_obj - cgLB
    # Define gradientFS
    gradientFS = fractionalScore - fs
    result = BranchingNode(branchingInfo,
                        total_obj,
                        y_values,
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
#region : root node
function solve_root_node(route_1e::Route)
    println("\nSolve root node of $(route_1e.sequence),   $(getServedParking1eRoute(route_1e))")
    root_node_branching_info = BranchingInfo(Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Int}(), Set{Int}(), Set{Int}(), Set{Int}(), 0)
    root_node_branching_info.forbidden_parkings = setdiff(Set(satellites), getServedParking1eRoute(route_1e))

    #region : initial columns
    execution_time = @elapsed begin
        columns_to_be_kept, columns_to_be_deleted = filter_2e_routes(root_node_branching_info, collect(1:length(routes_2e)))
    end
    global execution_time_filtering += execution_time

    execution_time = @elapsed begin
        # for (route,_) in enumerate(routes_2e)
        #     add_2eroute!(model, route, sync, custVisit, number2evfixe, 
        #                 maxVolumnMM, globalLowerBound, globalUpperBound, y_vars)
        # end

        for idx in columns_to_be_deleted
            if haskey(y_vars, idx)
                y = y_vars[idx]
                JuMP.set_upper_bound(y, 0.0)
                JuMP.set_lower_bound(y, 0.0)
            end
        end
        for idx in columns_to_be_kept
            if haskey(y_vars, idx)
                y = y_vars[idx]
                JuMP.set_upper_bound(y, 1.0)
                JuMP.set_lower_bound(y, 0.0)
            end
        end
    end
    global execution_time_build_model += execution_time
    #endregion

    execution_time = @elapsed begin
        root_node = solve_column_generation(route_1e, root_node_branching_info, 0,0,0,0)
    end
    global execution_time_column_generation += execution_time
    # println("execution time of column generation : $(round(execution_time, digits=2))s")

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
            return node_stack
        end       
    else
        return nothing 
    end


end

#endregion
#region : child node
function solve_child_node(route_1e, node::BranchingNode, branching_decision::BranchingInfo, id)
    println("")
    @info "Solve child node $id"
    println("Solve child node $id")
    displayBranchingRule(branching_decision)
    # * Instead of copying the model, just filter out routes and set bounds to 0
    execution_time = @elapsed begin
        routes_2e_to_keep, columns_to_be_deleted = filter_2e_routes(branching_decision, collect(1:length(routes_2e)))
    end
    global execution_time_filtering += execution_time

    execution_time = @elapsed begin
        # * Set bounds instead of actually deleting them
        for route_idx in columns_to_be_deleted
            if haskey(y_vars, route_idx)
                y = y_vars[route_idx]
                JuMP.set_upper_bound(y, 0.0)
                JuMP.set_lower_bound(y, 0.0)
            end
        end
        for route_idx in routes_2e_to_keep
            if haskey(y_vars, route_idx)
                y = y_vars[route_idx]
                JuMP.set_upper_bound(y, 1.0)
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
        child_node = solve_column_generation(route_1e, branching_decision, node.cgLowerBound, 
                                             node.fractionalScore, id, node.id) 

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
#region : branch and price 2e subproblem
function solve_branch_and_price_2e_subproblem(route_1e::Route, node_stack)

    if isnothing(node_stack)
        return
    else
        # * 2. Branch and Price child nodes
        num_iter_sp = 1
        current_node_id = 0
        while !isempty(node_stack) # && num_iter_sp < 51
            println("================Iteration $num_iter_sp of B&P for SP$num_iter_global $(route_1e.sequence) parkings$([r for r in getServedParking1eRoute(route_1e)])================")
            
            # * 2.1 Select a node from search tree
            node = select_node_from_tree(node_stack)
            
            if node.cgLowerBound > upperBound
                @info "$(round(node.cgLowerBound, digits=2)), Exceed Upper Bound, prune"
                println("$(round(node.cgLowerBound, digits=2)), Exceed Upper Bound, prune")
            else
                # * 2.2 Obtain branching strategy
                execution_time = @elapsed begin
                    branching_decisions = branchingStrategy(node.y_value, route_1e, routes_2e, node.branchingInfo)
                end
                global execution_time_branching += execution_time

                # execution_time = @elapsed begin
                    left_child_node  = solve_child_node(route_1e, node, branching_decisions[1], current_node_id + 1)
                    right_child_node = solve_child_node(route_1e, node, branching_decisions[2], current_node_id + 2)
                    current_node_id += 2
                    if !isnothing(left_child_node)
                        push!(node_stack, left_child_node)
                    end
                    if !isnothing(right_child_node)
                        push!(node_stack, right_child_node)
                    end
                # end
                # global execution_time_child_node += execution_time
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

function solve_MILP_model_root()

    milpModel = Model(CPLEX.Optimizer)
    set_silent(milpModel)

    @variable(milpModel, x[1:length(routes_1e_complete)], Bin)
    @variable(milpModel, y[1:length(routes_2e)], Bin)

    @constraint(milpModel, [p in satellites], sum(y[idx]*r.b2out[p] for (idx, r) in enumerate(routes_2e)) <=
                                              nb_vehicle_per_satellite* sum(x[idx]*r.b1[p] for (idx, r) in enumerate(routes_1e_complete)))
    @constraint(milpModel, [i in customers], sum(y[idx]*r.a[i] for (idx,r) in enumerate(routes_2e)) >=1)
    @constraint(milpModel, [p in satellites], sum(y[idx]*r.b2in[p] for (idx,r) in enumerate(routes_2e))== sum(y[idx]*r.b2out[p] for (idx,r) in enumerate(routes_2e)))
    @constraint(milpModel, [p in satellites], sum(y[idx]*r.load*r.b2out[p] for (idx,r) in enumerate(routes_2e))<= capacity_microhub)
    @constraint(milpModel, sum(x[idx] for (idx, _) in enumerate(routes_1e_complete)) ==1 )

    @objective(milpModel, Min, sum(y[idx]*r.cost for (idx,r) in enumerate(routes_2e))
                             + sum(x[idx]*r.cost for (idx,r) in enumerate(routes_1e_complete)))

    optimize!(milpModel)

    println("MILP Result:\n",round(objective_value(milpModel)))

    for (idx,r) in enumerate(routes_2e) 
        if value(y[idx]) != 0
            println(r.sequence, "   $(round(r.cost, digits=2))")
        end
    end

    for (idx,r) in enumerate(routes_1e_complete) 
        if value(x[idx]) != 0
            println(r.sequence, "   $(round(r.cost, digits=2))")
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