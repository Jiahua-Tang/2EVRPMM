include("branchingStrategies.jl")
include("Utiles.jl")
include("columnGeneration.jl")

function blockColumn()
    
end

function select_node_from_tree(node_stack)
    node = node_stack[1]
    score = (-node.gradientLB + 0.5*node.gradientFS) / (1+log(node.branchingInfo.depth))
    for node_iter in node_stack 
        score_iter = (node_iter.gradientLB + 0.5*node_iter.gradientFS)/(1+log(node_iter.branchingInfo.depth))
        if score_iter < score
            node = node_iter
            score = score_iter
        end
    end

    @info "Display selected node $(node.id) in level $(node.branchingInfo.depth), parent node $(node.parent_id): from $(length(node_stack)) nodes"
    println("Display selected node $(node.id) in level $(node.branchingInfo.depth), parent node $(node.parent_id): from $(length(node_stack)) nodes, current upper bound = $(round(upperBound, digits=2))")
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

        # Start-parking – customer rules
        for must_include in branchingInfo.must_include_combinations 
            # if a route includes the target customer, its START parking must be the given one
            if must_include[2] in route.sequence && !(must_include[1] == route.sequence[1])
                valide = false
                break            
            end
        end    

        for forbidden in branchingInfo.forbidden_combinations 
            # a route starting from forbidden[1] cannot serve customer forbidden[2]
            if Int(forbidden[1] == route.sequence[1]) + Int(forbidden[2] in route.sequence) == 2
                valide = false
                break
            end
        end

        # End-parking – customer rules (new)
        for must_include_end in branchingInfo.must_include_end_combinations
            # if a route includes the target customer, its END parking must be the given one
            if must_include_end[2] in route.sequence && !(must_include_end[1] == route.sequence[end])
                valide = false
                break
            end
        end

        for forbidden_end in branchingInfo.forbidden_end_combinations
            # a route ending at forbidden_end[1] cannot serve customer forbidden_end[2]
            if Int(forbidden_end[1] == route.sequence[end]) + Int(forbidden_end[2] in route.sequence) == 2
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
        lrp_subproblems = get_sorted_2e_subproblems()
    end
    # println("time to get sorted 2e subproblem = $(/round(execution_time, digits=2))s")

    return lrp_subproblems
end

#region : column generation
function solve_column_generation(route_1e, branchingInfo::BranchingInfo, cgLB, fs, id, parent_id)
    # * Column generation process:
    # *     - 1. solve formulation
    # *     - 2. get dual multiplier
    # *     - 3. execute labelling algorithm
    # *     - 4. check existence new routes

    selected_parkings = getServedParking1eRoute(route_1e)
    
    num_iter_cg = 1
    # Pre-allocate dual multiplier arrays to avoid reallocation each iteration
    n_satellites = length(satellites)
    n_customers = length(customers)
    π1 = Vector{Float64}(undef, n_satellites + 1)
    π2 = Vector{Float64}(undef, n_satellites + n_customers + 1)
    π3 = Vector{Float64}(undef, n_satellites + 1)
    π4 = Vector{Float64}(undef, n_satellites + 1)
    

    

    # execution_time_loop = @elapsed begin
    while true # num_iter_cg < 2 # && true
        # println("-------------Iter CG $num_iter_cg-------------")
        if num_iter == 2
            for route in dummyRoutes
                if haskey(y_vars, route.id)
                    y = y_vars[route.id]
                    JuMP.set_upper_bound(y, 0.0)
                    JuMP.set_lower_bound(y, 0.0)
                end
            end
        end


        # * 1. solve formulation
        # execution_time_lp = @elapsed begin
            optimize!(model)
        # end
        # println("--execution time solving lp: ", round(execution_time_lp, digits=3))

        if has_values(model)
            # Cache objective value (used multiple times)
            obj_val = objective_value(model)
            lpObjValue = obj_val + route_1e.cost
            # println("result of column generation : $(round(lpObjValue, digits=2))")
            # execution_time_dual = @elapsed begin
            # * 2. get dual multiplier - optimized to avoid allocations
            #region : retrieve dual multiplier
            # n_vars = length(y_vars)
            # y_values = Vector{Float64}(undef, n_vars)
            # sorted_keys = sort!(collect(keys(y_vars)))
            #     @inbounds for (idx, k) in enumerate(sorted_keys)
            #         y_values[idx] = value(y_vars[k])
            #         #region: PRINT cg y value
            #         if y_values[idx] !=  0
            #             # sum_y_value += y_values[idx]
            #             println("y$(routes_2e[value(k)].sequence) = $(round(y_values[idx],digits=2))")
            #         end
            #     end
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
                π3[i+1] = shadow_price(number2evfixe[i])
            end
            
            π4[1] = 0.0
            @inbounds for i in 1:n_satellites
                π4[i+1] = abs(shadow_price(maxVolumnMM[i]))
            end

            π5 = abs(shadow_price(globalLowerBound))
            π6 = abs(shadow_price(globalUpperBound))

            # println("π5 = ", round(π5, digits=2))
            # println("π6 = ", round(π6, digits=2))
            #endregion
            # end
            # println("--execution time dual: ", round(execution_time_dual, digits=3))

            # * 3. execute labelling algorithm
            # execution_time_p = @elapsed begin
                new_columns_found = pricing(selected_parkings, collect(1:length(routes_2e)), π1, π2, π3, π4, π5, π6, branchingInfo)
                # println("now there are $(length(routes_2e)) 2e routes in total")
            # end
            # println("--execution time solving pricing: ", round(execution_time_p, digits=3),"\n")
            if !new_columns_found
                break
            end

        else
            println("No feasible solution for LMP")
            return nothing
        end
        num_iter_cg += 1
    end
    # end
    # println("--execution time loop: ", round(execution_time_loop,digits=2))
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
    sum_y_value = 0
    @inbounds for (idx, k) in enumerate(sorted_keys)
        y_values[idx] = value(y_vars[k])
        #region: PRINT cg y value
        if y_values[idx] !=  0
            sum_y_value += y_values[idx]
            println("y$(routes_2e[value(k)].sequence) = $(round(y_values[idx],digits=2))")
        end

        #region: block some used columns

        #endregion
    end
    println("number of 2e routes: ",length(routes_2e))
    # println("sum of y value is : $(round(sum_y_value,digits=2))")
    #endregion

    # Check for integer solution and compute fractional score
    isLeaf = true
    fractionalScore = 0.0
    @inbounds for y_val in y_values
        if 1e-8 < y_val < 1-1e-8
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
                if y_values[i] >= 1-1e-8
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
    # execution_time_verified_root_node = @elapsed begin
        println("\nSolve root node of $(route_1e.sequence),   $(getServedParking1eRoute(route_1e))")
        root_node_branching_info = BranchingInfo(
            Set{Tuple{Int, Int}}(),  # must_include_combinations (start parking, customer)
            Set{Tuple{Int, Int}}(),  # forbidden_combinations (start parking, customer)
            Set{Tuple{Int, Int}}(),  # must_include_end_combinations (end parking, customer)
            Set{Tuple{Int, Int}}(),  # forbidden_end_combinations (end parking, customer)
            Set{Tuple{Int, Int}}(),  # must_served_together
            Set{Tuple{Int, Int}}(),  # forbidden_served_together
            Set{Int}(),              # must_include_parkings
            Set{Int}(),              # forbidden_parkings
            Set{Int}(),              # upper_bound_number_2e_routes
            Set{Int}(),              # lower_bound_number_2e_routes
            0                        # depth
        )
        root_node_branching_info.forbidden_parkings = setdiff(Set(satellites), getServedParking1eRoute(route_1e))

        #region : initial columns
        # execution_time = @elapsed begin
            columns_to_be_kept, columns_to_be_deleted = filter_2e_routes(root_node_branching_info, collect(1:length(routes_2e)))
        # end
        # global execution_time_filtering += execution_time
        # execution_time = @elapsed begin
            for idx in columns_to_be_deleted
                if haskey(y_vars, idx)
                    y = y_vars[idx]
                    JuMP.set_upper_bound(y, 0.0)
                    JuMP.set_lower_bound(y, 0.0)
                end
            end
            for idx in columns_to_be_kept
                # println(routes_2e[idx].sequence)
                if haskey(y_vars, idx)
                    y = y_vars[idx]
                    JuMP.set_upper_bound(y, 1.0)
                    JuMP.set_lower_bound(y, 0.0)
                end
            end
        # end
        # println("execution time on filtering and bounding initial columns: ",round(execution_time, digits=2),"s")

        # execution_time = @elapsed begin
            println("number of 2e routes before column generation: ", length(columns_to_be_kept))
            root_node = solve_column_generation(route_1e, root_node_branching_info, 0,0,0,0)
        # end
        # println("-execution time of column generation : $(round(execution_time, digits=2))s")

    # end
    # println("verification: execution time for root node: ", execution_time_verified_root_node,"")

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
            # execution_time = @elapsed begin
                if root_node.cgLowerBound > upperBound
                    return nothing
                end
                
                node_stack = [root_node]
            # println("Branching stack contains now $(length(node_stack)) nodes, current upper bound is $(round(upperBound,digits=2))")  
            # end
            # println("execution time solving MILP: $(round(execution_time, digits=3)) second")
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

                #region: write node matrix
                row_data = [round(node.id), round(node.parent_id),round(node.cgLowerBound,digits=3), round(node.fractionalScore, digits=3), round(node.gradientLB,digits=3), round(node.gradientFS, digits=3),"prune"]
                open("NodeMatrix.csv", "a") do file
                    println(file, join(row_data, ",")) 
                end
                #endregion
            else
                # * 2.2 Obtain branching strategy
                execution_time = @elapsed begin
                    branching_decisions = branchingStrategy(node.y_value, route_1e, routes_2e, node.branchingInfo)
                end
                global execution_time_branching += execution_time

                # execution_time = @elapsed begin
                    # display(branching_decisions)
                    # display(branching_decisions[1])
                    # display(branching_decisions[2])

                    #region: write node matrix
                    row_data = [round(node.id), round(node.parent_id),round(node.cgLowerBound,digits=3), round(node.fractionalScore, digits=3), round(node.gradientLB,digits=3), round(node.gradientFS, digits=3),current_node_id+1, current_node_id+2]
                    open("NodeMatrix.csv", "a") do file
                        println(file, join(row_data, ",")) 
                    end
                    #endregion

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
    # execution_time_milp_function = @elapsed begin
    # execution_time_milp_1 = @elapsed begin

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

        set_optimizer_attribute(milpModel, "CPX_PARAM_THREADS", 4)
        @variable(milpModel, y[1:length(routes_2e_pool)], Bin)

        @constraint(milpModel, [p in selected_parkings], sum(y[idx]*r.b2out[p] for (idx,r) in enumerate(routes_2e_pool))<=nb_vehicle_per_satellite)
        @constraint(milpModel, [i in customers], sum(y[idx]*r.a[i] for (idx,r) in enumerate(routes_2e_pool)) >=1)
        @constraint(milpModel, [p in selected_parkings], sum(y[idx]*r.b2in[p] for (idx,r) in enumerate(routes_2e_pool))== sum(y[idx]*r.b2out[p] for (idx,r) in enumerate(routes_2e_pool)))
        @constraint(milpModel, [p in selected_parkings], sum(y[idx]*r.load*r.b2out[p] for (idx,r) in enumerate(routes_2e_pool))<= capacity_microhub)

        @objective(milpModel, Min, sum(y[idx]*r.cost for (idx,r) in enumerate(routes_2e_pool)))
    # end
    # println("execution time solving milp function etape 1: $(round(execution_time_milp_1, digits=3)) seconds")
    
    optimize!(milpModel)

    println("MILP Result:\n",round(objective_value(milpModel)+route_1e.cost,digits=2))
    
    # solve_time = MOI.get(model, MOI.SolveTime())
    # println("execution time CPLEX solve root MILP: ", solve_time(milpModel), " seconds")
    

    # println(route_1e.sequence, "    $(round(route_1e.cost, digits=2))")
    # for (idx,r) in enumerate(routes_2e_pool) 
    #     if value(y[idx]) != 0
    #         println(r.sequence, "   $(round(r.cost, digits=2))")
    #     end
    # end
    # execution_time_milp_3 = @elapsed begin
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
    # end
    # println("execution time solving milp function etape 3: $(round(execution_time_milp_3, digits=3)) seconds")

# end
# println("verification execution time : $execution_time_milp_function seconds")
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
    reversed_route = checkExistanceReversedRoute(
        sort([r for r in 1:length(y) if 0 < y[r] < 1],
             by = r -> y[r] * (1 - y[r]),
             rev = true),
        routes_pool
    )
    if !isnothing(reversed_route)
        return branchOnReverseRoute(branchingInfo, reversed_route)
    end

    # Case C: combination rules (start parking–customer, end parking–customer, then customer–customer)
    result = branchOnCombinationParkingCustomer(route_1e, branchingInfo, y, routes_pool)
    if !isnothing(result)
        return result
    end

    # ## Case D: combination of customers
    # result = branchOnArc(branchingInfo, y, routes_pool)
    # return result

end