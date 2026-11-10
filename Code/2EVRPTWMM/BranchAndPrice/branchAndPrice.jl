include("branchingStrategies.jl")
include("Utiles.jl")
include("columnGeneration.jl")

# Test flag: when true, branchingStrategy skips every other rule (route-count,
# per-satellite, reversed-route) and only ever branches on a customer-customer
# pair taken from the most fractional route. Set `global cc_only_branching = true`
# before running the branch-and-price to enable this test mode.
global cc_only_branching = false

# Primal-heuristic flag: when true, after each fractional root node the
# restricted master is solved as a MILP over the columns found so far
# (solve_MILP_model, 10 s limit) to get an incumbent before branching.
# Off by default: on cf1-2,3,30 it found 612.14 (optimum 610.60) at once but
# the tree grew from 6,344 to 6,926 nodes, because an incumbent also switches
# select_node_from_tree to its gradient-score rule earlier.
global milp_incumbent_heuristic = false

# Baseline test flag: when true, branchingStrategy skips every other rule and
# always branches on a random customer pair drawn from the most fractional
# route (random-pair Ryan-Foster branching). Falls back to the combination
# rule (branchOnCombinationParkingCustomer) if no valid random pair is found.
# Checked first: takes priority over `cc_only_branching`.
global random_pair_branching = false

# Search-strategy flag: when true, select_node_from_tree explores the B&P tree
# depth-first (always expand the most recently created node, i.e. pop the end
# of node_stack) instead of the default best-bound/gradient-score selection.
global dfs_node_selection = false

# Baseline flag: when true, microhubs stay at the parkings where they start
# (parking_availability == 1) instead of being relocated. The 1e route of each
# subset of these parkings is solved with an arc-based TSP model
# (solve_1e_tsp_arc). 2e routes may still end at a different satellite than
# the one they left from, as in the full mode. Must be set before
# preparation_branch_and_price.
global fixed_microhub_baseline = false

# Seed for the random pair choice. The generator is re-created from this seed
# for every instance (in preparation_branch_and_price), so a random-branching
# run is reproducible and does not depend on what else used the global RNG.
global random_branching_seed = 42
global random_branching_rng = MersenneTwister(random_branching_seed)

# Time limit of the whole run. The run script sets bap_start_time (same clock
# as its start_time) and bap_time_limit before calling preparation_branch_and_price.
# time_limit_hit stays true once the limit is passed, so the run script can tell
# a run cut short (not proven optimal) from a finished one.
global bap_start_time = time()
global bap_time_limit = Inf
global time_limit_hit = false

function time_limit_exceeded()
    if !time_limit_hit && time() - bap_start_time > bap_time_limit
        global time_limit_hit = true
    end
    return time_limit_hit
end

function blockColumn()
    
end

function select_node_from_tree(node_stack)
    if dfs_node_selection
        node = pop!(node_stack)
        @info "Display selected node $(node.id) in level $(node.branchingInfo.depth), parent node $(node.parent_id): from $(length(node_stack)+1) nodes (DFS)"
        println("Display selected node $(node.id) in level $(node.branchingInfo.depth), parent node $(node.parent_id): from $(length(node_stack)+1) nodes (DFS), current upper bound = $(round(upperBound, digits=2))")
        return node
    end

    if upperBound == Inf
        # No integer solution yet: select node with highest CG lower bound
        node = node_stack[1]
        for node_iter in node_stack
            if node_iter.cgLowerBound > node.cgLowerBound
                node = node_iter
            end
        end
    else
        # Integer solution exists: use original rule
        node = node_stack[1]
        base_lb = node.cgLowerBound
        score = 0.5*node.gradientFS / (1+log(node.branchingInfo.depth))
        for node_iter in node_stack
            score_iter = node_iter.cgLowerBound - base_lb + 0.5*node_iter.gradientFS/(1+log(node_iter.branchingInfo.depth))
            if score_iter < score
                node = node_iter
                score = score_iter
            end
        end
    end

    @info "Display selected node $(node.id) in level $(node.branchingInfo.depth), parent node $(node.parent_id): from $(length(node_stack)) nodes"
    println("Display selected node $(node.id) in level $(node.branchingInfo.depth), parent node $(node.parent_id): from $(length(node_stack)) nodes, current upper bound = $(round(upperBound, digits=2))")

    deleteat!(node_stack, findfirst(==(node), node_stack))
    return node
end

# Upper bound currently set in CPLEX for each column (missing key = 1.0, the
# value add_2eroute! creates it with). Lower bounds are always 0 and never
# change. Going through set_y_ub! skips the (costly) CPLEX call whenever the
# bound is already right, e.g. for the many columns of other subproblems that
# stay fixed at 0 across every node of this subproblem.
global y_ub = Dict{Int, Float64}()

function set_y_ub!(route_id::Int, ub::Float64)
    if get(y_ub, route_id, 1.0) != ub
        JuMP.set_upper_bound(y_vars[route_id], ub)
        y_ub[route_id] = ub
    end
end

# Whether column `route` is allowed under the branching decisions in `branchingInfo`.
function route_respects_branching(branchingInfo::BranchingInfo, route)
    seq = route.sequence
    first_node = seq[1]
    last_node = seq[end]
    if first_node in branchingInfo.forbidden_parkings || last_node in branchingInfo.forbidden_parkings
        return false
    end
    # Start-parking – customer rules
    for (p, c) in branchingInfo.must_include_combinations
        # if a route includes the target customer, its START parking must be the given one
        if c in seq && p != first_node
            return false
        end
    end
    for (p, c) in branchingInfo.forbidden_combinations
        # a route starting from p cannot serve customer c
        if p == first_node && c in seq
            return false
        end
    end
    # End-parking – customer rules
    for (p, c) in branchingInfo.must_include_end_combinations
        # if a route includes the target customer, its END parking must be the given one
        if c in seq && p != last_node
            return false
        end
    end
    for (p, c) in branchingInfo.forbidden_end_combinations
        # a route ending at p cannot serve customer c
        if p == last_node && c in seq
            return false
        end
    end
    for (c1, c2) in branchingInfo.must_served_together
        # a route including only one of the two customers is not allowed
        if (c1 in seq) != (c2 in seq)
            return false
        end
    end
    for (c1, c2) in branchingInfo.forbidden_served_together
        # a route including both customers is not allowed
        if (c1 in seq) && (c2 in seq)
            return false
        end
    end
    return true
end

function filter_2e_routes(branchingInfo::BranchingInfo, routes::Vector{Int})
    result = Vector{Int}()
    routes_to_delete = Vector{Int}()
    for route_id in routes
        if route_respects_branching(branchingInfo, routes_2e[route_id])
            push!(result, route_id)
            set_y_ub!(route_id, 1.0)
        else
            push!(routes_to_delete, route_id)
            set_y_ub!(route_id, 0.0)
        end
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
    global total_bap_nodes = 0
    global time_limit_hit = false
    global y_ub = Dict{Int, Float64}()   # bound cache for the new master model
    global random_branching_rng = MersenneTwister(random_branching_seed)
    #endregion

    generate2eInitialRoutes()

    global neighbours = get_neighbours_optimized(10)

    lrp_subproblems = get_sorted_2e_subproblems()

    return lrp_subproblems
end

#region : column generation
function solve_column_generation(route_1e, branchingInfo::BranchingInfo, cgLB, fs, id, parent_id)
    # * Column generation process:
    # *     - 1. solve formulation
    # *     - 2. get dual multiplier
    # *     - 3. execute labelling algorithm
    # *     - 4. check existence new routes

    global total_bap_nodes += 1

    selected_parkings = getServedParking1eRoute(route_1e)
    
    num_iter_cg = 1
    # Pre-allocate dual multiplier arrays to avoid reallocation each iteration
    n_satellites = length(satellites)
    n_customers = length(customers)
    π1 = Vector{Float64}(undef, n_satellites + 1)
    π2 = Vector{Float64}(undef, n_satellites + n_customers + 1)
    π3 = Vector{Float64}(undef, n_satellites + 1)
    π4 = Vector{Float64}(undef, n_satellites + 1)

    solution_contains_dummy_route = true
    

    # execution_time_loop = @elapsed begin

    dummy_route_in_solution = true
    dummy_route_selected = Vector{Int}()
    while true # num_iter_cg < 2 # && true
        # println("-------------Iter CG $num_iter_cg-------------")
        # Out of time: drop the node. Its LP bound is not final, so it can
        # neither be pruned nor branched on correctly; the run stops anyway.
        if time_limit_exceeded()
            println("Time limit reached during column generation of node $id")
            return nothing
        end
        if length(dummy_route_selected) == 0 && num_iter_cg > 1 && dummy_route_in_solution
            dummy_route_in_solution = false
            for route in dummyRoutes_numeration
                # if haskey(y_vars, route.id)
                    set_y_ub!(route, 0.0)
                # end
                # println(route.id, "  ", route.sequence)
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
            # println("result of column generation : $(round(lpObjValue, digits=3))")
            # execution_time_dual = @elapsed begin
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
                # dual(), not shadow_price(): for an equality constraint JuMP's
                # shadow_price is -|dual| in a min problem, so it loses the sign.
                # With a positive flow dual, every route ending at a different
                # satellite than it starts from was priced wrongly (up to ~11 off
                # on ce1-3,5,20), so CG could stop with improving columns left.
                π3[i+1] = dual(number2evfixe[i])
            end
            
            π4[1] = 0.0
            @inbounds for i in 1:n_satellites
                π4[i+1] = abs(shadow_price(maxVolumnMM[i]))
            end

            π5 = abs(shadow_price(globalLowerBound))
            π6 = abs(shadow_price(globalUpperBound))

            π_sat_lb = zeros(Float64, length(points))
            π_sat_ub = zeros(Float64, length(points))
            @inbounds for s in satellites
                π_sat_lb[s] = abs(shadow_price(satelliteRouteLB[s]))
                π_sat_ub[s] = abs(shadow_price(satelliteRouteUB[s]))
            end

            # println("π5 = ", round(π5, digits=2))
            # println("π6 = ", round(π6, digits=2))
            #endregion
            #end
            # println("--execution time dual: ", round(execution_time_dual, digits=3))


            #region: PRINT cg y value
            # dummy_route_selected = Vector{Int}()
            # Only dummy columns matter here; reading every column's value was
            # a full pass over the pool on each CG iteration.
            for k in dummyRoutes_numeration
                if haskey(y_vars, k) && value(y_vars[k]) != 0
                    push!(dummy_route_selected, k)
                end
            end
            # println("Dummy route in solution: ", length(dummy_route_selected))
            #endregion

            # * 3. execute labelling algorithm
            # execution_time_p = @elapsed begin
                _max_cols = length(customers) >= 75 ? 200 : 100
                new_columns_found = pricing(selected_parkings, collect(1:length(routes_2e)), π1, π2, π3, π4, π5, π6, branchingInfo, π_sat_lb, π_sat_ub; max_cols = _max_cols)
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


    # Extract y_values more efficiently - avoid sort and collect
    n_vars = length(y_vars)
    y_values = Vector{Float64}(undef, n_vars)
    sorted_keys = sort!(collect(keys(y_vars)))
    sum_y_value = 0.0
    sum_per_satellite = Dict{Int, Float64}()
    @inbounds for (idx, k) in enumerate(sorted_keys)
        # Columns fixed at 0 (upper bound 0) are 0; skip the CPLEX query.
        y_values[idx] = get(y_ub, k, 1.0) == 0.0 ? 0.0 : value(y_vars[k])
        #region: PRINT cg y value
        if y_values[idx] != 0
            sum_y_value += y_values[idx]
            start_parking = routes_2e[value(k)].sequence[1]
            sum_per_satellite[start_parking] = get(sum_per_satellite, start_parking, 0.0) + y_values[idx]
            if status_debug
                println("y$(routes_2e[value(k)].sequence) = $(round(y_values[idx], digits=2)), $(round(routes_2e[value(k)].cost, digits=2))")
            end
            # println("y$(routes_2e[value(k)].sequence) = $(round(y_values[idx],digits=2)), $(round(routes_2e[value(k)].cost, digits=2))")
        end
        #endregion
    end
    # println("sum of y values (total)            = $(round(sum_y_value, digits=3))")
    # for s in sort!(collect(keys(sum_per_satellite)))
    #     println("sum of y values starting at depot $s = $(round(sum_per_satellite[s], digits=3))")
    # end
    if total_obj >= upperBound - 1e-6
        @info "Exceed Upper Bound, prune"
        println("Exceed Upper Bound, prune")
        return nothing
    end
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
            if length(customers) >= 75
                # Fallback row in case the job is killed before the end: keeps
                # the best solution but no time, since the time an incumbent is
                # found is not the execution time. The final row in the run
                # script overwrites it with the true total time.
                _row = [
                    Dates.format(now(), "dd-mm-yyyy-HH-MM-SS-s"),
                    fixed_microhub_baseline ? "bl_bp" : "bp", "\"$filename\"",
                    length(customers), length(satellites), sum(parking_availability),
                    nb_vehicle_per_satellite,
                    "/", "/", "/",
                    "/", "/", "/",
                    upperBound, "incumbent"
                ]
                open(outfile, "w") do _f
                    println(_f, join(_row, ","))
                end
            end
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
    # if upperBound != Inf
    #     gradientLB = upperBound - cgLB
    # else
        gradientLB = total_obj - cgLB
    # end
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
# Route-count and per-satellite bounds of the master without any branching.
function reset_master_bounds!()
    set_normalized_rhs(globalLowerBound, -lower_bound_2e_routes)
    set_normalized_rhs(globalUpperBound, upper_bound_2e_routes)
    for s in satellites
        set_normalized_rhs(satelliteRouteLB[s], 0)
        set_normalized_rhs(satelliteRouteUB[s], nb_vehicle_per_satellite)
    end
end

function solve_root_node(route_1e::Route)
    # A root has no branching decisions: start from the default bounds (the
    # previous subproblem's last child may have tightened them).
    reset_master_bounds!()
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
            Dict{Int,Int}(),         # upper_bound_per_satellite
            Dict{Int,Int}(),         # lower_bound_per_satellite
            0                        # depth
        )
        root_node_branching_info.forbidden_parkings = setdiff(Set(satellites), getServedParking1eRoute(route_1e))

        columns_to_be_kept, columns_to_be_deleted = filter_2e_routes(root_node_branching_info, collect(1:length(routes_2e)))

        # println("number of 2e routes before column generation: ", length(columns_to_be_kept))
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
                # Get an incumbent from the columns found at the root before
                # branching, so the tree can prune by bound early.
                if milp_incumbent_heuristic
                    solve_MILP_model(route_1e)
                end
                if root_node.cgLowerBound >= upperBound - 1e-6
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
    
    if status_debug
        displayBranchingRule(branching_decision)
    end

    # * Instead of copying the model, just filter out routes and set bounds to 0
    execution_time = @elapsed begin
        routes_2e_to_keep, columns_to_be_deleted = filter_2e_routes(branching_decision, collect(1:length(routes_2e)))
    end
    global execution_time_filtering += execution_time

    execution_time = @elapsed begin
        # * Column bounds were already set by filter_2e_routes above.

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

        # * Per-satellite fleet bounds
        for s in satellites
            lb = get(branching_decision.lower_bound_per_satellite, s, 0)
            ub = get(branching_decision.upper_bound_per_satellite, s, nb_vehicle_per_satellite)
            set_normalized_rhs(satelliteRouteLB[s], -lb)
            set_normalized_rhs(satelliteRouteUB[s], ub)
        end
    end
    global execution_time_set_bound += execution_time
    # global execution_time_build_model += execution_time

    execution_time = @elapsed begin
        child_node = solve_column_generation(route_1e, branching_decision, node.cgLowerBound, node.fractionalScore, id, node.id)
    end
    global execution_time_column_generation += execution_time

    execution_time = @elapsed begin
        # * Reset route-count and satellite bounds after solving, so the next
        # * node or root does not inherit this child's branching bounds.
        # * Column bounds are not reset: every node (and every root)
        # * re-applies them via filter_2e_routes.
        reset_master_bounds!()
    end
    global execution_time_set_bound += execution_time
    # global execution_time_build_model += execution_time

    return child_node
end
#endregion
#region : branch and price 2e subproblem
function new_branch_and_price(route_1e::Route, node_stack)
    if isnothing(node_stack)
        return
    end

    num_iter_sp = 1
    current_node_id = 0

    while !isempty(node_stack)
        println("================Iteration $num_iter_sp of B&P for SP$num_iter_global $(route_1e.sequence)parkings$([r for r in getServedParking1eRoute(route_1e)])================")

        
        
    end
end
function solve_branch_and_price_2e_subproblem(route_1e::Route, node_stack)

    if isnothing(node_stack)
        return
    else
        # * 2. Branch and Price child nodes
        num_iter_sp = 1
        current_node_id = 0

        while !isempty(node_stack) # && num_iter_sp < 51
            if time_limit_exceeded()
                println("\n Time limit reached in branch-and-price tree: $(length(node_stack)) open nodes left unexplored.")
                break
            end
            println("================Iteration $num_iter_sp of B&P for SP$num_iter_global $(route_1e.sequence) parkings$([r for r in getServedParking1eRoute(route_1e)])================")
            
            # * 2.1 Select a node from search tree
            node = select_node_from_tree(node_stack)
            
            if node.cgLowerBound >= upperBound - 1e-6
                @info "$(round(node.cgLowerBound, digits=2)), Exceed Upper Bound, prune"
                println("$(round(node.cgLowerBound, digits=2)), Exceed Upper Bound, prune")
            else
                # * 2.2 Obtain branching strategy
                execution_time = @elapsed begin
                    branching_decisions = branchingStrategy(node.y_value, route_1e, routes_2e, node.branchingInfo)
                end
                global execution_time_branching += execution_time

                if isnothing(branching_decisions)
                    @warn "No branching decision found for node $(node.id); dropping node without children"
                    println("No branching decision found for node $(node.id); dropping node without children")
                else
                    left_child_node  = solve_child_node(route_1e, node, branching_decisions[1], current_node_id + 1)
                    right_child_node = solve_child_node(route_1e, node, branching_decisions[2], current_node_id + 2)
                    current_node_id += 2

                    if !isnothing(left_child_node)
                        push!(node_stack, left_child_node)
                    end
                    if !isnothing(right_child_node)
                        push!(node_stack, right_child_node)
                    end
                end
            end
            num_iter_sp += 1
        end
    end

end
#endregion
# Primal heuristic: solve the restricted master as a MILP over the columns
# generated so far (for this 1e route's parkings) to get an incumbent early,
# so the B&P tree can prune by bound from the start. Any solution found is a
# genuine feasible solution: same constraints as the master, and dummy routes
# carrying the +1e4 infeasibility penalty are excluded.
function solve_MILP_model(route_1e; time_limit = 10.0)
    selected_parkings = getServedParking1eRoute(route_1e)

    routes_2e_pool = Vector{Route}()
    for route in routes_2e
        if route.sequence[1] in selected_parkings && route.sequence[end] in selected_parkings &&
           route.cost < 1e4
            push!(routes_2e_pool, route)
        end
    end
    isempty(routes_2e_pool) && return

    milpModel = Model(CPLEX.Optimizer)
    set_silent(milpModel)
    set_optimizer_attribute(milpModel, "CPX_PARAM_THREADS", 1)
    set_time_limit_sec(milpModel, time_limit)
    @variable(milpModel, y[1:length(routes_2e_pool)], Bin)

    @constraint(milpModel, [p in selected_parkings], sum(y[idx]*r.b2out[p] for (idx,r) in enumerate(routes_2e_pool))<=nb_vehicle_per_satellite)
    @constraint(milpModel, [i in customers], sum(y[idx]*r.a[i] for (idx,r) in enumerate(routes_2e_pool)) >=1)
    @constraint(milpModel, [p in selected_parkings], sum(y[idx]*r.b2in[p] for (idx,r) in enumerate(routes_2e_pool))== sum(y[idx]*r.b2out[p] for (idx,r) in enumerate(routes_2e_pool)))
    @constraint(milpModel, [p in selected_parkings], sum(y[idx]*r.load*r.b2out[p] for (idx,r) in enumerate(routes_2e_pool))<= capacity_microhub)

    @objective(milpModel, Min, sum(y[idx]*r.cost for (idx,r) in enumerate(routes_2e_pool)))

    optimize!(milpModel)
    if !has_values(milpModel)
        println("MILP heuristic: no integer solution found")
        return
    end

    milp_value = objective_value(milpModel) + route_1e.cost
    println("MILP heuristic result: ", round(milp_value, digits=2))
    if milp_value < upperBound
        global upperBound = milp_value
        global optimalSolution = Vector{Route}()
        println("Update upper bound (MILP heuristic) = $upperBound")
        push!(optimalSolution, route_1e)
        for (idx,r) in enumerate(routes_2e_pool)
            if value(y[idx]) > 0.5
                push!(optimalSolution, r)
            end
        end
    end
end


function branchingStrategy(y, route_1e, routes_pool, branchingInfo::BranchingInfo)

    left_branch = deepcopy(branchingInfo)
    right_branch = deepcopy(branchingInfo)

    if random_pair_branching
        result = branchOnRandomPairMostFractionalRoute(branchingInfo, y, routes_pool)
        if isnothing(result)
            @info "random_pair_branching: no valid random customer pair found; falling back to satellite-customer branching"
            println("random_pair_branching: no valid random customer pair found; falling back to satellite-customer branching")
            result = branchOnCombinationParkingCustomer(route_1e, branchingInfo, y, routes_pool)
            if isnothing(result)
                @warn "random_pair_branching: no valid satellite-customer branching found either; node cannot be branched further"
                println("random_pair_branching: no valid satellite-customer branching found either; node cannot be branched further")
            end
        end
        return result
    end

    if cc_only_branching
        result = branchOnCustomerCustomerMostFractionalRoute(branchingInfo, y, routes_pool)
        if isnothing(result)
            @warn "cc_only_branching: no valid customer-customer pair found in any fractional route; node cannot be branched further"
            println("cc_only_branching: no valid customer-customer pair found in any fractional route; node cannot be branched further")
        end
        return result
    end

    ## Case A: total number of 2e route is fractional
    if !(abs(sum(y)-round(sum(y)))<1e-8)
        # Skip this rule if some route already carries a small fractional
        # weight (< 1/3), or if the fractional routes are all exactly 0.5 —
        # in either case fall through to A2/B/C instead.
        fractional_y = [y[r] for r in 1:length(y) if 0 < y[r] < 1]
        has_small_fractional_route = any(v -> v < 1/3, fractional_y)
        all_halves = !isempty(fractional_y) && all(v -> abs(v - 0.5) < 1e-8, fractional_y)
        if !has_small_fractional_route && !all_halves
            @info "Branch on total number of 2e routes:  $(floor(sum(y))), $(ceil(sum(y)))"
            println("Branch on total number of 2e routes:  $(floor(sum(y))), $(ceil(sum(y)))")
            push!(left_branch.lower_bound_number_2e_routes, ceil(sum(y)))
            push!(right_branch.upper_bound_number_2e_routes, floor(sum(y)))
            left_branch.depth += 1
            right_branch.depth += 1
            return left_branch, right_branch
        elseif has_small_fractional_route
            @info "Skipping total-route-count branch: a fractional y < 1/3 exists"
            println("Skipping total-route-count branch: a fractional y value < 1/3 exists")
        else
            @info "Skipping total-route-count branch: all fractional y values are 0.5"
            println("Skipping total-route-count branch: all fractional y values are 0.5")
        end
    end

    ## Case A2: per-satellite fleet sum is fractional
    sum_per_satellite = Dict{Int, Float64}()
    for (r_idx, y_val) in enumerate(y)
        if y_val > 1e-8
            s = routes_pool[r_idx].sequence[1]
            sum_per_satellite[s] = get(sum_per_satellite, s, 0.0) + y_val
        end
    end
    for s in sort(collect(keys(sum_per_satellite)))
        val = sum_per_satellite[s]
        if !(abs(val - round(val)) < 1e-8)
            # Same skip rule as Case A, scoped to this satellite's own routes:
            # skip if a fractional y < 1/3 exists, or if the fractional
            # routes here are all exactly 0.5 — try the next satellite (or
            # fall through to B/C) instead.
            fractional_y_sat = [y[r] for r in 1:length(y) if 0 < y[r] < 1 && routes_pool[r].sequence[1] == s]
            has_small_fractional_route_sat = any(v -> v < 1/3, fractional_y_sat)
            all_halves_sat = !isempty(fractional_y_sat) && all(v -> abs(v - 0.5) < 1e-8, fractional_y_sat)
            if !has_small_fractional_route_sat && !all_halves_sat
                println("Branch on satellite $s fleet: $(Int(floor(val))) / $(Int(ceil(val)))")
                left_branch.lower_bound_per_satellite[s] = Int(ceil(val))
                right_branch.upper_bound_per_satellite[s] = Int(floor(val))
                left_branch.depth += 1
                right_branch.depth += 1
                return left_branch, right_branch
            elseif has_small_fractional_route_sat
                @info "Skipping satellite $s fleet branch: a fractional y < 1/3 exists"
                println("Skipping satellite $s fleet branch: a fractional y value < 1/3 exists")
            else
                @info "Skipping satellite $s fleet branch: all fractional y values are 0.5"
                println("Skipping satellite $s fleet branch: all fractional y values are 0.5")
            end
        end
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