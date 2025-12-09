

facilities = [1, 2, 3]
customers = [4, 5, 6, 7, 8, 9]
demands = [0, 0, 0, 10, 4, 13, 5, 19, 15]
nb_positions = 6
arc_costs =
    [
        Inf 25.3 25.4 25.4 35.4 37.4 31.9 24.6 34.2;
        25.3 Inf 21.2 16.2 27.1 26.8 17.8 16.7 23.2;
        25.4 21.2 Inf 14.2 23.4 23.8 18.3 17.0 21.6;
        25.4 16.2 14.2 Inf 28.6 28.8 22.6 15.6 29.5;
        35.4 27.1 23.4 28.6 Inf 42.1 30.4 24.9 39.1;
        37.4 26.8 23.8 28.8 42.1 Inf 32.4 29.5 38.2;
        31.9 17.8 18.3 22.6 30.4 32.4 Inf 22.5 30.7;
        24.6 16.7 17.0 15.6 24.9 29.5 22.5 Inf 21.4;
        34.2 23.2 21.6 29.5 39.1 38.2 30.7 21.4 Inf;
    ]

nb_customers = length(customers)
nb_facilities = length(facilities)
capacity = 20

using BlockDecomposition, Coluna, CPLEX, JuMP

nb_routes = Int(ceil(sum(demands)/capacity))
nb_routes_per_facilities = 2

function create_model(selected_facilities::Vector{Int}, optimizer, pricing_algorithms)
    global locations = vcat(selected_facilities, customers)
    @info locations
    @axis(facilities_axis, collect(selected_facilities))

    model = BlockModel(optimizer)

    @variable(model, x[i in customers, j in facilities_axis], Bin)

    @variable(model, z[u in locations, v in locations], Bin)

    @constraint(model, cov[i in customers], sum(x[i, j] for j in selected_facilities) >= 1)

    @constraint(model, inout[j in facilities_axis], sum(z[j, i] for i in locations) == sum(z[i, j] for i in locations))

    # @constraint(model, ub_veh[j in selected_facilities],
    #             sum(z[j,i] for i in customers) <= nb_routes_per_facilities)


    @objective(model, Min, sum(arc_costs[u,v] * z[u,v] for u in locations, v in locations if u != v))

    @dantzig_wolfe_decomposition(model, dec, facilities_axis)

    subproblems = BlockDecomposition.getsubproblems(dec)

    specify!.(subproblems, lower_multiplicity = 0, upper_multiplicity = nb_routes_per_facilities, solver = pricing_algorithms)

    subproblemrepresentative.(z, Ref(subproblems))

    return model, x, z

end


coluna = optimizer_with_attributes(
    Coluna.Optimizer,
    "params" => Coluna.Params(
        solver=Coluna.Algorithm.TreeSearchAlgorithm( ## default branch-and-bound of Coluna
            maxnumnodes=100,
            conqueralg=Coluna.ColCutGenConquer() ## default column and cut generation of Coluna
        ) ## default branch-cut-and-price
    ),
    "default_optimizer" => CPLEX.Optimizer # GLPK for the master & the subproblems
);



# We first define a structure to store the routes:
mutable struct Route
    length::Int # record the length of the route (number of visited customers + 1) 
    path::Vector{Int} # record the sequence of visited customers 
end;

# We can reduce the number of enumerated routes by exploiting the following property.
# Consider two routes starting from the same facility and visiting the same subset of locations (customers).
# These two routes correspond to columns with the same vector of coefficients in master constraints. 
# A solution containing the route with a larger traveled distance (i.e., larger route original cost) is dominated:
# this dominated route can be replaced by the other route without increasing the total solution cost. 
# Therefore, for each subset of locations of a size not exceeding the maximum one, 
# the enumeration procedure keeps only one route visiting this subset, the one with the smallest cost.

# A method that computes the cost of a route:
function route_original_cost(arc_costs, route::Route)
    route_cost = 0.0
    path = route.path
    path_length = route.length
    for i in 1:(path_length-1)
        route_cost += arc_costs[path[i], path[i+1]]
    end
    return route_cost
end;

# This procedure finds a least-cost sequence of visiting the given set of customers starting from a given facility.

function best_visit_sequence(arc_costs, cust_subset, facility_id)
    ## generate all the possible visit orders
    set_size = size(cust_subset)[1]
    all_paths = collect(multiset_permutations(cust_subset, set_size))
    all_routes = Vector{Route}()
    for path in all_paths
        ## add the first index i.e. the facility id 
        enpath = vcat([facility_id], path)
        ## length of the route = 1 + number of visited customers
        route = Route(set_size + 1, enpath)
        push!(all_routes, route)
    end
    ## compute each route original cost
    routes_costs = map(r ->
            (r, route_original_cost(arc_costs, r)), all_routes)
    ## keep only the best visit sequence
    tmp = argmin([c for (_, c) in routes_costs])
    (best_order, _) = routes_costs[tmp]
    return best_order
end;

# We are now able to compute a dominating route for all the possible customers' subsets,
# given a facility id:

using Combinatorics

function best_route_forall_cust_subsets(arc_costs, customers, facility_id, max_size)
    best_routes = Vector{Route}()
    all_subsets = Vector{Vector{Int}}()
    for subset_size in 1:max_size
        subsets = collect(combinations(customers, subset_size))
        for s in subsets
            push!(all_subsets, s)
        end
    end
    for s in all_subsets
        route_s = best_visit_sequence(arc_costs, s, facility_id)
        push!(best_routes, route_s)
    end
    return best_routes
end;

# We store all the information given by the enumeration phase in a dictionary.
# For each facility id, we match a vector of routes that are the best visiting sequences
# for each possible subset of customers.

routes_per_facility = Dict(
    j => best_route_forall_cust_subsets(arc_costs, customers, j, nb_positions) for j in facilities
)

# Our pricing callback must compute the reduced cost of each route, 
# given the reduced cost of the subproblem variables `x` and `z`.
# Remember that subproblem variables `z` are implicitly defined by master representative variables `z`.
# We remark that `z` variables participate only in the objective function.
# Thus their reduced costs are initially equal to the original costs (i.e., objective coefficients)
# This is not true anymore after adding branching constraints and robust cuts involving variables `z`.

# We need methods to compute the contributions to the reduced cost of the `x` and `z` variables:

function x_contribution(route::Route, j::Int, x_red_costs)
    x = 0.0
    visited_customers = route.path[2:route.length]
    for i in visited_customers
        x += x_red_costs["x_$(i)_$(j)"]
    end
    return x
end;

function z_contribution(route::Route, z_red_costs)
    z = 0.0
    for i in 1:(route.length-1)
        current_position = route.path[i]
        next_position = route.path[i+1]
        z += z_red_costs["z_$(current_position)_$(next_position)"]
    end
    return z
end;

# We are now able to write our pricing callback: 

mutable struct Label
    reduced_cost::Float64
    accumulate_capacity::Int
    visited_node::Vector{Int}
end

function update_arc_cost_matrix(selected_parkings)
    ## Reshape the graph, add a common start point 
    ##      and end point for all selected parking

    ## Define start point and end point indices
    start_point = length(facilities) + length(customers) + 1  # Should be 10
    end_point = length(facilities) + length(customers) + 2    # Should be 11
    
    ## Create new arc cost matrix including start and end points
    n_original = size(arc_costs, 1)  # Original number of nodes (9)
    n_new = n_original + 2           # New number of nodes (11: +start +end)
    
    # Initialize new arc cost matrix with infinite costs
    new_arc_costs = fill(Inf, n_new, n_new)
    
    # Copy original arc costs
    new_arc_costs[1:n_original, 1:n_original] = arc_costs
    
    ## single direction from start point to selected parkings
    for parking in selected_parkings
        # Set cost from start point (index 10) to selected parking
        # Using a reasonable cost (could be 0 or based on some distance)
        new_arc_costs[start_point, parking] = 0.0  # Free transition from start to parking
    end
    
    # No arcs FROM selected parkings TO start point (one-directional only)
    for parking in selected_parkings
        new_arc_costs[parking, start_point] = Inf
    end
    
    # No arcs from start point to customers or other facilities
    for i in 1:n_original
        if !(i in selected_parkings)
            new_arc_costs[start_point, i] = Inf
            new_arc_costs[i, start_point] = Inf
        end
    end
    
    # Self-loop at start point should be infinite
    new_arc_costs[start_point, start_point] = Inf
    
    ## single direction from selected parkings to end point
    for parking in selected_parkings
        # Set cost from selected parking to end point
        new_arc_costs[parking, end_point] = 0.0  # Free transition from parking to end
    end
    
    # No arcs FROM end point TO selected parkings (one-directional only)
    for parking in selected_parkings
        new_arc_costs[end_point, parking] = Inf
    end
    
    # No arcs from customers or other facilities to end point
    for i in 1:n_original
        if !(i in selected_parkings)
            new_arc_costs[i, end_point] = Inf
            new_arc_costs[end_point, i] = Inf
        end
    end
    
    # No connection between start and end points
    new_arc_costs[start_point, end_point] = Inf
    new_arc_costs[end_point, start_point] = Inf
    
    # Self-loop at end point should be infinite
    new_arc_costs[end_point, end_point] = Inf
    
    @info "Original arc costs size: $(size(arc_costs))"
    @info "New arc costs size: $(size(new_arc_costs))"
    @info "Start point index: $start_point"
    @info "End point index: $end_point"
    @info "Selected parkings: $selected_parkings"
    @info "Arcs from start point: $([(start_point, p, new_arc_costs[start_point, p]) for p in selected_parkings])"
    @info "Arcs to end point: $([(p, end_point, new_arc_costs[p, end_point]) for p in selected_parkings])"
    
    println("\n=== NEW ARC COST MATRIX ($(n_new)x$(n_new)) ===")
    display(new_arc_costs)
    println("\nNode indices:")
    println("  Facilities: $facilities")
    println("  Customers: $customers") 
    println("  Start point: $start_point")
    println("  End point: $end_point")
    println("  Selected parkings: $selected_parkings")
    
    return new_arc_costs, start_point, end_point
end

function displayLabel(label::Label)
    # println("- Origin parking: $(label.origin_node)")
    # println("  Current node: $(label.current_node)")
    println("- Reduced cost: $(round(label.reduced_cost, digits=2))")
    println("  Accumulated capacity: $(label.accumulate_capacity)")
    # println("  Accumulated duration: $(round(label.accumulated_duration, digits=2))")
    println("  Visit sequence: $(label.visited_node)\n")
end

function dominanceRule(label1, label2)
    rcBool = label1.reduced_cost <= label2.reduced_cost
    capBool = label1.accumulate_capacity <= label2.accumulate_capacity
    # durationBool = label1.accumulated_duration <= label2.accumulated_duration
    ineBool = label1.reduced_cost == label2.reduced_cost && label1.accumulate_capacity == label2.accumulate_capacity # && label1.accumulated_duration == label2.accumulated_duration
    if rcBool && capBool&& !ineBool # && durationBool 
        # @info "Dominance relation found: "
        # printLabel(label1)
        # println("dominates")
        # printLabel(label2)
        # println(label1.visitedNodes, " dominate ", label2.visitedNodes)
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

function extendLabel(current_facility, x_red_costs, z_red_costs, label, next_position)
    ## reduced cost of a sequence equals the sum of two components
    current_position = label.visited_node[end]
    # println("$(current_facility), $current_position, $next_position")

    if next_position in facilities # new_arc_costs[next_position, current_facility] == Inf
        return nothing
    end

    reduced_cost = label.reduced_cost + x_red_costs["x_$(next_position)_$(current_facility)"]
                                      + z_red_costs["z_$(current_position)_$(next_position)"]
    
    accumulate_capacity = label.accumulate_capacity + demands[next_position]
    visited_node = push!(deepcopy(label.visited_node), next_position)
    new_label = Label(reduced_cost, accumulate_capacity, visited_node)

    return new_label
end

function labelling(selected_parking::Vector{Int}, current_facility ,x_red_costs, z_red_costs)

    unprocessedLabels = Dict{Int, Vector{Label}}()
    processedLabels = Dict{Int, Vector{Label}}()
    depotLabels = Vector{Label}()

    start_point = nb_facilities + nb_customers + 1

    active_nodes = vcat(current_facility, customers) # , start_point) #, end_point)
    for node in active_nodes
        unprocessedLabels[node] = Vector{Label}()
        processedLabels[node] = Vector{Label}()
    end

    initial_label = Label(0, 0, [current_facility])
    push!(unprocessedLabels[current_facility], initial_label)

    num_iter_labelling = 1
    while num_iter_labelling < 3 # !isempty(collect(Iterators.flatten(values(unprocessedLabels))))

        println("\n===================Iter $num_iter_labelling===================")

        ## Line 3
        all_labels = collect(Iterators.flatten(values(unprocessedLabels)))
        min_label = all_labels[findmin(l -> l.reduced_cost, all_labels)[2]]
        @info "Selected label:"
        displayLabel(min_label)

        min_idx = findfirst(==(min_label), unprocessedLabels[min_label.visited_node[end]])
        deleteat!(unprocessedLabels[min_label.visited_node[end]], min_idx)

        # Set node in visiting sequence as unreachable
        # TODO

        ## Line 9
        push!(processedLabels[min_label.visited_node[end]], min_label)

        ## Line 4 : Propagation to new node

        ## Line 5
        @info "Propagate labels:"
        current_node = min_label.visited_node[end]

        for node in active_nodes
            new_label = extendLabel(current_facility, x_red_costs, z_red_costs, min_label, node)
            ## Line 6
            if !isnothing(new_label)
                displayLabel(new_label)
                ## Line 7
                node in facilities && push!(depotLabels[node], new_label)
                # node in customers && 
                push!(unprocessedLabels[node], new_label)

                ## Line 8
                if node in customers
                    unprocessedLabels[node], processedLabels[node] = dominanceCheck(unprocessedLabels[node], processedLabels[node])
                end
            end
        end
        num_iter_labelling += 1
    end

    ## Collect result : the labels end in a facility with reduced cost <= 0
    result = []

    # for node in selected_parking
    #     if label.reduced_cost < -1e-8 && length(label.visited_node) > 3
    #         push!(result, label)
    #     end
    # end

    return result

end

function pricing_callback(cbdata)
    ## Get the id of the facility.
    j = BlockDecomposition.indice(BlockDecomposition.callback_spid(cbdata, model))

    ## Retrieve variables reduced costs.
    z_red_costs = Dict(
        "z_$(u)_$(v)" => BlockDecomposition.callback_reduced_cost(cbdata, z[u, v]) for u in locations, v in locations)
    x_red_costs = Dict(
        "x_$(i)_$(j)" => BlockDecomposition.callback_reduced_cost(cbdata, x[i, j]) for i in customers
    # )
    # display(x_red_costs)
    # display(z_red_costs)
    # labelling([1,3], j,  x_red_costs, z_red_costs)
    ## Keep route with minimum reduced cost.
    red_costs_j = map(r -> (
            r,
            x_contribution(r, j, x_red_costs) + z_contribution(r, z_red_costs) # the reduced cost of a route is the sum of the contribution of the variables
        ), routes_per_facility[j]
    )
    min_index = argmin([x for (_, x) in red_costs_j])
    (best_route, min_reduced_cost) = red_costs_j[min_index]

    ## Retrieve the route's arcs.
    best_route_arcs = Vector{Tuple{Int,Int}}()
    for i in 1:(best_route.length-1)
        push!(best_route_arcs, (best_route.path[i], best_route.path[i+1]))
    end
    best_route_customers = best_route.path[2:best_route.length]

    ## Create the solution (send only variables with non-zero values).
    z_vars = [z[u, v] for (u, v) in best_route_arcs]
    # @info "Display z_vars"
    # display(z_vars)
    x_vars = [x[i, j] for i in best_route_customers]
    sol_vars = vcat(z_vars, x_vars)
    sol_vals = ones(Float64, length(z_vars) + length(x_vars))
    sol_cost = min_reduced_cost

    ## Submit the solution to the subproblem to Coluna.
    MOI.submit(model, BlockDecomposition.PricingSolution(cbdata), sol_cost, sol_vars, sol_vals)

    ## Submit the dual bound to the solution of the subproblem.
    ## This bound is used to compute the contribution of the subproblem to the lagrangian
    ## bound in column generation.
    MOI.submit(model, BlockDecomposition.PricingDualBound(cbdata), sol_cost) ## optimal solution

end;

# Test the labelling function with new arc costs
selected_parkings_test = [1, 3]  # Same as the model creation
# println("Testing labelling function...")
# new_arc_costs, start_point, end_point = update_arc_cost_matrix(selected_parkings_test)

# println("\nNew arc cost matrix (11x11):")
# display(new_arc_costs)

# Create the model:
model, x, z = create_model([1,3], coluna, pricing_callback);

# Solve:
JuMP.optimize!(model)

for i in locations
    for j in locations
        value(z[i, j])!=0 && println("z[$i $j]=",value(z[i, j]))
    end
end
println("")

for j in [1,3]
    for i in customers
        value(x[i, j])!=0 && println("x[$i $j]=",value(x[i, j]))
    end
end