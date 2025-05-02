
mutable struct BranchingInfo
    must_include_combinations::Set{Tuple{Int, Int}}  # combination of parking-customer that must be in a path
    forbidden_combinations::Set{Tuple{Int, Int}}   # combination of parking-customer that cannot be in a path
    
    must_served_together::Set{Tuple{Int, Int}}   # combination of customers that must be in a path
    forbidden_served_together::Set{Tuple{Int, Int}}   # combination of customers that cannot be in a path
    
    must_include_parkings::Set{Int}   # parking that must be used in solution
    forbidden_parkings::Set{Int}   # parking that cannot be used in solution
    
    upper_bound_number_2e_routes::Set{Int} # upper bound of number of total 2e routes
    lower_bound_number_2e_routes::Set{Int} # lower bound of number of total 2e routes
    
    special_order_set_1e::Set{Int}  # set of 1e route where all variables are 0

    depth::Int
end

function getUsedParkings(y, routes)
    selected_y = [r for r in 1:length(y) if y[r] > 0]
    used_parkings = Set{Int}()

    if !isempty(selected_y)
        for y_value in selected_y
            push!(used_parkings, routes[y_value].sequence[1])
        end
    end
    return used_parkings
end

function getNbUsedParkings(usedParkings)
    return length(usedParkings)
end

function displayBranchingRule(branchingInfo::BranchingInfo, routes_1e)
    if !isempty(branchingInfo.must_include_combinations)
        print("\n   - MUST combination:  ")
        for value in branchingInfo.must_include_combinations 
            print(value, "  ")
        end        
    end

    if !isempty(branchingInfo.forbidden_combinations)
        print("\n   - FORBIDDEN combination:   ")
        for value in branchingInfo.forbidden_combinations 
            print(value, "  ")
        end        
    end

    if !isempty(branchingInfo.must_served_together)
        print("\n   + Customers MUST     served together:   ")
        for value in branchingInfo.must_served_together
            print(value, "  ")
        end    
    end
    
    if !isempty(branchingInfo.forbidden_served_together)
        print("\n   + Customers FORBIDDEN served together:   ")
        for value in branchingInfo.forbidden_served_together 
            print(value, "  ")
        end
    end

    if !isempty(branchingInfo.must_include_parkings)
        print("\n   * Parkings MUST be included:   ")
        for value in branchingInfo.must_include_parkings
            print(value, "  ")
        end
    end

    if !isempty(branchingInfo.forbidden_parkings)
        print("\n   * Parkings CANNOT be included:   ")
        for value in branchingInfo.forbidden_parkings 
            print(value, "  ")
        end
    end

    if !isempty(branchingInfo.upper_bound_number_2e_routes)
        print("\n   # Total number of 2e routes cannot EXCEED:   ")
        for value in branchingInfo.upper_bound_number_2e_routes
            print(value, "  ")
        end
    end

    if !isempty(branchingInfo.lower_bound_number_2e_routes)
        print("\n   # Total number of 2e routes cannot UNDER:   ")
        for value in branchingInfo.lower_bound_number_2e_routes 
            print(value, "  ")
        end
    end

    if !isempty(branchingInfo.special_order_set_1e)
        print("\n   & Special order set of 1e route (sum = 0):")
        for value in branchingInfo.special_order_set_1e
            print("   route: $(routes_1e[value].sequence)    served parkings: $(collect(getServedParking1eRoute(routes_1e[value])))")
        end
        
    end

    if branchingInfo.depth != 0
        println("\n   Depth:  ", branchingInfo.depth)
    end
end

function most_costive_points(optimal_1e_route, served_mm::Vector{Int})
    # println("TESTTEST ", optimal_1e_route)
    full_route = optimal_1e_route.sequence
    point_costs = Dict{Int, Float64}()

    # Map each node to its index in full route
    full_index = Dict(n => i for (i, n) in enumerate(full_route))

    # Loop through internal nodes (excluding depots)
    for i in 2:length(served_mm)-1
        node = served_mm[i]
        idx_in_full = full_index[node]

        # Find previous served node in full route
        prev_node = nothing
        for j = idx_in_full-1:-1:1
            if full_route[j] in served_mm
                prev_node = full_route[j]
                break
            end
        end

        # Find next served node in full route
        next_node = nothing
        for j = idx_in_full+1:length(full_route)
            if full_route[j] in served_mm
                next_node = full_route[j]
                break
            end
        end

        # If both neighbors are found, sum arc costs in full route
        if prev_node !== nothing && next_node !== nothing
            idx_start = full_index[prev_node]
            idx_end = full_index[next_node]
            cost = 0.0
            for k in idx_start:idx_end-1
                from = full_route[k]
                to = full_route[k+1]
                cost += arc_cost[from, to]
            end
            point_costs[node] = cost
        end
    end

    # Sort the point_costs dict by descending cost
    sorted_nodes = sort(collect(point_costs), by = x -> -x[2])

    # Separate keys and values if needed
    nodes_sorted = [x[1] for x in sorted_nodes]
    return nodes_sorted
end

function checkExistanceReversedRoute(y, routes)
    result = nothing

    fractional_routes = Set{Vector{Int}}()
    for y_value in y
        push!(fractional_routes, routes[y_value].sequence)
    end
    
    for (_, route) in enumerate(fractional_routes)
        # println(route, "  ", length(route))
        if reverse(route) in fractional_routes
            result = route
            break
        end
    end

    return result
end

function build_branch_order(route::Vector{Int})
    branch_order = Vector{Tuple{Int, Int}}()

    # First: arcs between middle nodes
    for i in 2:(length(route)-2)
        if route[i] < route[i+1]
            push!(branch_order, (route[i], route[i+1])) 
        else
            push!(branch_order, (route[i+1], route[i])) 
        end
    end

    return branch_order
end

function customers_sorted_by_visits(route_to_branch, routes::Set{Vector{Int}})
    customer_counts = Dict{Int, Int}()
    
    for node in route_to_branch
        count = 1  # Start with 1 for route_to_branch itself
        for r in routes
            if node in r[2:end-1] && r != route_to_branch
                count += 1
            end
        end
        customer_counts[node] = count
    end

    # Now sort customers by decreasing number of visits
    sorted_customers = sort(collect(keys(customer_counts)), by = x -> -customer_counts[x])
    return sorted_customers
end

function checkAddingParkingLegality(parking::Int, list_must, list_forbidden)
    ## if add parking into list must include and forbidden, still satisfy constraint
    if !(length(push!(list_must, parking)) <= nb_microhub)
        return false
    end
    if !(length(push!(list_forbidden, parking)) <= nb_parking - nb_microhub)
        return false
    end
    return true
end

function getIncludeExcludeParkingList(branchingInfo)
    list_must_parking = Set{Int}()
    list_forbidden_parking = Set{Int}()
    
    for value in branchingInfo.must_include_combinations 
        push!(list_must_parking, value[1])
    end
    for value in branchingInfo.forbidden_combinations 
        push!(list_forbidden_parking, value[1])
    end
    # display(list_must_parking)
    # println(length(list_must_parking) ,"  nb mm:", nb_microhub)
    # display(list_forbidden_parking)
    # println(length(list_forbidden_parking),"  nb pk-nb mm", nb_parking - nb_microhub)

    limit_status = length(list_must_parking) == nb_microhub || length(list_forbidden_parking) == nb_parking - nb_microhub
    return list_must_parking, list_forbidden_parking, limit_status
end

#=============================================================================#
function split_to_balance_0_5(x::Vector{Float64}, indices::Vector{Int})

    # Sort the original indices by descending value of x[i]
    dict = Dict{Float64, Int}()
    for idx in 1:length(x) 
        dict[x[idx]] = indices[idx]
    end

    display(dict)
    
    keys_list = collect(keys(dict))
    sorted_keys = sort(keys_list, rev=true)  # Sort descending for greedy

    subset1, subset2 = Float64[], Float64[]
    sum1, sum2 = 0.0, 0.0

    for k in sorted_keys
        if sum1 <= sum2
            push!(subset1, k)
            sum1 += k
        else
            push!(subset2, k)
            sum2 += k
        end
    end

    # Return corresponding values
    values1 = [dict[k] for k in subset1]
    values2 = [dict[k] for k in subset2]
    return values1, values2
end

function branchOnSpecialOrderSet(branchingInfo, x, routes_1)
    @info "Start to branch on 1e routes"
    fractional_x = [r for r in 1:length(x) if x[r] > 0]
    value_x = Vector{Float64}()
    for (i,v) in enumerate(fractional_x)
        push!(value_x, x[v])
    end
    subset_1, subset_2 = split_to_balance_0_5(value_x, fractional_x)
    display(subset_1)
    @info ("Branching decision : special order set1: $subset_1, special order set2: $subset_2 ")
    println("Set 1:")
    for ele in subset_1 
        println("     ",(routes_1[ele].sequence))
    end
    println("Set 2:")
    for ele in subset_2
        println("     ",(routes_1[ele].sequence))
    end
    left_branch = deepcopy(branchingInfo)
    right_branch = deepcopy(branchingInfo)

    union!(left_branch.special_order_set_1e, subset_1)
    union!(right_branch.special_order_set_1e, subset_2)
    
    left_branch.depth += 1
    right_branch.depth += 1
    
    return left_branch, right_branch

end

function branchOnParking(branchingInfo, x, y, routes1, routes)
    @info "Start to branch on selection of parking by 1e routes"
    branchingDecisionFound = false
    branchingDecision = nothing
    ## Find most "useless" route by 2e variable value
    x = [r for r in 1:length(x) if x[r] > 0]
    cost_per_parking = Dict{Int, Float64}()
    for y_value in [r for r in 1:length(y) if y[r] > 0]
        parking_idx = routes[y_value].sequence[1]
        cost_per_parking[parking_idx] = get(cost_per_parking, parking_idx, 0.0) + routes[y_value].cost
    end
    for x_value in x
        for p in getServedParking1eRoute(routes1[x_value])
            if !(p in keys(cost_per_parking))
                cost_per_parking[p] = 0
            end
        end
    end
    cost_per_parking = sort(collect(cost_per_parking), by = x -> x[2])

    ## Branch on least cost parking: concerning 1e route
    while !branchingDecisionFound
        for parking in first.(cost_per_parking)
            if !(parking in branchingInfo.must_include_parkings) && !(parking in branchingInfo.forbidden_parkings)
                branchingDecision = parking
                branchingDecisionFound = true
                break
            end
        end
    end
    
    if branchingDecisionFound
        @info ("Branching decision : usage of parking: $branchingDecision")
        left_branch = deepcopy(branchingInfo)
        right_branch = deepcopy(branchingInfo)
    
        push!(left_branch.must_include_parkings, branchingDecision)
        push!(right_branch.forbidden_parkings, branchingDecision)
        
        left_branch.depth += 1
        right_branch.depth += 1
        
        return left_branch, right_branch
    else
        return nothing
    end

end

function branchOnTotalNumber2eRoutes(branchingInfo, totalNumber)
    @info "Start to branch on fraction total number of 2e routes"
    # branchingDecision = nothing
    # branchingDecisionFound = false

    ub = Int(ceil(totalNumber))
    lb = Int(floor(totalNumber))

    @info ("Branching decision : upper / lower bound of total 2e route: $ub / $lb")
    left_branch = deepcopy(branchingInfo)
    right_branch = deepcopy(branchingInfo)

    push!(left_branch.upper_bound_number_2e_routes, ub)
    push!(right_branch.lower_bound_number_2e_routes, lb)
    
    left_branch.depth += 1
    right_branch.depth += 1

    return left_branch, right_branch

end

function branchOnReverseRoute(branchingInfo, reverse_route)
    branchingDecision = nothing
    branchingDecisionFound = false

    @info "Start to branch on reversed route  $reverse_route"
    branch_order = build_branch_order(reverse_route)

    while !branchingDecisionFound && !isempty(branch_order)
        branchingDecision = branch_order[1]
        branch_order = branch_order[2:end]
        if !(branchingDecision in branchingInfo.must_served_together) && !(branchingDecision in branchingInfo.forbidden_served_together)
            branchingDecisionFound = true    
            break             
        end  
    end

    if branchingDecisionFound
        ## branch from middle
        @info ("Branching decision : combination of customers: $branchingDecision")
        left_branch = deepcopy(branchingInfo)
        right_branch = deepcopy(branchingInfo)

        push!(left_branch.must_served_together, branchingDecision)
        push!(right_branch.forbidden_served_together, branchingDecision)

        left_branch.depth += 1
        right_branch.depth += 1

        return left_branch, right_branch   
    else
        return nothing
    end
end

function branchOnRandomCombinationCustomers(branchingInfo, fractional_y, routes)

    @info "Start to branch on random combination of customers"

    branchingDecision = nothing
    branchingDecisionFound = false
    
    visited_customers = Set{Int}()
    for y_value in fractional_y 
        for cust in routes[y_value].sequence[2:end-1]
            push!(visited_customers, cust)
        end
    end
    print("TESTTEST   ")
    display(visited_customers)
end

#=============================================================================#
## Branching strategies on parking-customer
function branchOnAssignmentParkingCustomer(branchingInfo, reverse_route)
    branchingDecision = nothing
    branchingDecisionFound = false

    @info "Start to branch on assignment parking-customer"
    branch_order = Set{Tuple{Int, Int}}()
    list_must_parking , list_forbidden_parking, limit_status = getIncludeExcludeParkingList(branchingInfo)
    ## when the size of indicate parkings reach the limit: 
    ## cannot add any decision concerning new parking 
    ## = can only add decision concerning existing parkings for both branches
    if limit_status
        if checkAddingParkingLegality(reverse_route[1], list_must_parking, list_forbidden_parking)
            push!(branch_order, (reverse_route[1], reverse_route[2]))
        end
        if checkAddingParkingLegality(reverse_route[end], list_must_parking, list_forbidden_parking)
            push!(branch_order, (reverse_route[end], reverse_route[end-1]))
        end
    else
        push!(branch_order, (reverse_route[1], reverse_route[2]))
        push!(branch_order, (reverse_route[end], reverse_route[end-1]))
    end

    while !branchingDecisionFound && !isempty(branch_order)
        branchingDecision = first(branch_order)
        delete!(branch_order, first(branch_order))
        if !(branchingDecision in branchingInfo.must_include_combinations) && !(branchingDecision in branchingInfo.forbidden_combinations)
            branchingDecisionFound = true    
            break             
        end            
    end
    
    if branchingDecisionFound
        @info ("Branching decision : assignment of parking-custmer : $branchingDecision")
        left_branch = deepcopy(branchingInfo)
        right_branch = deepcopy(branchingInfo)

        push!(left_branch.must_include_combinations, branchingDecision)
        push!(right_branch.forbidden_combinations, branchingDecision)

        left_branch.depth += 1
        right_branch.depth += 1

        return left_branch, right_branch    
    else
        return nothing
    end
end

function branchOnArcParkingCustomer(branchingInfo, selected_y, routes)
    @info "Start to branch on arc between parking-customer"
    branchingDecision = nothing
    branchingDecisionFound = false

    sorted_y = sort(selected_y, by = y -> routes[y].cost)
    branch_order = Vector{Tuple{Int, Int}}()

    list_must_parking , list_forbidden_parking, limit_status = getIncludeExcludeParkingList(branchingInfo)
    ## when the size of indicate parkings reach the limit: 
    ## cannot add any decision concerning new parking
    ## = can only add decision concerning existing parkings for both branches
    for y_value in sorted_y
        if limit_status
            if checkAddingParkingLegality(routes[y_value].sequence[1], list_must_parking, list_forbidden_parking)
                for cust in routes[y_value].sequence[2:end-1] 
                    push!(branch_order, (routes[y_value].sequence[1], cust))
                end            
            end   
        else
            for cust in routes[y_value].sequence[2:end-1] 
                push!(branch_order, (routes[y_value].sequence[1], cust))
            end           
        end
    end
    while !branchingDecisionFound && !isempty(branch_order)
        branchingDecision = branch_order[1]
        branch_order = branch_order[2:end]
        if !(branchingDecision in branchingInfo.must_include_combinations) && !(branchingDecision in branchingInfo.forbidden_combinations)
            branchingDecisionFound = true    
            break             
        end  
    end

    if branchingDecisionFound
        ## branch from middle
        @info ("Branching decision : combination of parking-customer: $branchingDecision")
        left_branch = deepcopy(branchingInfo)
        right_branch = deepcopy(branchingInfo)

        push!(left_branch.must_include_combinations, branchingDecision)
        push!(right_branch.forbidden_combinations, branchingDecision)

        left_branch.depth += 1
        right_branch.depth += 1

        return left_branch, right_branch   
    else
        return nothing
    end

end

function branchOnMostFractionalAndVisitedPair(branchingInfo, sorted_fractional_y, routes)
    @info "Start to branch on most fractional route's most visited customer"

    branchingDecision = nothing
    branchingDecisionFound = false

    selected_routes = Set{Vector{Int}}()
    for y_value in sorted_fractional_y 
        push!(selected_routes, routes[y_value].sequence)
    end
    branch_order = Set{Tuple{Int, Int}}()

    list_must_parking , list_forbidden_parking, limit_status = getIncludeExcludeParkingList(branchingInfo)
    ## when the size of indicate parkings reach the limit: 
    ## cannot add any decision concerning new parking 
    ## = can only add decision concerning existing parkings for both branches

    for y_value in sorted_fractional_y
        route_to_branch = routes[y_value].sequence
        if limit_status
            if checkAddingParkingLegality(route_to_branch[1], list_must_parking, list_forbidden_parking)
                sorted_customers = customers_sorted_by_visits(route_to_branch, selected_routes)
                for cust in sorted_customers
                    if cust in customers
                        push!(branch_order, (route_to_branch[1], cust))   
                    end         
                end            
            end  
        else
            sorted_customers = customers_sorted_by_visits(route_to_branch, selected_routes)
            for cust in sorted_customers
                if cust in customers
                    push!(branch_order, (route_to_branch[1], cust))   
                end         
            end       
        end
    end

    while !branchingDecisionFound && !isempty(branch_order)
        branchingDecision = first(branch_order)
        delete!(branch_order, first(branch_order))
        if !(branchingDecision in branchingInfo.must_include_combinations) && !(branchingDecision in branchingInfo.forbidden_combinations)
            branchingDecisionFound = true    
            break             
        end            
    end
    
    if branchingDecisionFound
        @info ("Branching decision : assignment of parking-custmer : $branchingDecision")
        left_branch = deepcopy(branchingInfo)
        right_branch = deepcopy(branchingInfo)

        push!(left_branch.must_include_combinations, branchingDecision)
        push!(right_branch.forbidden_combinations, branchingDecision)

        left_branch.depth += 1
        right_branch.depth += 1

        return left_branch, right_branch    
    else
        return nothing
    end

end


    # cost_per_parking = Dict{Int, Float64}()
    # for y_value in [r for r in 1:length(y) if y[r] == 1]
    #     parking_idx = routes[y_value].sequence[1]
    #     cost_per_parking[parking_idx] = get(cost_per_parking, parking_idx, 0.0) + routes[y_value].cost
    # end
    # while !branchingDecisionFound
    #     branchingDecision = findmin(cost_per_parking)[2]  # Get the key with the minimum value
    #     delete!(cost_per_parking, branchingDecision)      # Remove that key from the dictionary
    #     if !(branchingDecision in branchingInfo.must_include_parkings) && !(branchingDecision in branchingInfo.forbidden_parkings)
    #         branchingDecisionFound = true
    #         break
    #     end
    # end
    # if branchingDecisionFound
    #     @info ("Branching decision : usage of parking : $branchingDecision")
    #     left_branch = deepcopy(branchingInfo)
    #     right_branch = deepcopy(branchingInfo)

    #     push!(left_branch.must_include_parkings, branchingDecision)
    #     push!(right_branch.forbidden_parkings, branchingDecision)

    #     left_branch.depth += 1
    #     right_branch.depth += 1

    #     return left_branch, right_branch
    # end