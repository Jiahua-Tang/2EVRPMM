function displayBranchingRule(branchingInfo::BranchingInfo)

    if !isempty(branchingInfo.must_include_combinations)
        print("   - MUST      start-satellite, customer:  ")
        for value in branchingInfo.must_include_combinations 
            print(value, "  ")
        end        
        print("\n")
    end

    if !isempty(branchingInfo.forbidden_combinations)
        print("   - FORBIDDEN start-satellite, customer:  ")
        for value in branchingInfo.forbidden_combinations 
            print(value, "  ")
        end     
        print("\n")
    end

    if !isempty(branchingInfo.must_include_end_combinations)
        print("   - MUST      end-satellite, customer:    ")
        for value in branchingInfo.must_include_end_combinations
            print(value, "  ")
        end
        print("\n")
    end

    if !isempty(branchingInfo.forbidden_end_combinations)
        print("   - FORBIDDEN end-satellite, customer:    ")
        for value in branchingInfo.forbidden_end_combinations
            print(value, "  ")
        end
        print("\n")
    end

    if !isempty(branchingInfo.must_served_together)
        print("   + Customers MUST      served together:  ")
        for value in branchingInfo.must_served_together
            print(value, "  ")
        end
        print("\n")
    end
    
    if !isempty(branchingInfo.forbidden_served_together)
        print("   + Customers FORBIDDEN served together:  ")
        for value in branchingInfo.forbidden_served_together 
            print(value, "  ")
        end
        print("\n")
    end

    if !isempty(branchingInfo.upper_bound_number_2e_routes)
        print("   # Total number of 2e routes cannot EXCEED:   ")
        for value in branchingInfo.upper_bound_number_2e_routes
            print(value, "  ")
        end
        print("\n")
    end

    if !isempty(branchingInfo.lower_bound_number_2e_routes)
        print("   # Total number of 2e routes cannot UNDER:   ")
        for value in branchingInfo.lower_bound_number_2e_routes
            print(value, "  ")
        end
        print("\n")
    end

    if !isempty(branchingInfo.upper_bound_per_satellite)
        print("   # Routes from satellite UPPER bound:  ")
        for (s, ub) in sort(collect(branchingInfo.upper_bound_per_satellite))
            print("sat $s <= $ub  ")
        end
        print("\n")
    end

    if !isempty(branchingInfo.lower_bound_per_satellite)
        print("   # Routes from satellite LOWER bound:  ")
        for (s, lb) in sort(collect(branchingInfo.lower_bound_per_satellite))
            print("sat $s >= $lb  ")
        end
        print("\n")
    end

    println("")
end

function checkExistanceReversedRoute(y, routes)
    result = nothing

    fractional_routes = Set{Vector{Int}}()
    for y_value in y
        push!(fractional_routes, routes[y_value].sequence)
    end
    
    for (_, route) in enumerate(fractional_routes)
        # println(route, "  ", length(route))
        if reverse(route) in fractional_routes && length(route)>3
            result = route
            break
        end
    end

    return result
end

function branchOnReverseRoute(branchingInfo, reverse_route)
    left_branch = deepcopy(branchingInfo)
    right_branch = deepcopy(branchingInfo)
    left_branch.depth += 1
    right_branch.depth += 1
    @info "Start to branch on reverse route $reverse_route"
    
    n = Int(ceil(length(reverse_route)/2))
    routeExistanceMust = Tuple(sort([reverse_route[1], reverse_route[n]])) in branchingInfo.must_include_combinations
    routeExistanceForbidden = Tuple(sort([reverse_route[1], reverse_route[n]])) in branchingInfo.forbidden_combinations
    if !routeExistanceMust && !routeExistanceForbidden
        @info "Branching decision : combination of parking-customer: $(Tuple(sort([reverse_route[1], reverse_route[n]])))"
        println("Branch on reverse route $reverse_route: $(Tuple(sort([reverse_route[1], reverse_route[n]])))")
        push!(left_branch.must_include_combinations, Tuple(sort([reverse_route[1], reverse_route[n]])))
        push!(right_branch.forbidden_combinations, Tuple(sort([reverse_route[1], reverse_route[n]])))
        return left_branch, right_branch
    end

    routeExistanceMust = Tuple(sort([reverse_route[n], reverse_route[end]])) in branchingInfo.must_include_combinations
    routeExistanceForbidden = Tuple(sort([reverse_route[n], reverse_route[end]])) in branchingInfo.forbidden_combinations
    if !routeExistanceMust && !routeExistanceForbidden
        @info "Branching decision : combination of parking-customer: $(Tuple(sort([reverse_route[n], reverse_route[end]])))"
        println("Branch on reverse route $reverse_route: $(Tuple(sort([reverse_route[n], reverse_route[end]])))")
        push!(left_branch.must_include_combinations, Tuple(sort([reverse_route[n], reverse_route[end]])))
        push!(right_branch.forbidden_combinations, Tuple(sort([reverse_route[n], reverse_route[end]])))
        return left_branch, right_branch
    end

    # If no branching decision could be made, return the branches as-is
    return left_branch, right_branch
end

function branchOnCombinationParkingCustomer(route_1e, branchingInfo, y, routes_pool)
    routes = deepcopy(routes_pool)
    left_branch = deepcopy(branchingInfo)
    right_branch = deepcopy(branchingInfo)
    left_branch.depth += 1
    right_branch.depth += 1

    selected_parkings = if route_1e.sequence == [1]
        collect(satellites)
    else
        getServedParking1eRoute(route_1e)
    end

    #region : sort fractional routes
    sorted_fractional_y = sort([r for r in 1:length(y) if 1e-8 < y[r] < 1-1e-8], by = r -> y[r] * (1 - y[r]), rev = true)
    selected_routes = Set{Vector{Int}}()
    for y_value in sorted_fractional_y
        push!(selected_routes, routes_pool[y_value].sequence)
    end
    if !isempty(sorted_fractional_y)
        max_frac_val = y[sorted_fractional_y[1]] * (1 - y[sorted_fractional_y[1]])
        tied_top = [r for r in sorted_fractional_y if abs(y[r] * (1 - y[r]) - max_frac_val) < 1e-8]
        if status_debug
            for (rank, idx) in enumerate(tied_top)
                println("Most fractional route #$rank: $(routes_pool[idx].sequence)  y = $(round(y[idx], digits=3))")
            end            
        end

    end
    #endregion

    #region : calculate and sort customers by appearance in fractional routes
    customers_selected_times = Dict{Int, Int}()
    for cust in customers
        customers_selected_times[cust] = 0
    end
    for route in selected_routes
        for cust in route[2:end-1]
            customers_selected_times[cust] += 1
        end
    end
    sorted_customers = [k for (k, _) in sort(collect(customers_selected_times), by = x -> x[2], rev = true)]
    if status_debug
        println("Sorted customers by fractional route appearances: ", [(c, customers_selected_times[c]) for c in sorted_customers if customers_selected_times[c] > 0])
    end

    if !isempty(sorted_fractional_y)
        most_frac_customers = routes_pool[sorted_fractional_y[1]].sequence[2:end-1]
        sorted_customers = vcat(most_frac_customers, [c for c in sorted_customers if !(c in most_frac_customers)])
    end
    #endregion

    ###################################################################
    # Influence score: sum of y values of fractional routes containing
    # the selected combination
    ###################################################################
    function score_start_parking_customer(parking, cust)
        return sum(y[r] for r in sorted_fractional_y
                   if routes_pool[r].sequence[1] == parking && cust in routes_pool[r].sequence;
                   init = 0.0)
    end

    function score_end_parking_customer(_, cust)
        return sum(y[r] for r in sorted_fractional_y
                   if cust in routes_pool[r].sequence;
                   init = 0.0)
    end

    function score_customer_customer(cust1, cust2)
        return sum(y[r] for r in sorted_fractional_y
                   if cust1 in routes_pool[r].sequence || cust2 in routes_pool[r].sequence;
                   init = 0.0)
    end

    ###################################################################
    # Finder 1: best candidate for (start parking, customer)
    ###################################################################
    function find_start_parking_customer()
        if length(getServedParking1eRoute(route_1e)) > 1
            for cust in sorted_customers
                for route in selected_routes
                    if cust in route
                        local_decision = (route[1], cust)
                        existance1 = local_decision in branchingInfo.must_include_combinations
                        existance2 = local_decision in branchingInfo.forbidden_combinations
                        if !existance1 && !existance2
                            valide = false
                            for route_2 in selected_routes
                                if route_2[1] != route[1] && cust in route_2
                                    valide = true
                                    break
                                end
                            end
                            if valide
                                return (local_decision, score_start_parking_customer(local_decision[1], local_decision[2]))
                            end
                        end
                    end
                end
            end
        end
        return nothing
    end

    ###################################################################
    # Finder 2: best candidate for (customer, customer)
    ###################################################################
    function find_customer_customer()
        for (idx1, cust1) in enumerate(sorted_customers)
            for (_, cust2) in enumerate(sorted_customers[idx1+1:end])
                existance_cust_1 = false
                existance_cust_2 = false
                existance_together = false
                valide = false
                for route in selected_routes
                    if cust1 in route && cust2 in route
                        existance_together = true
                    end
                    if cust1 in route && !(cust2 in route)
                        existance_cust_1 = true
                    end
                    if cust2 in route && !(cust1 in route)
                        existance_cust_2 = true
                    end
                    if existance_cust_1 && existance_cust_2 && existance_together
                        valide = true
                        break
                    end
                end
                if valide
                    local_decision = Tuple(sort([cust1, cust2]))
                    existance1 = local_decision in branchingInfo.must_served_together
                    existance2 = local_decision in branchingInfo.forbidden_served_together
                    if !existance1 && !existance2
                        return (local_decision, score_customer_customer(local_decision[1], local_decision[2]))
                    end
                end
            end
        end
        return nothing
    end

    ###################################################################
    # Finder 3: best candidate for (end parking, customer)
    ###################################################################
    function find_end_parking_customer()
        if !isempty(selected_parkings)
            for cust in sorted_customers
                for route in selected_routes
                    if cust in route[2:end-1]
                        if !(route[end] in selected_parkings)
                            continue
                        end
                        local_decision = (route[end], cust)
                        existance1 = local_decision in branchingInfo.must_include_end_combinations
                        existance2 = local_decision in branchingInfo.forbidden_end_combinations
                        if !existance1 && !existance2
                            valide = false
                            for route_2 in selected_routes
                                if cust in route_2[2:end-1] &&
                                   route_2[end] != route[end] &&
                                   route_2[end] in selected_parkings
                                    valide = true
                                    break
                                end
                            end
                            if valide
                                return (local_decision, score_end_parking_customer(local_decision[1], local_decision[2]))
                            end
                        end
                    end
                end
            end
        end
        return nothing
    end

    ###################################################################
    # Evaluate all three rule types and pick the highest influence score
    # Tiebreak: customer–customer > start-parking–customer > end-parking–customer
    ###################################################################
    pc_result  = find_start_parking_customer()
    cc_result  = find_customer_customer()
    epc_result = find_end_parking_customer()

    pc_score  = isnothing(pc_result)  ? -Inf : pc_result[2]
    cc_score  = isnothing(cc_result)  ? -Inf : cc_result[2]
    epc_score = isnothing(epc_result) ? -Inf : epc_result[2]

    pc_label  = isnothing(pc_result)  ? "N/A" : string(pc_result[1])
    cc_label  = isnothing(cc_result)  ? "N/A" : string(cc_result[1])
    epc_label = isnothing(epc_result) ? "N/A" : string(epc_result[1])
    println("Influence scores:")
    println("   start-sat-cust : $(round(pc_score,  digits=3))  $pc_label")
    println("   cust-cust      : $(round(cc_score,  digits=3))  $cc_label")
    println("   end-sat-cust   : $(round(epc_score, digits=3))  $epc_label")

    best_score = max(pc_score, cc_score, epc_score)
    if best_score == -Inf
        return nothing
    end

    if cc_score == best_score
        local_decision = cc_result[1]
        push!(left_branch.must_served_together, local_decision)
        push!(right_branch.forbidden_served_together, local_decision)
        @info "Branch on combination customer-customer: $local_decision"
        println("Branch on combination customer-customer: $local_decision")
    elseif pc_score == best_score
        local_decision = pc_result[1]
        push!(left_branch.must_include_combinations, local_decision)
        push!(right_branch.forbidden_combinations, local_decision)
        @info "Branch on combination parking-customer: $local_decision"
        println("Branch on combination parking-customer: $local_decision")
    else
        local_decision = epc_result[1]
        push!(left_branch.must_include_end_combinations, local_decision)
        push!(right_branch.forbidden_end_combinations, local_decision)
        @info "Branch on combination end-parking-customer: $local_decision"
        println("Branch on combination end-parking-customer: $local_decision")
    end

    return (left_branch, right_branch)
end
