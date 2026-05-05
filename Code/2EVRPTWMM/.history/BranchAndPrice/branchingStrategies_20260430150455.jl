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

        println("Branch on reverse route $reverse_route: $(Tuple(sort([reverse_route[n], reverse_route[1]])))")
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

    # Determine which satellites (parkings) are relevant for this 1e route
    selected_parkings = if route_1e.sequence == [1]
        collect(satellites)
    else
        getServedParking1eRoute(route_1e)
    end

    #region : sort fractional routes
    sorted_fractional_y = sort([r for r in 1:length(y) if 0 < y[r]], by = r -> y[r] * (1 - y[r]), rev = true)
    selected_routes = Set{Vector{Int}}()
    for y_value in sorted_fractional_y 
        push!(selected_routes, routes_pool[y_value].sequence)
    end
    # println("selected routes: ")
    # for route in selected_routes 
    #     println(route)
    # end
    #endregion

    #region : calculate and sort customers selecte times
    customers_selected_times = Dict{Int, Int}()
    for cust in customers 
        customers_selected_times[cust] = 0
    end
    for route in selected_routes
        for cust in route[2:end-1] 
            customers_selected_times[cust] += 1
        end
    end
    sorted_customers = [k for (k, v) in sort(collect(customers_selected_times), by = x -> x[2], rev = true)]
    #endregion

    ###################################################################
    # Helper 1: try branch on (start parking, customer)
    ###################################################################
    function try_parking_customer!()
        # Need at least two served satellites on the 1e route to make sense
        if length(getServedParking1eRoute(route_1e)) > 1
            for cust in sorted_customers 
                for route in selected_routes 
                    if cust in route
                        # select the combination of start parking of the route and this customer
                        local_decision = (route[1], cust)

                        existance1 = local_decision in branchingInfo.must_include_combinations
                        existance2 = local_decision in branchingInfo.forbidden_combinations
                        
                        if !existance1 && !existance2
                            # check that there exists another fractional route serving the same customer
                            # but starting from a different parking
                            valide = false
                            for route_2 in selected_routes 
                                if route_2[1] != route[1] && cust in route_2
                                    valide = true
                                    break
                                end
                            end
                            
                            if valide                       
                                push!(left_branch.must_include_combinations, local_decision)
                                push!(right_branch.forbidden_combinations, local_decision)
                                @info "Branch on combination parking-customer: $local_decision"
                                return (left_branch, right_branch)
                            end
                        end
                    end
                end
            end      
        end
        return nothing
    end

    ###################################################################
    # Helper 2: try branch on (customer, customer)
    ###################################################################
    function try_customer_customer!()
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
                        push!(left_branch.must_served_together, local_decision)
                        push!(right_branch.forbidden_served_together, local_decision)
                        return (left_branch, right_branch)
                    end                
                end
            end
    end
        return nothing
    end

    ###################################################################
    # Helper 3: try branch on (end parking, customer)
    ###################################################################
    function try_end_parking_customer!()
        # Need at least one relevant end satellite
        if !isempty(selected_parkings)
            for cust in sorted_customers
                for route in selected_routes
                    # consider only true customers inside the route
                    if cust in route[2:end-1]
                        # end of the route must be a selected parking
                        if !(route[end] in selected_parkings)
                            continue
                        end

                        local_decision = (route[end], cust)

                        existance1 = local_decision in branchingInfo.must_include_end_combinations
                        existance2 = local_decision in branchingInfo.forbidden_end_combinations

                        if !existance1 && !existance2
                            # check that there exists another fractional route serving the same
                            # customer but ending at a different selected parking
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
                                push!(left_branch.must_include_end_combinations, local_decision)
                                push!(right_branch.forbidden_end_combinations, local_decision)
                                @info "Branch on combination end-parking-customer: $local_decision"
                                return (left_branch, right_branch)
                            end
                        end
                    end
                end
            end
        end
        return nothing
    end

    ###################################################################
    # Balance between three rule types:
    #   - customer–customer
    #   - start parking–customer
    #   - end parking–customer
    ###################################################################
    num_start_parking_customer = length(branchingInfo.must_include_combinations) +
                                 length(branchingInfo.forbidden_combinations)
    num_end_parking_customer   = length(branchingInfo.must_include_end_combinations) +
                                 length(branchingInfo.forbidden_end_combinations)
    num_customer_customer      = length(branchingInfo.must_served_together) +
                                 length(branchingInfo.forbidden_served_together)

    # Base preference order when counts are equal:
    #   1) customer–customer
    #   2) start parking–customer
    #   3) end parking–customer
    rule_order = [:customer_customer, :start_parking_customer, :end_parking_customer]
    counts = Dict(
        :customer_customer      => num_customer_customer,
        :start_parking_customer => num_start_parking_customer,
        :end_parking_customer   => num_end_parking_customer,
    )

    sorted_rules = sort(rule_order; lt = (r1, r2) -> begin
        c1 = counts[r1]
        c2 = counts[r2]
        if c1 == c2
            # tie-break by base preference order
            findfirst(==(r1), rule_order) < findfirst(==(r2), rule_order)
        else
            c1 < c2
        end
    end)

    for r in sorted_rules
        result = if r == :customer_customer
            try_customer_customer!()
        elseif r == :start_parking_customer
            try_parking_customer!()
        else # :end_parking_customer
            try_end_parking_customer!()
        end

        if !isnothing(result)
            return result
        end
    end

    return nothing
    
    # # # * branch on arc
    # sorted_arc = Dict{Tuple{Int,Int}, Int}()

    # for route in selected_routes
    #     println(route)
    #     if length(route) > 3
    #         for i in 2:(length(route)-2)
    #             a = route[i]
    #             b = route[i+1]
    #             key = minmax(a, b)
    #             println(key)
    #             sorted_arc[key] = get(sorted_arc, key, 0) + 1
    #         end
    #     end
    # end
    # println(sorted_arc)

end
