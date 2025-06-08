function displayBranchingRule(branchingInfo::BranchingInfo)

    if !isempty(branchingInfo.special_order_set_must_include)
        print("   ^ Special order set 2e reversed route must include:\n")
        for route in branchingInfo.special_order_set_must_include
            println("       ", route.sequence)
        end
    end

    if !isempty(branchingInfo.special_order_set_forbidden_include)
        print("   ^ Special order set 2e reversed route forbidden include:\n")
        for route in branchingInfo.special_order_set_forbidden_include
            println("       ", route.sequence)
        end
    end

    if !isempty(branchingInfo.must_include_combinations)
        print("   - MUST combination:  ")
        for value in branchingInfo.must_include_combinations 
            print(value, "  ")
        end        
        println("")     
    end

    if !isempty(branchingInfo.forbidden_combinations)
        print("   - FORBIDDEN combination:   ")
        for value in branchingInfo.forbidden_combinations 
            print(value, "  ")
        end
        println("")        
    end

    if !isempty(branchingInfo.must_served_together)
        print("   + Customers MUST     served together:   ")
        for value in branchingInfo.must_served_together
            print(value, "  ")
        end
        println("")    
    end
    
    if !isempty(branchingInfo.forbidden_served_together)
        print("   + Customers FORBIDDEN served together:   ")
        for value in branchingInfo.forbidden_served_together 
            print(value, "  ")
        end
        println("")
    end

    # if !isempty(branchingInfo.must_include_parkings)
    #     print("\n   * Parkings MUST be included:   ")
    #     for value in branchingInfo.must_include_parkings
    #         print(value, "  ")
    #     end
    #     println("")
    # end

    # if !isempty(branchingInfo.forbidden_parkings)
    #     print("   * Parkings CANNOT be included:   ")
    #     for value in branchingInfo.forbidden_parkings 
    #         print(value, "  ")
    #     end
    #     println("")
    # end

    if !isempty(branchingInfo.upper_bound_number_2e_routes)
        print("   # Total number of 2e routes cannot EXCEED:   ")
        for value in branchingInfo.upper_bound_number_2e_routes
            print(value, "  ")
        end
        println("")
    end

    if !isempty(branchingInfo.lower_bound_number_2e_routes)
        print("   # Total number of 2e routes cannot UNDER:   ")
        for value in branchingInfo.lower_bound_number_2e_routes 
            print(value, "  ")
        end
        println("")
    end

    if branchingInfo.depth != 0
        println("   Depth:  ", branchingInfo.depth)
    end
    # println("")
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
        push!(left_branch.must_include_combinations, Tuple(sort([reverse_route[1], reverse_route[n]])))
        push!(right_branch.forbidden_combinations, Tuple(sort([reverse_route[1], reverse_route[n]])))
        return left_branch, right_branch
    end

    routeExistanceMust = Tuple(sort([reverse_route[n], reverse_route[end]])) in branchingInfo.must_include_combinations
    routeExistanceForbidden = Tuple(sort([reverse_route[n], reverse_route[end]])) in branchingInfo.forbidden_combinations
    if !routeExistanceMust && !routeExistanceForbidden
        @info "Branching decision : combination of parking-customer: $(Tuple(sort([reverse_route[n], reverse_route[end]])))"
        push!(left_branch.must_include_combinations, Tuple(sort([reverse_route[n], reverse_route[end]])))
        push!(right_branch.forbidden_combinations, Tuple(sort([reverse_route[n], reverse_route[end]])))
        return left_branch, right_branch
    end
    
    # routeExistanceMust = Tuple(sort([reverse_route[1], reverse_route[2]])) in branchingInfo.must_include_combinations
    # routeExistanceForbidden = Tuple(sort([reverse_route[1], reverse_route[2]])) in branchingInfo.forbidden_combinations
    # if !routeExistanceMust && !routeExistanceForbidden
    #     @info "Branching decision : combination of parking-customer: $(Tuple(sort([reverse_route[1], reverse_route[2]])) )"
    #     push!(left_branch.must_include_combinations, Tuple(sort([reverse_route[1], reverse_route[2]])))
    #     push!(right_branch.forbidden_combinations, Tuple(sort([reverse_route[1], reverse_route[2]])))
    #     return left_branch, right_branch
    # end
    
    # routeExistanceMust = Tuple(sort([reverse_route[end], reverse_route[end-1]])) in branchingInfo.must_include_combinations
    # routeExistanceForbidden = Tuple(sort([reverse_route[end], reverse_route[end-1]])) in branchingInfo.forbidden_combinations
    # if !routeExistanceMust && !routeExistanceForbidden
    #     @info "Branching decision : combination of parking-customer: $(Tuple(sort([reverse_route[end], reverse_route[end-1]])))"
    #     push!(left_branch.must_include_combinations, Tuple(sort([reverse_route[end], reverse_route[end-1]])))
    #     push!(right_branch.forbidden_combinations, Tuple(sort([reverse_route[end], reverse_route[end-1]])))
    #     return left_branch, right_branch
    # end

    routeExistanceMust = false
    routeExistanceForbidden = false
    for route in branchingInfo.special_order_set_must_include 
        if route.sequence == reverse_route
            routeExistanceMust == true
            break
        end
    end
    for route in branchingInfo.special_order_set_forbidden_include
        if route.sequence == reverse_route
            routeExistanceForbidden == true
            break
        end
    end

    if routeExistanceMust || routeExistanceForbidden
        @info "Branching decision : special order set: $(reverse(reverse_route))"
        push!(left_branch.special_order_set_must_include, generate2eRoute(reverse(reverse_route)))
        push!(right_branch.special_order_set_forbidden_include, generate2eRoute(reverse(reverse_route)))
    else
        @info "Branching decision : special order set: $reverse_route"
        push!(left_branch.special_order_set_must_include, generate2eRoute(reverse_route))
        push!(right_branch.special_order_set_forbidden_include, generate2eRoute(reverse_route))
    end


    return left_branch, right_branch
end

function branchOnCombinationParkingCustomer(branchingInfo, y, routes_pool)
    routes = deepcopy(routes_pool)
    # @info "Start to branch on most fractional route's most visited customer"
    left_branch = deepcopy(branchingInfo)
    right_branch = deepcopy(branchingInfo)
    left_branch.depth += 1
    right_branch.depth += 1

    sorted_fractional_y = sort([r for r in 1:length(y) if 0 < y[r]], by = r -> y[r] * (1 - y[r]), rev = true)

    branchingDecision = nothing
    branchingDecisionFound = false

    selected_routes = Set{Vector{Int}}()
    for y_value in sorted_fractional_y 
        push!(selected_routes, routes_pool[y_value].sequence)
    end

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

    for cust in sorted_customers 
        for route in selected_routes 
            if cust in route
                existance1 = branchingDecision in branchingInfo.must_include_combinations
                existance2 = branchingDecision in branchingInfo.forbidden_combinations

                if !existance1 && !existance2
                    valide = false
                    branchingDecision = (route[1], cust)
                    for route_2 in selected_routes 
                        if route_2[1] != route[1] && cust in route_2
                            valide = true
                            break
                        end
                    end
                    
                    if valide                       
                        push!(left_branch.must_include_combinations, branchingDecision)
                        push!(right_branch.forbidden_combinations, branchingDecision)
                        @info "Branch on combination parking-customer: $branchingDecision"
                        return left_branch, right_branch 
                    end
                end
            end
        end
    end
    # for route in selected_routes 
    #     println(route)
    # end
    for (idx1, cust1) in enumerate(sorted_customers)
        for (idx2, cust2) in enumerate(sorted_customers[idx1+1:end])
            existance_cust_1 = false
            existance_cust_2 = false
            existance_together = false            
            valide = false

            for route in selected_routes
                if cust1 in route && cust2 in route
                    existance_together = true
                    # println(cust1, "  ", cust2, "  ", route)
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
                branchingDecision = Tuple(sort([cust1, cust2]))
                existance1 = branchingDecision in branchingInfo.must_served_together
                existance2 = branchingDecision in branchingInfo.forbidden_served_together

                if !existance1 && !existance2
                    push!(left_branch.must_served_together, branchingDecision)
                    push!(right_branch.forbidden_served_together, branchingDecision)
                    @info "Branch on combination customers: $branchingDecision"
                    return left_branch, right_branch
                end                
            end

        end
    end
end
