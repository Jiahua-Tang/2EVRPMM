function displayBranchingRule(branchingInfo::BranchingInfo)

    if !isempty(branchingInfo.must_include_combinations)
        print("   - MUST      combination (start parking, customer):  ")
        for value in branchingInfo.must_include_combinations 
            print(value, "  ")
        end        
        print("\n")
    end

    if !isempty(branchingInfo.forbidden_combinations)
        print("   - FORBIDDEN combination (start parking, customer):  ")
        for value in branchingInfo.forbidden_combinations 
            print(value, "  ")
        end     
        print("\n")
    end

    if !isempty(branchingInfo.must_include_end_combinations)
        print("   - MUST      combination (end   parking, customer):  ")
        for value in branchingInfo.must_include_end_combinations
            print(value, "  ")
        end
        print("\n")
    end

    if !isempty(branchingInfo.forbidden_end_combinations)
        print("   - FORBIDDEN combination (end   parking, customer):  ")
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

    # If no branching decision could be made, return the branches as-is
    return left_branch, right_branch
end

function branchOnCombinationParkingCustomer(route_1e, branchingInfo, y, routes_pool)
    routes = deepcopy(routes_pool)
    selected_parkings = getServedParking1eRoute(route_1e)
    # @info "Start to branch on most fractional route's most visited customer"
    left_branch = deepcopy(branchingInfo)
    right_branch = deepcopy(branchingInfo)
    left_branch.depth += 1
    right_branch.depth += 1

    branchingDecision = nothing
    branchingDecisionFound = false

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

    #region : calculate and sort customers selected times
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

    # 本地 helper：起点停车场 – 顾客（起点必须是 selected_parkings 之一）
    function try_start_parking_customer!()
        if length(selected_parkings) > 1
            for cust in sorted_customers 
                for route in selected_routes 
                    if cust in route && (route[1] in selected_parkings)
                        branchingDecision = (route[1], cust)

                        existance1 = branchingDecision in branchingInfo.must_include_combinations
                        existance2 = branchingDecision in branchingInfo.forbidden_combinations
                        
                        if !existance1 && !existance2
                            valide = false
                            for route_2 in selected_routes 
                                if route_2[1] != route[1] && cust in route_2
                                    valide = true
                                    break
                                end
                            end
                            
                            if valide                       
                                push!(left_branch.must_include_combinations, branchingDecision)
                                push!(right_branch.forbidden_combinations, branchingDecision)
                                @info "Branch on combination start-parking-customer: $branchingDecision"
                                return left_branch, right_branch 
                            end
                        end
                    end
                end
            end
        end
        return nothing
    end

    # 本地 helper：终点停车场 – 顾客（终点必须是 selected_parkings 之一）
    function try_end_parking_customer!()
        if length(selected_parkings) > 1
            for cust in sorted_customers
                for route in selected_routes
                    if cust in route && (route[end] in selected_parkings)
                        branchingDecision = (route[end], cust)

                        existance1 = branchingDecision in branchingInfo.must_include_end_combinations
                        existance2 = branchingDecision in branchingInfo.forbidden_end_combinations

                        if !existance1 && !existance2
                            valide = false
                            for route_2 in selected_routes
                                if route_2[end] != route[end] && cust in route_2
                                    valide = true
                                    break
                                end
                            end

                            if valide
                                push!(left_branch.must_include_end_combinations, branchingDecision)
                                push!(right_branch.forbidden_end_combinations, branchingDecision)
                                @info "Branch on combination end-parking-customer: $branchingDecision"
                                return left_branch, right_branch
                            end
                        end
                    end
                end
            end      
        end
        return nothing
    end

    # 本地 helper：顾客 – 顾客
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
                    branchingDecision = Tuple(sort([cust1, cust2]))
                    existance1 = branchingDecision in branchingInfo.must_served_together
                    existance2 = branchingDecision in branchingInfo.forbidden_served_together

                    if !existance1 && !existance2
                        push!(left_branch.must_served_together, branchingDecision)
                        push!(right_branch.forbidden_served_together, branchingDecision)
                        # @info "Branch on combination customers: $branchingDecision"
                        return left_branch, right_branch
                    end                
                end
            end
        end
        return nothing
    end

    # 选择顺序（组合分支的 Case C）：
    # 1) 先在顾客–顾客上分支
    # 2) 如果不行，再试起点停车场–顾客
    # 3) 再尝试一次顾客–顾客（理论上不会再找到新的，但保持顺序一致）
    # 4) 最后尝试终点停车场–顾客

    result = try_customer_customer!()
    if !isnothing(result)
        return result
    end

    result = try_start_parking_customer!()
    if !isnothing(result)
        return result
    end

    result = try_customer_customer!()
    if !isnothing(result)
        return result
    end

    return try_end_parking_customer!()
    
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
end
