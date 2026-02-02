function displayBranchingRule(branchingInfo::BranchingInfo)

    if !isempty(branchingInfo.must_include_combinations)
        print("   - MUST      combination:  ")
        for value in branchingInfo.must_include_combinations 
            print(value, "  ")
        end        
        print("\n")
    end

    if !isempty(branchingInfo.forbidden_combinations)
        print("   - FORBIDDEN combination:  ")
        for value in branchingInfo.forbidden_combinations 
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

    # 本地 helper：尝试在 parking–customer 上分支
    function try_parking_customer!()
        # 只有当 1e 路线服务多个卫星时，parking–customer 分支才有意义
        if length(getServedParking1eRoute(route_1e)) > 1
            for cust in sorted_customers 
                for route in selected_routes 
                    if cust in route
                        # 选择：该路线上起点停车场 + 出现次数多的顾客
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
                                @info "Branch on combination parking-customer: $branchingDecision"
                                return left_branch, right_branch 
                            end
                        end
                    end
                end
            end      
        end
        return nothing
    end

    # 本地 helper：尝试在 customer–customer 上分支
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

    # 统计当前节点已有的两类 branching rule 数量
    count_parking_customer = length(branchingInfo.must_include_combinations) +
                             length(branchingInfo.forbidden_combinations)
    count_customer_customer = length(branchingInfo.must_served_together) +
                              length(branchingInfo.forbidden_served_together)

    # 选择顺序：
    # 1) 如果还没有任何 branching rule：优先 parking–customer，其次 customer–customer
    # 2) 否则，如果 parking–customer 规则多：先 customer–customer，后 parking–customer
    # 3) 否则，如果 customer–customer 规则多：先 parking–customer，后 customer–customer
    # 4) 数量相等：保持原先偏好（先 parking–customer）
    if count_parking_customer == 0 && count_customer_customer == 0
        result = try_parking_customer!()
        if !isnothing(result)
            return result
        end
        return try_customer_customer!()
    elseif count_parking_customer > count_customer_customer
        result = try_customer_customer!()
        if !isnothing(result)
            return result
        end
        return try_parking_customer!()
    elseif count_customer_customer > count_parking_customer
        result = try_parking_customer!()
        if !isnothing(result)
            return result
        end
        return try_customer_customer!()
    else
        # 数量相等时，沿用原先的优先级：先 parking–customer，再 customer–customer
        result = try_parking_customer!()
        if !isnothing(result)
            return result
        end
        return try_customer_customer!()
    end
    
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
