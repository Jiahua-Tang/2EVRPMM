using JuMP, CPLEX, Plots, Random, DataStructures, Combinatorics
import DataFrames
import HiGHS
import Plots
import SparseArrays
import Test  #src
include("Utiles.jl")
include("column_generation.jl")
include("branching_strategy.jl")

function filter_2e_routes(branchingInfo::BranchingInfo, routes::Vector{Route})
    result = Vector{Route}()
    # displayBranchingRule(branchingInfo)
    
    for route in routes
        valide = true

        if route.sequence[1] in branchingInfo.forbidden_parkings || route.sequence[end] in branchingInfo.forbidden_parkings
            valide = false
        end

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
            push!(result, route)
        end
        # print("$(route.sequence)   $valide")
    end

    return result
end

function calculate_lrp_lower_bound(routes_1e_complete)
    lb_per_route = Dict{Int, Float64}()    
    lower_bounds = Vector{Float64}()

    for (r, route) in enumerate(routes_1e_complete)
        available_parkings = getServedParking1eRoute(route)
        empty_parkings = setdiff!(collect(satellites), available_parkings)
        available_points = sort(collect(union(available_parkings, customers)))
    
        model = Model(CPLEX.Optimizer)
        set_silent(model)
        @variable(model, x[A2, A2], Bin)
        @variable(model, u[customers]>=0,Int)
    
        @constraint(model, [i in customers], sum(x[i,j] for j in A2)==1)
        @constraint(model, [p in empty_parkings], sum(x[p,i] for i in A2) == 0)
        @constraint(model, [p in available_parkings], sum(x[p,j] for j in A2)>=1)
        @constraint(model, [i in available_parkings, j in available_parkings], x[i,j]==0)
        @constraint(model, [i in available_points], sum(x[i,j] for j in A2) == sum(x[j,i] for j in A2))
        @constraint(model, [i in customers, j in customers], u[i] + 1 <= u[j] + length(customers)*(1-x[i,j]))
        
        @objective(model, Min, route.cost + sum(arc_cost[i,j]*x[i,j] for i in A2 for j in A2))
    
        optimize!(model)
    
        status = termination_status(model)
        if status == MOI.OPTIMAL || status == MOI.FEASIBLE_POINT 
            lb_per_route[r] = objective_value(model)
            push!(lower_bounds, objective_value(model))
        end
    end

    return lb_per_route, lower_bounds
end

function solve_lp(routes)
    ## might have unreachable customers
    model = Model(CPLEX.Optimizer)
    set_silent(model)

    @variable(model, 1 >= x[1:length(routes)] >= 0)

    @constraint(model, cov[i in customers], sum(route.a[i-1-length(satellites)]*x[r] for (r,route) in enumerate(routes))>=1)

    @objective(model, Min, sum(route.cost * x[r] for (r,route) in enumerate(routes)))
    
    optimize!(model)

    status = termination_status(model)
    if status == MOI.OPTIMAL || status == MOI.FEASIBLE_POINT

        println("   Result of RLMP\n")
        # for (r, route) in enumerate(routes) 
        #     if value(x[r]) != 0
        #         println("   x=",round(value(x[r]),digits=2),"  ",route.sequence,"   ",round(route.cost, digits=2))
        #     end
        # end
        # println("objective value of RLMP is: ", (objective_value(model)),"\n")

        pi = collect(dual.(cov))
        # display(round.(pi, digits=2))
        pi = vcat(zeros(1+length(satellites)), pi)
        return pi, objective_value(model)
    else
        return nothing
    end
end

function solve_sp(pi, available_parkings, available_points, empty_parkings)
    sp_model = Model(CPLEX.Optimizer)

    # set_silent(sp_model)
    
    @variable(sp_model, x[A2, A2], Bin)
    @variable(sp_model, u[A2], Int)

    @constraint(sp_model, sum(x[i,j] for i in available_parkings for j in customers)==1)
    @constraint(sp_model, sum(x[j,i] for i in available_parkings for j in customers)==1)
    @constraint(sp_model, [i in customers], sum(x[i,j] for j in A2)==sum(x[j,i] for j in A2))
    @constraint(sp_model, [i in empty_parkings], sum(x[i,j] for j in A2)==0)
    # @constraint(sp_model, sum(x[i,j]*demands[i] for i in A2 for j in A2) <= capacity_2e_vehicle)
    # @constraint(sp_model, sum(x[i,j]*arc_cost[i,j] for i in A2 for j in A2)<=maximum_duration_2e_vehicle)
    @constraint(sp_model, [i in customers, j in customers], u[i] + 1 <= u[j] + length(A2)*(1-x[i,j]))

    @objective(sp_model, Min, sum((arc_cost[i,j] - pi[i])*x[i,j] for i in A2 for j in A2))

    optimize!(sp_model)


    status = termination_status(sp_model)
        # println(status)
    if status == MOI.OPTIMAL || status == MOI.FEASIBLE_POINT
        if objective_value(sp_model) < -1e-8
            # println("    ", round.(pi[customers],digits=2))
            # for i in A2 
            #     for j in A2 
            #         if value(x[i,j])!=0                        
            #             println("    x[$i $j] = ", value(x[i,j]))
            #         end
            #     end
            # end
            # println("    RESULT of SUBPROBLEM: ",transformRoute(x),"  reduced cost= ",round(objective_value(sp_model),digits=2))
            return generate2eRoute(transformRoute(x)), objective_value(sp_model) 
        else
            return nothing         
        end
    else
        return nothing
    end 
end

function calculate_aggregated_lower_bound(routes_1e_complete, routes_2e)
    
    lb_per_route = Dict{Int, Float64}()

    for (r, route) in enumerate(routes_1e_complete)
        num_iter = 1
        println("\n", route.sequence, "  ", getServedParking1eRoute(route))
        activate_parkings = getServedParking1eRoute(route)
        # filter 2e routes by 1e route
        subset_routes_2e = Vector{Route}()
        for tour in routes_2e 
            if tour.sequence[1] in activate_parkings && tour.sequence[end] in activate_parkings
                push!(subset_routes_2e, tour)
            end
        end      

        available_parkings = getServedParking1eRoute(route)
        empty_parkings = setdiff!(collect(satellites), available_parkings)
        available_points = sort(collect(union(available_parkings, customers)))
        
        ## Start column generation to decrease lower bound
        result_sp_hist = nothing
        while true
            result_lp = solve_lp(subset_routes_2e)

            if !isnothing(result_lp)
                
                result_sp = solve_sp(result_lp[1], available_parkings, available_points, empty_parkings)
                if !isnothing(result_sp) && result_sp[2]!= result_sp_hist
                    push!(subset_routes_2e, result_sp[1])
                    result_sp_hist = result_sp[2]
                else
                    # println("cost of 1e route ",route.sequence, "is  ", round(route.cost, digits=2), "   \nlinear lb of correspond 2e is : ", round(result_lp[2],digits=2),"\n")
                    # println(route.sequence, "   ",getServedParking1eRoute(route), "   ", route.cost + result_lp[2])
                    
                    lb_per_route[r] = route.cost + result_lp[2]
                    break
                end
            else
                break
            end
        end


        num_iter += 1
        # if num_iter >1
        #     break
        # end
    end
    return lb_per_route
end


function packBranchingNode(route_1e, routes_2e_pool, branchingInfo)
## given a branchingInfo, transform it into a branchingNode structure (add cg result)
    # println("Number of 2e routes:  ",length(routes_2e))
    routes_2e_pool = filter_2e_routes(branchingInfo, routes_2e)
    # println("Number of 2e routes after filtered:  ", length(routes_2e_pool))
    result = column_generation(route_1e, routes_2e_pool, branchingInfo)

    if !isnothing(result)
        y_value = result[4]
        # for (_, y) in enumerate([r for r in 1:length(y_value) if 0 < y_value[r]]) 
        #     println("   $(routes_2e_pool[y].sequence)  $(round(y_value[y],digits=2))")
        # end
        routes_2e_pool = result[2]
        global routes_2e
        for route in result[1]
            push!(routes_2e, route)
        end
        if checkExistanceDummyRoute(y_value, routes_2e_pool)
        ## Dummy route used at the end of column generation, branch can be pruned
            @info "Dummy routes used, exceed upper bound, prune"
            return nothing

        elseif isempty([r for r in 1:length(y_value) if 0 < y_value[r] < 1])
        ## Integer solution found, note as a leaf node
            ## display CG result
            println("   Total number of 2e routes: ", sum(y_value) , ", lb of cg = $(round(result[5],digits=2))")
            for (_, y) in enumerate([r for r in 1:length(y_value) if 0 < y_value[r]]) 
                println("   $(routes_2e_pool[y].sequence)  $(round(y_value[y],digits=2))")
            end
            @info "Integer Solution Found  $(result[5])"
            isLeaf = true
            if result[5] < upperBound
                global  upperBound
                upperBound = result[5]
                global optimalSolution
                optimalSolution = Vector{Route}()
                for (_, y) in enumerate([r for r in 1:length(y_value) if y_value[r]==1]) 
                    push!(optimalSolution, routes_2e_pool[y])
                end                   
            end
        else
        ## Create a child node
            isLeaf = false
        end
        branchingNode = BranchingNode(branchingInfo, result[5], y_value, isLeaf)
        return branchingNode
    else
        @info "RLMP infeasible, prune "
        return nothing
    end

end

function branchAndPriceWithScore(route_1e::Vector{Route}, routes_2e::Vector{Route})
    root_branch = BranchingInfo(Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), Set{Int}(), Set{Int}(), Set{Int}(), Set{Int}(),Set{Route}(),Set{Route}(), 0)
    root_branch.forbidden_parkings = setdiff(Set(satellites), getServedParking1eRoute(route_1e[1]))

    # CG for root node
    println("\n================Iteration 0 of B&P for SP$num_iter $(route_1e[1].sequence)================")
    branchingNode = packBranchingNode(route_1e, filter_2e_routes(root_branch, routes_2e), root_branch)
    node_stack = [branchingNode]
    println("Branching stack contains now $(length(node_stack)) nodes, current upper bound is $(round(upperBound,digits=2))")

    num_iter_sp = 1
    while !isempty(node_stack) # && num_iter_sp < 11
        println("\n================Iteration $num_iter_sp of B&P for SP$num_iter $(route_1e[1].sequence)================")
        
        node = pop!(node_stack)
        # deepest_nodes = Vector{BranchingNode}()
        # max_depth = 0
        # for (idx, value) in enumerate(node_stack) 
        #     if value.branchingInfo.depth == max_depth
        #         push!(deepest_nodes, value)
        #     elseif value.branchingInfo.depth > max_depth
        #         max_depth = value.branchingInfo.depth
        #         deepest_nodes = Vector{BranchingNode}()
        #         push!(deepest_nodes, value)
        #     end
        # end
        # node = deepest_nodes[1]
        # min_idx = 1
        # if length(deepest_nodes) > 1
        #     for (idx, deepest_node) in enumerate(deepest_nodes)
        #         if deepest_node.cgLowerBound < node.cgLowerBound
        #             node = deepest_node
        #             min_idx = idx
        #         end
        #     end
        # end
        # deleteat!(node_stack, min_idx)

        displayBranchingRule(node.branchingInfo)

        if node.cgLowerBound > upperBound
            @info "$(round(node.cgLowerBound, digits=2)), Exceed Upper Bound, prune"
        elseif node.isLeaf
            @info "Leaf Node already"
        else            
        ## start branching
            println("Number of 2e routes:  ",length(routes_2e))
            routes_2e_pool = filter_2e_routes(node.branchingInfo, routes_2e)
            println("Number of 2e routes after filtered:  ", length(routes_2e_pool))

            println("   Total number of 2e routes: ", sum(node.y_value) , ", lb of cg = $(round(node.cgLowerBound,digits=2))")
            for (_, y) in enumerate([r for r in 1:length(node.y_value) if 0 < node.y_value[r]]) 
                println("   $(routes_2e_pool[y].sequence)  $(round(node.y_value[y],digits=2))")
            end

            ## BranchAndPrice
            result = branchingStrategy(node.y_value, routes_2e_pool, node.branchingInfo)
            if !isnothing(result)
                leftBranchingNode = packBranchingNode(route_1e, routes_2e_pool, result[1])
                rightBranchingNode = packBranchingNode(route_1e, routes_2e_pool, result[2])
                if !isnothing(leftBranchingNode)
                    push!(node_stack, leftBranchingNode)
                end
                if !isnothing(rightBranchingNode)
                    push!(node_stack, rightBranchingNode)
                end
            else
                println("No branching decision made")
            end
        end
        println("Branching stack contains now $(length(node_stack)) nodes, current upper bound is $(round(upperBound,digits=2))")
        # for node in node_stack 
        #     displayBranchingNode(node)
        # end
        num_iter_sp += 1    
    end
end

function columnGenerationPer1eRoute(route_1e::Vector{Route}, routes_2e::Vector{Route}, branchingInfo::BranchingInfo)
    
    served_parkings = getServedParking1eRoute(route_1e[1])
    new_routes_generated = Vector{Route}()

    stopStatus = false
    while !stopStatus
        
        rlmpResult = solveRestrictedMasterProblem(route_1e, routes_2e, branchingInfo)
        if !isnothing(rlmpResult)
            π1, π2, π3, π4 = rlmpResult[1], rlmpResult[2], rlmpResult[3], rlmpResult[4]
            # displayDualValue(π1, π2, π3, π4)
            for parking in served_parkings 
                result = solve_2e_MILP(π1, π2, π3, π4, parking, branchingInfo)
        
                if !isnothing(result) && !(isnothing(result[1]))
                    routeAlreadyGenerated = false
                    for route in new_routes_generated 
                        if result[1].sequence == route.sequence
                            routeAlreadyGenerated = true
                            break
                        end
                    end
                    if routeAlreadyGenerated
                        @info "Generted existing route"
                        stopStatus = true
                        ## if there is a dummy route is included in solution, rlmp is considered infeasible
                        for (idx,value) in enumerate(rlmpResult[6])
                            if value != 0 && length(routes_2e[idx].sequence)>length(customers)+1
                                return nothing
                            end
                        end

                        println(route_1e[1].sequence, "   ", round(rlmpResult[7], digits=2), "   ", isempty([r for r in 1:length(rlmpResult[6]) if 0 < rlmpResult[6][r] < 1]))
                        ## return Value of x, Value of y, objective value of RLMP, new routes
                        return rlmpResult[5], rlmpResult[6], rlmpResult[7], routes_2e, new_routes_generated
                        break
                    else
                        push!(routes_2e, result[1])
                        push!(new_routes_generated, result[1])
                        println("---------New route generated: ", result[1].sequence,"   cost=",round(result[1].cost,digits=2),"  rc =$(round(result[2],digits=2))")
                    end

                else
                    @info "No new route generated by subproblem"
                    stopStatus = true
                    ## if there is a dummy route is included in solution, rlmp is considered infeasible
                    for (idx,value) in enumerate(rlmpResult[6])
                        if value != 0 && length(routes_2e[idx].sequence)>length(customers)+1
                            return nothing
                        end
                    end

                    println(route_1e[1].sequence, "   ", round(rlmpResult[7], digits=2), "   ", isempty([r for r in 1:length(rlmpResult[6]) if 0 < rlmpResult[6][r] < 1]))
                    ## return Value of x, Value of y, objective value of RLMP, new routes
                    return rlmpResult[5], rlmpResult[6], rlmpResult[7], routes_2e, new_routes_generated
                    break
                end
            end      
        else
            @info ("No result for subproblem of: ",route_1e[1].sequence)
            return nothing      
        end

    end
end

function branchingStrategy(y, routes_2e,  branchingInfo::BranchingInfo)
    
    left_branch = deepcopy(branchingInfo)
    right_branch = deepcopy(branchingInfo)

    ## Case A: total number of 2e route is fractional
    if !(abs(sum(y)-round(sum(y)))<1e-8)
        @info "Branch on total number of 2e routes:  $(floor(sum(y))), $(ceil(sum(y)))"

        push!(left_branch.lower_bound_number_2e_routes, ceil(sum(y)))
        push!(right_branch.upper_bound_number_2e_routes, floor(sum(y)))
        left_branch.depth += 1
        right_branch.depth += 1

        return left_branch, right_branch
    end

    ## Case B: reversed routes exist
    reversed_route = checkExistanceReversedRoute(sort([r for r in 1:length(y) if 0 < y[r] < 1], by = r -> y[r] * (1 - y[r]), rev = true), routes_2e)
    if !isnothing(reversed_route)
        return branchOnReverseRoute(branchingInfo, reversed_route)
    end

    ## Case C: combination of parking-customer
    result = branchOnCombinationParkingCustomer(branchingInfo, y, routes_2e)
    if !isnothing(result)
        return result
    end
end

function testLMP()
    feasible_1e_routes = Vector{Route}()
    push!(feasible_1e_routes, generate1eRoute([1,2,6,4,1]))
    feasible_2e_routes = generateAllFeasible2eRoute()

    routes_originated_p = Vector{Vector{Int}}()
    for s in satellites 
        routes = Vector{Int}()
        for (r,route) in enumerate(feasible_2e_routes)
            if route.sequence[1] == s
                push!(routes,r)
            end
        end
        push!(routes_originated_p, routes)
    end

    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, 1>=x[1:length(feasible_1e_routes)]>=0)
    @variable(model, 1>=y[1:length(feasible_2e_routes)]>=0)

    @constraint(model, sync[s in satellites], sum(route.b2out[s] * y[r] for (r, route) in enumerate(feasible_2e_routes))
    -nb_vehicle_per_satellite*sum(route.b1[s] * x[r] for (r, route) in enumerate(feasible_1e_routes))<=0)
    @constraint(model, custVisit[i in customers], 1 - sum(route.a[i-1-length(satellites)] * y[r] for (r, route) in enumerate(feasible_2e_routes)) <= 0 )
    @constraint(model, number2evfixe[s in satellites], sum(route.b2in[s] * y[r] for (r, route) in enumerate(feasible_2e_routes)) == sum(route.b2out[s] * y[r] for (r, route) in enumerate(feasible_2e_routes)))
    @constraint(model, maxVolumnMM[s in satellites], sum( feasible_2e_routes[r].a[i-1-length(satellites
    )]*demands[i]*y[r] for r in routes_originated_p[s-1] for i in customers) - capacity_microhub <= 0)
    @constraint(model, single1eV, sum(x[r] for (r,_) in enumerate(feasible_1e_routes))==1)

    @objective(model, Min, sum(y[r] * route.cost for (r, route) in enumerate(feasible_2e_routes)) + sum(x[r] * route.cost for (r, route) in enumerate(feasible_1e_routes)))

    optimize!(model)

    display(termination_status(model))
    println(sum(value.(y)))
    for (r, route) in enumerate(feasible_2e_routes) 
        if value(y[r]) != 0
            println(round(value(y[r]), digits=2),"   ",route.sequence)
        end
    end
end
