using  JuMP, CPLEX, DataFrames, DataStructures

using JuMP, CPLEX, Plots, Random, DataStructures, Combinatorics, Printf
import DataFrames
import HiGHS
import Plots
import SparseArrays
import Test  #src
include("Utiles.jl")
include("labelling.jl")
# include("ngPathLabelling.jl")


function displayDualValue(π1, π2, π3, π4)
    print("Dual price π1= [  ")
    for x in π1
        print(round(x, digits=2),"  ")
    end
    print("]\nDual price π2= [  ")
    for x in π2[customers]
        print(round(x, digits=2),"  ")
    end
    print("]\nDual price π3= [  ")
    for x in π3
        print(round(x, digits=2),"  ")
    end
    print("]\nDual price π4= [  ")
    for x in π4
        print(round(x, digits=2),"  ")
    end
    # print("]\nDual price π5= [  ")
    # for x in π5
    #     print(round(x, digits=2),"  ")
    # end
    println("]")
end

function column_generation(filtered_1e_routes, filtered_2e_routes, branchingInfo)
    stopStatus = false
    num_iter = 1
    π1, π2, π3, π4 = 0, 0, 0, 0
    new_columns_generated = Dict{Int, Vector{Label}}()
    for s in satellites 
        new_columns_generated[s] = []
    end
    x = nothing
    y = nothing

    while !(stopStatus) # && num_iter < 6
        println("\n======================Iter CG $num_iter======================")
        execution_time = @elapsed begin
            rlmpResult = solveRestrictedMasterProblem(filtered_1e_routes, filtered_2e_routes, branchingInfo)
        end
        global execution_time_rmp += execution_time
        if !isnothing(rlmpResult)
            ## if RLMP has feasible solution
            π1, π2 = rlmpResult[1], rlmpResult[2]
            π3, π4 = rlmpResult[3], rlmpResult[4]
            x = rlmpResult[5]
            y = rlmpResult[6]
            # for (_, y_value) in enumerate([r for r in 1:length(y) if 0 < y[r]]) 
            #     println("   $(filtered_2e_routes[y_value].sequence)  $(round(y[y_value],digits=2))")
            # end
            # displayDualValue(π1, π2, π3, π4)

            status_2e_subproblem_ternimate = true

            ## Solve subproblem by each parking
            for s in satellites
                if !(s in branchingInfo.forbidden_parkings)
                    execution_time = @elapsed begin
                        subproblem_2e_result = solve_2e_labelling(π1, π2, π3, π4, s, branchingInfo)
                    end
                    global execution_time_subproblem += execution_time
                    if length(subproblem_2e_result) != 0
                        # @info "Number of new routes generated of subproblem parking $s: $(length(subproblem_2e_result))"
                        if !(length(subproblem_2e_result) == length(new_columns_generated[s]) && subproblem_2e_result[1].visitedNodes == new_columns_generated[s][1].visitedNodes)
                            new_columns_generated[s] = subproblem_2e_result
                            for label in subproblem_2e_result 
                                global routes_2e
                                push!(routes_2e, generate2eRoute(label.visitedNodes))
                                push!(filtered_2e_routes, generate2eRoute(label.visitedNodes))
                            end
                            status_2e_subproblem_ternimate = false
                        else
                            # println("Subproblem for parking $s converge (not 0)")
                        end
                    else
                        # println("Subproblem for parking $s converge")
                    end
                end
            end
            ## All subproblems yiled no more negative reduced cost
            if status_2e_subproblem_ternimate
                stopStatus = true
                println("\nStop CG: no more negative RC")
                return nothing , filtered_2e_routes, value.(x), value.(y), rlmpResult[7]
            end
        else
            # @info "RLMP infeasible shown in CG"
            println("\nStop CG: RLMP infeasible")
            stopStatus = true
            return nothing
        end 

        num_iter += 1
    end   

    # # Display all routes grouped by depart parking
    # for s in satellites
    #     for r in routes_2
    #         if r.sequence[1] == s
    #             println(r.sequence, "   Cost=", round(r.cost, digits=2))
    #         end
    #     end
    # end
end

function solveRestrictedMasterProblem(routes_1e, routes_2e_pool, branchingInfo)

    ## Relaxed Restricted Master Problem
    routes_originated_p = Vector{Vector{Int}}()
    for s in satellites
        routes = Vector{Int}()
        for (r,route) in enumerate(routes_2e_pool)
            if route.sequence[1] == s
                push!(routes,r)
            end
        end
        push!(routes_originated_p, routes)
    end

    # for route in routes_2e_pool 
    #     println("Route ", route.sequence, "   Cost=", round(route.cost, digits=2))
    # end

    # println("Number of 2e routes: ", length(routes_2e_pool))
    # println("Number of 1e routes: ", length(routes_1e))
    model = Model(CPLEX.Optimizer)
    @variable(model, x[1:length(routes_1e)]>=0)
    @variable(model, y[1:length(routes_2e_pool)]>=0)
    relax_integrality(model)

    set_silent(model)

    if !isempty(branchingInfo.lower_bound_number_2e_routes)
        lower_bound = maximum(branchingInfo.lower_bound_number_2e_routes) 
        @constraint(model, sum(y[r] for (r, route) in enumerate(routes_2e_pool))>=lower_bound)
    end
    if !isempty(branchingInfo.upper_bound_number_2e_routes)
        upper_bound = minimum(branchingInfo.upper_bound_number_2e_routes) 
        @constraint(model, sum(y[r] for (r, route) in enumerate(routes_2e_pool))<=upper_bound)
    end
    
    special_order_set_must = Set{Int}()
    special_order_set_forbidden = Set{Int}()

    for (idx, route) in enumerate(routes_2e_pool) 
        for r in branchingInfo.special_order_set_must_include 
            if route.sequence == r.sequence
                push!(special_order_set_must, idx)
            end
        end
        for r in branchingInfo.special_order_set_forbidden_include 
            if route.sequence == r.sequence
                push!(special_order_set_forbidden, idx)
            end
        end
    end

    # display(special_order_set_must)
    # display(special_order_set_forbidden)

    if !isempty(special_order_set_must)
        @constraint(model, [r in special_order_set_must], y[r] == 1)
    end
    if !isempty(special_order_set_forbidden)
        @constraint(model, sum(y[r] for r in special_order_set_forbidden) == 0)
    end

    @constraint(model, sync[s in satellites], sum(route.b2out[s] * y[r] for (r, route) in enumerate(routes_2e_pool))
        -nb_vehicle_per_satellite * sum(route.b1[s] * x[r] for (r, route) in enumerate(routes_1e))<=0)
    @constraint(model, custVisit[i in customers], 1 - sum(route.a[i-1-length(satellites)] * y[r] for (r,route) in enumerate(routes_2e_pool)) <= 0 )
    @constraint(model, number2evfixe[s in satellites], sum(route.b2in[s] * y[r] for (r, route) in enumerate(routes_2e_pool)) == sum(route.b2out[s] * y[r] for (r, route) in enumerate(routes_2e_pool)))
    @constraint(model, maxVolumnMM[s in satellites], sum( routes_2e_pool[r].a[i-1-length(satellites
    )]*demands[i]*y[r] for r in routes_originated_p[s-1] for i in customers) - capacity_microhub <= 0)
    @constraint(model, single1eV, sum(x[r] for (r,_) in enumerate(routes_1e))>=1)
    @constraint(model, min2eRoute, sum(y[r] for (r,_) in enumerate(routes_2e_pool))>=ceil(sum(demands)/capacity_2e_vehicle))

    @objective(model, Min, sum(y[r] * route.cost for (r,route) in enumerate(routes_2e_pool)) + 
                        sum(x[r] * route.cost for (r,route) in enumerate(routes_1e)))
    
    execution_time = @elapsed begin
        optimize!(model)
    end
    global solving_rmp_time += execution_time
  
    println("Objective value of master problem is: ",value(objective_value(model)))

    
    status = termination_status(model)
    if status == MOI.OPTIMAL || status == MOI.FEASIBLE_POINT
        # println(status)
        # println("Objective value of master problem is: ",value(objective_value(model)))
        # println("Status of 1e route: ")
        # for (r,route) in enumerate(routes_1e)
        #     if value(x[r]) != 0
        #         print("   x=", round(value(x[r]),digits=2), "   1e Route ", route.sequence, "   Load= ", route.load ,  "   Cost= ", round(route.cost,digits=2), "   Available parking: ")
        #         for (idx, availability) in enumerate(route.b1)
        #             if availability == 1
        #                 print(" ", idx)
        #             end
        #         end
        #         println("")
        #     end 
        # end

        # println("Status of 2e route: ")
        # for (r,route) in enumerate(routes_2e_pool)
        #     if value(y[r]) != 0
        #         println("   y=",round(value(y[r]),digits=2),"   2e Route ", route.sequence, "   Load= ", route.load , "   Cost= ", round(route.cost,digits=2))
        #     end
        # end

        # println("\n   Total number of active satellites: ",sum( route.b1[s] * value(x[r]) for s in satellites for (r,route) in enumerate(routes_1e)))
        # println("   Total number of 2e routes: ", sum(value(y[r]) for r in 1:length(routes_2e_pool)))
        
        # println("Status of satellite")
        # for s in satellites
        #     freight = 0
        #     for (r,route) in enumerate(routes_2)
        #         if route.sequence[1] == s && value(y[r])!=0
        #             freight += route.load
        #         end
        #     end
        #     if freight != 0
        #         println("   parking $s stores $freight freight")
        #     end
        # end
        # println("   Total 2nd route cost = $tc")
        π1 = [dual(sync[i]) for i in keys(sync)]
        π2 = [dual(custVisit[i]) for i in keys(custVisit)]
        π3 = [dual(number2evfixe[i]) for i in keys(number2evfixe)]
        π4 = [dual(maxVolumnMM[i]) for i in keys(maxVolumnMM)]

        π1 = abs.(vcat(0,π1))
        π2 = abs.(vcat(zeros(1+length(satellites)),π2))
        π3 = abs.(vcat(0,π3))
        π4 = abs.(vcat(0,π4))

        # println("tetstest")

        x_value = []
        y_value = []

        for (r, _) in enumerate(routes_1e) 
            push!(x_value, value(x[r]))
        end
        for (r,_) in enumerate(routes_2e_pool)
            push!(y_value, value(y[r]))
        end
        
        return π1, π2, π3, π4, x_value, y_value, objective_value(model)
    else
        return nothing
    end
end