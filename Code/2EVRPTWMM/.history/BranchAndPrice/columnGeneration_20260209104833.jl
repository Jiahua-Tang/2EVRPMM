using JuMP, CPLEX, Plots, Random, DataStructures, Combinatorics, Printf
import DataFrames
import HiGHS
import Plots
import SparseArrays
import Test  #src
include("Utiles.jl")
include("../Utiles.jl")

function calculateDualValueRoute(route::Vector{Int}, π1, π2, π3, π4)
    # println(round.(π1, digits=2))
    # println(round.(π2, digits=2))
    # println(round.(π3, digits=2))
    # println(round.(π4, digits=2))
    reduced_cost = 0
    route_distance = 0
    load = 0
    for (idx, node) in enumerate(route[1:end-1]) 
        route_distance += arc_cost[route[idx], route[idx+1]]
        load += demands[node]
    end

    cust_value = 0

    reduced_cost = route_distance
    for cust in route[2:end-1] 
        reduced_cost -= π2[cust]
        cust_value -= π2[cust]
    end
    reduced_cost += π1[route[1]]
    for parking in satellites
        bin = parking == route[1] ? 1 : 0
        bout = parking == route[end] ? 1 : 0
        reduced_cost += π3[parking] * (bin - bout) 
    end
    reduced_cost += π4[route[1]] * load

    # println("distance = $(round(route_distance, digits=2)),  load = $load, rc = $(round(reduced_cost, digits=2))")
    return reduced_cost, route_distance, cust_value

end

function add_2eroute!(route::Int)
    execution_time_ac = @elapsed begin
        route = routes_2e[route]
        ## create column variable
        y = @variable(model, lower_bound = 0.0, upper_bound=1.0)
        global y_vars[route.id] = y

        ## objective coefficients
        global model
        JuMP.set_objective_coefficient(model, y, route.cost)

        ## sync constraint : + b2out[s] * y
        global sync
        @inbounds for s in eachindex(sync) 
            b = route.b2out[s+1]
            # println("$(route.sequence) b2out[$(s+1)] = $b")
            if b != 0
                JuMP.set_normalized_coefficient(sync[s], y, b)
            end
        end

        ## customer-coverage: 1 - sum(a_i * y) <= 0  → coefficient is -a_i
        # println("")
        global custVisit
        @inbounds for i in eachindex(custVisit)
            ai = route.a[i+length(A1)]
            # ai == 1 && println(route.sequence, " a[$(i+length(A1))] = $ai")
            if ai != 0
                set_normalized_coefficient(custVisit[i], y, -ai)
            end
        end

        ## 2E flow balance per satellite: sum(b2in) - sum(b2out) == 0
        global number2evfixe
        @inbounds for s in eachindex(number2evfixe)
            coeff = route.b2in[s+1] - route.b2out[s+1]
            if coeff != 0
                set_normalized_coefficient(number2evfixe[s], y, coeff)
            end
        end

        ## microhub capacity at origin s0: ∑ a_i * demand_i * y - cap ≤ 0  (only for origin)
        load = 0
        for i in route.sequence 
            load += demands[i]
        end

        global maxVolumnMM
        @inbounds for s in eachindex(maxVolumnMM) 
            b = route.b2out[s+1]
            if b != 0
                JuMP.set_normalized_coefficient(maxVolumnMM[s], y, load)
            end
        end

        global globalUpperBound
        global globalLowerBound
        JuMP.set_normalized_coefficient(globalLowerBound, y, -1)  # For sum(y) ≥ bound: sum(-y) ≤ -lowerbound
        JuMP.set_normalized_coefficient(globalUpperBound, y, 1)   # For sum(y) ≤ bound: sum(y) ≤ upperbound
    end
    global execution_time_add_columns += execution_time_ac
    return y
end


mutable struct Label
    current_node::Int
    reduced_cost::Float64
    accumulated_capacity::Int
    accumulated_duration::Float64
    earliest_time::Float64
    M::Vector{Int} # * equals to 1 if the extension to new node_i will cause a forbidden cycle
    visitedSequence::Vector{Int}
end

function get_neighbours(rho::Int)
    neighbours = Dict{Int, Vector{Int}}()

    # * for satellite node, cycle is not allowed
    for node in satellites 
        neighbours[node] = collect(A2)
    end

    # * for customer node, define neighborhoods by rho
    for node in customers
        # Get all possible destinations except the node itself
        candidates = [(other, arc_cost[node, other]) for other in customers if other != node]
        
        # Sort by cost
        sorted_candidates = sort(candidates, by = x -> x[2])
        
        # Take top size_neighbour neighbors
        ng_neighbours = vcat(node, [x[1] for x in Iterators.take(sorted_candidates, rho)])
        
        neighbours[node] = ng_neighbours
    end

    # for node in A2 
    #     println("neighbours of $node :  $(neighbours[node])" )
    # end

    return neighbours
end
#region
# function ng_labelling(π1, π2, π3, π4, selected_parkings, branchingInfo)
#     # * For a label, when extending to new node, if there is a cycle detected
#     # * check if in the cycle, there exist a node doesn't belong to neighbours of node
#     # * if yes, label created, otw delete the label

#     ## Case C : combination of parking - customer
#     ## Case D : combination of customer - customer

#     # * Initialization
#     unprocessedLabels = Dict{Int, Vector{Label}}()
#     processedLabels = Dict{Int, Vector{Label}}()
#     depotLabels = Vector{Label}()
#     active_nodes = vcat(collect(selected_parkings), customers)
#     result = []

#     for node in active_nodes
#         unprocessedLabels[node] = Vector{Label}()
#         processedLabels[node] = Vector{Label}()
#     end

#     for parking in selected_parkings
#         rc = π1[parking] - π3[parking] 
#         visitedNodes = zeros(Int, length(A2)+1)
#         visitedNodes[parking] = 1
#         l = Label(parking, rc, 0, 0, 0, [parking], [parking])
#         push!(unprocessedLabels[parking], l)
#     end

#     # * Start labeling algo
#     num_iter_labelling = 1
#     while !isempty(collect(Iterators.flatten(values(unprocessedLabels)))) # && num_iter_labelling < 11
#         # println("\n======iter of labeling : $num_iter_labelling======")
#         # println("$(length(collect(Iterators.flatten(values(unprocessedLabels))))) unprocecssed labels")
#         # * selected unprocessed label : arg min{reduced cost}
#         all_labels = collect(Iterators.flatten(values(unprocessedLabels)))
#         min_label = all_labels[findmin(l -> l.reduced_cost, all_labels)[2]]
#         min_idx = findfirst(==(min_label), unprocessedLabels[min_label.current_node])
#         deleteat!(unprocessedLabels[min_label.current_node], min_idx)

#         #region : display labels
#         # println("\n===Selected min label: ")
#         # displayLabel(min_label)
#         # println("\n$(length(depotLabels)) depot labels")

#         # println("\n===Unprocessed label: ")
#         # for node in active_nodes
#             # if !isempty(unprocessedLabels[node])
#                 # println("$(length(unprocessedLabels[node])) unprocessed labels of node $node")
#         #         print("-")
#         #     end
#         #     for (idx, ele) in enumerate(unprocessedLabels[node])
#         #         if idx > 1
#         #             print(" ")
#         #         else
#         #             print("")
#         #         end
#         #        displayLabel(ele)
#         #     end

#         # end
#         #endregion

#         #region : branching rule check
#         if min_label.current_node in satellites && !isempty(branchingInfo.forbidden_combinations)
#             # TODO : delete a parking node from active nodes
#         end
#         if min_label.current_node in customers && !isempty(branchingInfo.forbidden_served_together)
#             # TODO : delete a customer node from active nodes
#         end
#         #endregion

#         # * extend label
#         for node in active_nodes
#             new_label = extendLabel_v2(π2, π3, π4, min_label, node)
#             if !isnothing(new_label)
#                 # ? Check depot labels dominance relation?
#                 # * push depot route with negative reduced cost into result pool
#                 if node in satellites && new_label.reduced_cost < -1e-8 && length(new_label.visitedSequence)>2
#                     # println("depot label dominance check")
#                     for label in depotLabels 
#                         dominance_result = dominanceCheckSingle(new_label, label)
#                         if dominance_result == 1
#                         ## new label is dominated by a existed depot label
#                         elseif dominance_result == 2
#                         ## a existed depot label is dominated by the new label
#                             idx_label = findfirst(==(label), depotLabels)
#                             deleteat!(depotLabels, idx_label)
#                             push!(depotLabels, new_label)
#                         else
#                             push!(depotLabels, new_label)
#                         end                            
#                     end
#                     push!(result, new_label)

#                 elseif node in customers
#                     # * check dominance

#                     new_label_is_dominated = false
#                     ## Check if new label is dominated by a processed label
#                     # println("cust label dominance check - processed label")
#                     for label in processedLabels[node]
#                         if dominanceCheckSingle(new_label, label) == 1
#                             new_label_is_dominated = true
#                             break
#                         end
#                     end
#                     ## Check if new label is dominated by or dominates a unprocessed label

#                     # println("cust label dominance check - unprocessed label")
#                     if !new_label_is_dominated
#                         for label in unprocessedLabels[node]
#                             dominance_result = dominanceCheckSingle(new_label, label)
#                             if dominance_result == 1
#                                 ## new label is dominated by a unprocessed label
#                                 new_label_is_dominated = true
#                                 break
#                             elseif dominance_result == 2
#                                 ## new label dominate a unprocessed label
#                                 # println("a unprocessed label is dominated")
#                                 min_idx = findfirst(==(label), unprocessedLabels[node])
#                                 deleteat!(unprocessedLabels[node], min_idx)
#                             end
#                         end
#                     end
#                     ## Save label to unprocessed labels set
#                     if !new_label_is_dominated 
#                         push!(unprocessedLabels[node], new_label)
#                     end    
#                 end
#             end
#         end
#         num_iter_labelling += 1

#         # println("$(length(collect(Iterators.flatten(values(unprocessedLabels))))) unprocecssed labels")
#     end
#     println("number of iteration : $num_iter_labelling, number of routes : $(length(result)), number of rest labels : $(length(depotLabels))")
#     return result
# end

# function basic_labelling(π1, π2, π3, π4, selected_parkings, branchingInfo)

#     ## Case C : combination of parking - customer
#     ## Case D : combination of customer - customer

#     # * Initialization
#     unprocessedLabels = Dict{Int, Vector{Label}}()
#     processedLabels = Dict{Int, Vector{Label}}()
#     depotLabels = Vector{Label}()
#     result = []

#     active_nodes = vcat(collect(selected_parkings), customers)

#     for node in active_nodes
#         unprocessedLabels[node] = Vector{Label}()
#         processedLabels[node] = Vector{Label}()
#     end

#     for parking in selected_parkings
#         rc = π1[parking] - π3[parking] 
#         visitedNodes = zeros(Int, length(A2)+1)
#         visitedNodes[parking] = 1
#         l = Label(parking, parking, rc, 0, 0, visitedNodes, [parking])
#         push!(unprocessedLabels[parking], l)
#     end

#     num_iter_labelling = 1
#     while !isempty(collect(Iterators.flatten(values(unprocessedLabels)))) && num_iter_labelling < 11
#         # * selected unprocessed label : arg min{reduced cost}
#         all_labels = collect(Iterators.flatten(values(unprocessedLabels)))
#         min_label = all_labels[findmin(l -> l.reduced_cost, all_labels)[2]]
#         min_idx = findfirst(==(min_label), unprocessedLabels[min_label.current_node])
#         deleteat!(unprocessedLabels[min_label.current_node], min_idx)

#         #region : display labels
#         for node in active_nodes
#             if !isempty(unprocessedLabels[node])
#                 print("-")
#             end
#             for (idx, ele) in enumerate(unprocessedLabels[node])
#                 if idx > 1
#                     print(" ")
#                 else
#                     print("")
#                 end
#                displayLabel(ele)
#             end
#         end
#         #endregion

#         #region : branching rule check
#         if min_label.current_node in satellites && !isempty(branchingInfo.forbidden_combinations)
#             # TODO : delete a parking node from active nodes
#         end
#         if min_label.current_node in customers && !isempty(branchingInfo.forbidden_served_together)
#             # TODO : delete a customer node from active nodes
#         end
#         #endregion

#         # * extend label
#         for node in active_nodes
#             new_label = extendLabel_v2(π2, π3, π4, min_label, node)
#             if !isnothing(new_label)
#                 # * check dominance
#                 new_label_is_dominated = false
#                 ## Check if new label is dominated by a processed label
#                 for label in processedLabels[node]
#                     if dominanceCheckSingle(new_label, label) == 1
#                         new_label_is_dominated = true
#                         break
#                     end
#                 end
#                 ## Check if new label is dominated by or dominates a unprocessed label
#                 if !new_label_is_dominated
#                     for label in unprocessedLabels[node]
#                         if dominanceCheckSingle(new_label, label) == 1
#                             ## new label is dominated by a unprocessed label
#                             new_label_is_dominated = true
#                             break
#                         elseif dominanceCheckSingle(new_label,label) == 2
#                             ## new label dominate a unprocessed label
#                             # println("a unprocessed label is dominated")
#                             min_idx = findfirst(==(label), unprocessedLabels[node])
#                             deleteat!(unprocessedLabels[node], min_idx)
#                         end
#                     end
#                 end
#                 # println(new_label_is_dominated)
#                 if !new_label_is_dominated 
#                     push!(unprocessedLabels[node], new_label)
#                 end
#             end
#         end
#         num_iter_labelling += 1
#     end

#     return result
       

# end

# mutable struct Label
#     origin_node::Int
#     current_node::Int
#     reduced_cost::Float64
#     accumulated_capacity::Int
#     accumulated_duration::Float64
#     visitedNodes::Vector{Int}
# end
#endregion
function pricing(selected_parkings, routes_2e_pool::Vector{Int}, π1, π2, π3, π4, π5, π6, branchingInfo::BranchingInfo) 

    #region : dual multiplier verification
    # println("π1=  ", round.(π1, digits=2))
    # println("π2=  ", round.(π2, digits=2))
    # println("π3=  ", round.(π3, digits=2))
    # println("π4=  ", round.(π4, digits=2))

    # for route in routes_2e_pool
    #     rc, dist, cust_v = calculateDualValueRoute(routes_2e[route].sequence, π1, π2, π3, π4)
    #     println("route exist:  $(routes_2e[route].sequence),   $(round(rc,digits=2)),   $(round(dist, digits=2)),   $(round(cust_v, digits=2))")
    # end
    #endregion

    # calculateLowerBoundSSP(π1, π2, π3, π4, selected_parkings)

    # execution_time = @elapsed begin
        execution_time_sp = @elapsed begin
            new_columns_found = ng_labelling_optimized(π1, π2, π3, π4, π5, π6,selected_parkings, branchingInfo)
        end
        global execution_time_subproblem += execution_time_sp
        # println("execution time of labelling: $(round(execution_time_sp, digits=2))s")

        # println("TEST return result from labelling")
        # new_routes_from = length(routes_2e_pool) + 1

    # end
    # println(new_routes_generated, "  ", length(routes_2e_pool[new_routes_from:end]))

    return new_columns_found
end

function calculateLowerBoundSSP(π1, π2, π3, π4, selected_parkings)
    ssp_model = Model(CPLEX.Optimizer)
    set_silent(ssp_model)

    @variable(ssp_model, x[A2, A2]>=0, Bin)
    @variable(ssp_model, planning_horizon >= u[i in A2] >= 0 )

    @constraint(ssp_model, [i in setdiff(satellites, selected_parkings)], sum(x[i,j] for j in customers)== 0)
    @constraint(ssp_model, [i in setdiff(satellites, selected_parkings)], sum(x[j,i] for j in customers)== 0)

    @constraint(ssp_model, sum(x[i,j] for i in selected_parkings, j in customers) == 1)
    @constraint(ssp_model, sum(x[j,i] for i in selected_parkings, j in customers) == 1)

    @constraint(ssp_model, [i in customers], sum(x[i,j] for j in A2) == sum(x[j,i] for j in A2))
    @constraint(ssp_model, [i in customers], sum(x[i,j] for j in A2) <= 1)

    @constraint(ssp_model, [i in customers, j in customers; i != j], u[j] >= u[i] + arc_cost[i,j] - planning_horizon*(1-x[i,j]))
    @constraint(ssp_model, [i in selected_parkings, j in customers], u[j] >= u[i] + arc_cost[i,j] - planning_horizon*(1-x[i,j]))
    @constraint(ssp_model, [i in customers], u[i] >= time_window[i][1])
    @constraint(ssp_model, [i in customers], u[i] <= time_window[i][2])

    @constraint(ssp_model,  sum(demands[i] * x[i,j] for i in customers, j in A2) <= capacity_2e_vehicle)

    @objective(ssp_model, Min, sum(arc_cost[i,j] * x[i,j] for i in A2 ,j in A2) 
                             + sum(π1[i] * x[i,j] for i in selected_parkings, j in customers) 
                             - sum(π2[i] * x[i,j] for i in customers, j in A2) 
                             + sum(π3[i] * x[i,j] for i in selected_parkings, j in customers) 
                             - sum(π3[i] * x[j,i] for i in selected_parkings, j in customers))
    optimize!(ssp_model)

    for i in A2, j in A2 
        if value(x[i,j]) != 0
            println("x[$i, $j]=",round(value(x[i,j]), digits=2))
        end
    end

    # for i in customers 
    #     println("u[$i]=$(round(value(u[i]), digits=2))")
    # end

    println(round(objective_value(ssp_model), digits=2), "  ", sum(demands[i] * value(x[i,j]) for i in customers, j in A2))
end

function extendLabel_v2(π2, π3, π4, label::Label, next_node::Int)
    ## Detect forbidden cycle
    if (next_node in label.M)
        # println("forbidden cycle detected: $(label.visitedSequence) $(label.M)-> $next_node")
        return nothing
    end
    ## Update reduced cost
    reduced_cost = label.reduced_cost + arc_cost[label.current_node, next_node]
    if next_node in customers
        reduced_cost = reduced_cost - π2[next_node] +
                         π4[label.visitedSequence[1]] * demands[next_node]
    end
    if next_node in satellites
        reduced_cost += π3[next_node]       
    end

    ## Update accumulated capacity
    accumulated_capacity = label.accumulated_capacity + demands[next_node]
    if accumulated_capacity > capacity_2e_vehicle
        return nothing
    end

    ## Update accumulated duration
    accumulated_duration = label.accumulated_duration + arc_cost[label.current_node, next_node]
    if accumulated_duration > maximum_duration_2e_vehicle
        return nothing
    end

    ## Update earliest feasible time
    earliest_feasible_time = time_window[next_node][1] > label.earliest_time + arc_cost[label.current_node, next_node] ? time_window[next_node][1] : label.earliest_time + arc_cost[label.current_node, next_node]
    if earliest_feasible_time > time_window[next_node][2]
        return nothing
    end

    ## Update set of M
    new_M = deepcopy(label.M)
    new_M = (new_M ∪ next_node) ∩ neighbours[next_node]

    ## Update visited sequence
    visitedSequence = push!(deepcopy(label.visitedSequence), next_node)
    
    new_label = Label(next_node, reduced_cost, 
                      accumulated_capacity, accumulated_duration, 
                      earliest_feasible_time, new_M, visitedSequence)
    # displayLabel(new_label)
    return new_label
end

function dominanceRule(label1, label2)
    # ? dominance relation exist between routes start from different depot ?
    # * function check if label 1 dominates label 2
    eq1 = label1.reduced_cost == label2.reduced_cost
    if label1.reduced_cost > label2.reduced_cost
        return nothing
    end
    eq2 = label1.accumulated_capacity == label2.accumulated_capacity
    if label1.accumulated_capacity > label2.accumulated_capacity
        return nothing
    end
    eq3 = label1.accumulated_duration == label2.accumulated_duration
    if label1.accumulated_duration > label2.accumulated_duration
        return nothing
    end
    eq4 = label1.earliest_time == label2.earliest_time
    if label1.earliest_time > label2.earliest_time
        return nothing
    end
    eq5 = label1.M == label2.M
    if !(label1.M ⊆ label2.M)
        return nothing
    end

    if eq1 && eq2 && eq3 && eq4 && eq5
        return nothing
    end
    return label2
end

function dominanceCheckSingle(l1, l2)
    result = dominanceRule(l1, l2)
    if !isnothing(result)
        ## l1 dominate l2
        return 2
    else
        result = dominanceRule(l2, l1)
        if !isnothing(result)
            ## l2 dominate l1
            return 1
        else
            ## no dominance relation exsit
            return nothing
        end
    end
end

# function dominanceCheck(unprocessedLabelsList, processedLabelsList)
#     unprocessedLabels = deepcopy(unprocessedLabelsList)
#     processedLabels = deepcopy(processedLabelsList)

#     idx_dominated = []
#     for (idx1, label1) in enumerate(unprocessedLabels) 
        
#         for (idx2, label2) in enumerate(unprocessedLabels)
#             if idx2 > idx1
#                 result = dominanceCheckSingle(label1, label2)
#                 if !isnothing(result)
#                     result == 1 && push!(idx_dominated, idx1)
#                     result == 2 && push!(idx_dominated, idx2)
#                 end
#             end
#         end

#         for (idx3, label3) in enumerate(processedLabels) 
#             result = dominanceCheckSingle(label1, label3)
#             if !isnothing(result)
#                 result == 1 && push!(idx_dominated, idx1)
#                 result == 2 && deleteat!(processedLabels, idx3)
#             end
#         end
#     end
#     deleteat!(unprocessedLabels, unique(sort(idx_dominated)))
#     return unprocessedLabels, processedLabels
# end

function displayLabel(label::Label)
    println("- Current node: $(label.current_node)")
    println("  Reduced cost: $(round(label.reduced_cost, digits=2))")
    println("  Accumulated capacity: $(label.accumulated_capacity)")
    # println("  Accumulated duration: $(round(label.accumulated_duration, digits=2))")
    println("  Earliest feasible time: $(round(label.earliest_time,digits=2))")
    println("  Set of M: $(label.M)")
    println("  Visited sequence: $(label.visitedSequence)")
end

#===================================================================================
    OPTIMIZED NG-ROUTE LABELING IMPLEMENTATION
    
    Key optimizations:
    1. PriorityQueue for O(log n) label selection instead of O(n)
    2. BitSet for M (memory set) - much faster set operations
    3. Eliminated unnecessary deepcopy operations
    4. Fixed dominance logic with early exits
    5. Pre-filter feasible nodes before extension
    6. Batch array operations
===================================================================================#

# Optimized Label struct using BitSet for memory set
mutable struct LabelOptimized
    current_node::Int
    reduced_cost::Float64
    accumulated_capacity::Int
    accumulated_duration::Float64
    earliest_time::Float64
    M::BitSet  # Uses BitSet instead of Vector{Int} for faster set operations
    visitedSequence::Vector{Int}
end

# Convert between Label types if needed
function to_optimized_label(label::Label)
    return LabelOptimized(
        label.current_node,
        label.reduced_cost,
        label.accumulated_capacity,
        label.accumulated_duration,
        label.earliest_time,
        BitSet(label.M),
        label.visitedSequence
    )
end

function to_standard_label(label::LabelOptimized)
    return Label(
        label.current_node,
        label.reduced_cost,
        label.accumulated_capacity,
        label.accumulated_duration,
        label.earliest_time,
        collect(label.M),
        label.visitedSequence
    )
end

"""
    extendLabel_optimized(π2, π3, π4, label, next_node, neighbours)

Optimized label extension with BitSet operations and no unnecessary copies.
"""
function extendLabel_optimized(π2, π3, π4, label::LabelOptimized, next_node::Int, neighbours::Dict{Int, BitSet})
    # Detect forbidden cycle - O(1) with BitSet

    if next_node in label.M
        return nothing
    end
    
    # Update reduced cost
    reduced_cost = label.reduced_cost + arc_cost[label.current_node, next_node]
    if next_node in customers
        reduced_cost = reduced_cost - π2[next_node] +
                         π4[label.visitedSequence[1]] * demands[next_node]
    end

    if next_node in satellites
        reduced_cost -= π3[next_node]       
    end
    
    # Update accumulated capacity
    accumulated_capacity = label.accumulated_capacity + demands[next_node]
    if accumulated_capacity > capacity_2e_vehicle
        return nothing
    end


    # Update accumulated duration
    accumulated_duration = label.accumulated_duration + arc_cost[label.current_node, next_node]
    if accumulated_duration > maximum_duration_2e_vehicle
        return nothing
    end

    # Update earliest feasible time
    earliest_feasible_time = time_window[next_node][1] > label.earliest_time + arc_cost[label.current_node, next_node] ? 
                            time_window[next_node][1] : label.earliest_time + arc_cost[label.current_node, next_node]
    if earliest_feasible_time > time_window[next_node][2]
        return nothing
    end

    # Update set of M - optimized with BitSet operations (O(n) but with BitSet it's much faster)
    # new_M = (M ∪ next_node) ∩ neighbours[next_node]
    new_M = intersect(union(label.M, next_node), neighbours[next_node])

    # Update visited sequence - use vcat to avoid explicit deepcopy
    visitedSequence = vcat(label.visitedSequence, next_node)
    
    new_label = LabelOptimized(next_node, reduced_cost, 
                      accumulated_capacity, accumulated_duration, 
                      earliest_feasible_time, new_M, visitedSequence)
    # if label.visitedSequence == [6, 10, 15] && next_node == 14
    #     println("test::  ", new_label)        
    # end


    return new_label
end

"""
    dominanceRule_optimized(label1, label2)

Optimized dominance check with early exits and BitSet operations.
Returns label2 if label1 dominates label2, 
        nothing if label1 doesn't dominate label2
"""
function dominanceRule_optimized(label1::LabelOptimized, label2::LabelOptimized)
    # Early exit checks with short-circuit evaluation
    if label1.reduced_cost > label2.reduced_cost
        return nothing
    end
    if label1.accumulated_capacity > label2.accumulated_capacity
        return nothing
    end
    # if label1.accumulated_duration > label2.accumulated_duration
    #     return nothing
    # end
    # if label1.earliest_time > label2.earliest_time
    #     return nothing
    # end
    # BitSet subset operation is O(n) but much faster than Vector
    if !issubset(label1.M, label2.M)
        return nothing
    end

    # Check if they're identical (no dominance if equal)
    # if label1.reduced_cost == label2.reduced_cost &&
    #    label1.accumulated_capacity == label2.accumulated_capacity &&
    #    label1.accumulated_duration == label2.accumulated_duration &&
    #    label1.earliest_time == label2.earliest_time &&
    #    label1.M == label2.M
    #     return nothing
    # end
    
    return label2
end

"""
    dominanceCheckSingle_optimized(l1, l2)

Check dominance between two labels.
Returns 1 if l2 dominates l1
        2 if l1 dominates l2
        nothing if no dominance.
"""
@inline function dominanceCheckSingle_optimized(l1::LabelOptimized, l2::LabelOptimized)
    # Use the original two-call approach but with @inline for better performance
    # Check if l1 dominates l2
    if l1.reduced_cost <= l2.reduced_cost && 
       l1.accumulated_capacity <= l2.accumulated_capacity &&
       issubset(l1.M, l2.M)
        # Ensure not identical (at least one strict inequality)
        if l1.reduced_cost < l2.reduced_cost || 
           l1.accumulated_capacity < l2.accumulated_capacity ||
           l1.M != l2.M
            return 2  # l1 dominates l2
        end
    end
    
    # Check if l2 dominates l1
    if l2.reduced_cost <= l1.reduced_cost && 
       l2.accumulated_capacity <= l1.accumulated_capacity &&
       issubset(l2.M, l1.M)
        # Ensure not identical (at least one strict inequality)
        if l2.reduced_cost < l1.reduced_cost || 
           l2.accumulated_capacity < l1.accumulated_capacity ||
           l2.M != l1.M
            return 1  # l2 dominates l1
        end
    end
    
    return nothing  # no dominance
end

"""
    get_neighbours_optimized(rho::Int)

Generate neighborhood sets using BitSet for faster operations.
Returns Dict{Int, BitSet} instead of Dict{Int, Vector{Int}}.
"""
function get_neighbours_optimized(rho::Int)
    neighbours = Dict{Int, BitSet}()

    # For satellite nodes, cycle is not allowed - include all nodes
    for node in satellites 
        neighbours[node] = BitSet(A2)
    end

    # For customer nodes, define neighborhoods by rho closest nodes
    for node in customers
        # Get all possible destinations except the node itself
        candidates = [(other, arc_cost[node, other]) for other in customers if other != node]
        
        # Sort by cost
        sorted_candidates = sort(candidates, by = x -> x[2])
        
        # Take top rho neighbors and include the node itself
        ng_neighbours = vcat([node], [x[1] for x in Iterators.take(sorted_candidates, rho)])
        
        neighbours[node] = BitSet(ng_neighbours)
    end

    return neighbours
end

"""
    ng_labelling_optimized(π1, π2, π3, π4, selected_parkings, branchingInfo; rho=5)

Optimized ng-route labeling algorithm with major performance improvements:
- PriorityQueue for efficient label selection
- BitSet for fast set operations
- Eliminated unnecessary deep copies
- Fixed dominance checking logic
- Pre-filtering of feasible nodes
"""
function ng_labelling_optimized(π1, π2, π3, π4, π5, π6, selected_parkings, branchingInfo)
    # println("Starting OPTIMIZED ng-path labelling algorithm")

    # println("π1=  ", round.(π1, digits=2))
    # println("π2=  ", round.(π2, digits=2))
    # println("π3=  ", round.(π3, digits=2))
    # println("π4=  ", round.(π4, digits=2))
    

    # Initialization - Pre-compute sets for O(1) membership checks
    satellites_set = BitSet(satellites)
    customers_set = BitSet(customers)
    active_nodes_set = BitSet(vcat(collect(selected_parkings), customers))
    
    processedLabels = Dict{Int, Vector{LabelOptimized}}()
    depotLabels = Vector{LabelOptimized}()
    sizehint!(depotLabels, 1000)  # Pre-allocate space
    result = Vector{LabelOptimized}()
    sizehint!(result, 1000)  # Pre-allocate space
    new_columns_found = false
    
    # Initialize containers for each node
    for node in active_nodes_set
        processedLabels[node] = Vector{LabelOptimized}()
    end
    
    # Use PriorityQueue for O(log n) label selection
    label_queue = PriorityQueue{LabelOptimized, Float64}()
    
    # Initialize with starting labels at each parking
    for parking in selected_parkings
        rc = π1[parking] + π3[parking] - π5 + π6
        l = LabelOptimized(parking, rc, 0, 0, 0, BitSet([parking]), [parking])
        enqueue!(label_queue, l, rc)
    end
    
    num_iter_labelling = 0
    num_new_columns = 0
    
    # Main labeling loop
    while !isempty(label_queue) && num_new_columns < 50
       num_iter_labelling += 1
        
        # Select label with minimum reduced cost - O(log n) with PriorityQueue
        min_label = dequeue!(label_queue)
        current_node = min_label.current_node
   
        # for ltest in 
            # if min_label.visitedSequence == [5, 11, 25]
            #     println("target label found")
            # end
        # end
        if selected_parkings == [5,4]
            println(min_label.visitedSequence)
        end
      
        # Mark as processed
        push!(processedLabels[current_node], min_label)
        
        # println("selected label:  ",min_label.visitedSequence)
        # Extend label to feasible nodes (iterate directly without creating intermediate array)
        for node in active_nodes_set
            # Check feasibility inline - O(1) with BitSet
            if node == current_node || node in min_label.M
                continue
            end

            #region : Branching rule
            # * branching rule : forbidden combination of parking-customer
            if any(
                forbidden -> (min_label.visitedSequence[1], node) == forbidden,
                branchingInfo.forbidden_combinations)
                continue
            end

            # * branching rule : forbidden combination of customer-customer
            branching_rule_legal = true
            for combination in branchingInfo.forbidden_served_together 
                to_check_sequence = vcat(min_label.visitedSequence, node)
                if Int(combination[1] in to_check_sequence) + Int(combination[2] in to_check_sequence) == 2
                    branching_rule_legal = false
                    break
                end
            end
            if !branching_rule_legal
                continue
            end

            # * check branching rule : obligatory combination parking-customer
            if any(comb ->
                min_label.visitedSequence[1] != comb[1] &&
                node == comb[2],
                branchingInfo.must_include_combinations)
                continue
            end
            #endregion
            
            new_label = extendLabel_optimized(π2, π3, π4, min_label, node, neighbours)
            if !isnothing(new_label)
                # if min_label.visitedSequence == [5, 11, 25]
                #     println(new_label.visitedSequence)
                # end
                
                # * Handle depot (satellite) labels
                if node in satellites_set && new_label.reduced_cost < -1e-8 && length(new_label.visitedSequence) > 2
                    # * branching rule : obligatory combination of customer-customer
                    branching_rule_legal = true
                    for combination in branchingInfo.must_served_together
                        if Int(combination[1] in new_label.visitedSequence) + Int(combination[2] in new_label.visitedSequence) == 1
                            branching_rule_legal = false
                            break
                        end
                    end
                    if !branching_rule_legal
                        continue
                    end

                    # if new_label.visitedSequence == [5, 11, 25, 12]
                    #     println("target label branching rule legality: $branching_rule_legal")
                    # end

                    # * check existence in routes pool
                    found_in_pool = false
                    for route in routes_2e_by_start[new_label.visitedSequence[1]] 
                        if route.sequence == new_label.visitedSequence
                            found_in_pool = true
                        end
                    end

                    if !found_in_pool
                        # * Check dominance against existing depot labels
                        is_dominated = false
                        labels_to_remove = Int[]
                        
                        for (idx, depot_label) in enumerate(depotLabels)
                            dom_result = dominanceCheckSingle_optimized(new_label, depot_label)
                            if dom_result == 1
                                # New label is dominated
                                is_dominated = true
                                break
                            elseif dom_result == 2
                                # New label dominates existing label
                                push!(labels_to_remove, idx)
                            end
                        end
                        
                        if !is_dominated
                            # Remove dominated labels (in reverse order to maintain indices)
                            for idx in reverse(labels_to_remove)
                                deleteat!(depotLabels, idx)
                            end

                            push!(depotLabels, new_label)
                            # if 4 in selected_parkings && 6 in selected_parkings && length(selected_parkings) == 2
                            #     println(new_label.visitedSequence, "  ", 
                            #             round(new_label.reduced_cost, digits=2), "   +", 
                            #             round(generate2eRoute(new_label.visitedSequence).cost, digits=2), "   -", 
                            #             round(sum(π2[new_label.visitedSequence]), digits=2))
                            # end
                            new_route = generate2eRoute(new_label.visitedSequence)
                            new_columns_found = true
                            push!(routes_2e, new_route)
                            push!(routes_2e_by_start[new_label.visitedSequence[1]], new_route)
                            add_2eroute!(length(routes_2e))
                            num_new_columns += 1
                        end
                    end
                    
                
                # * Handle customer labels - O(1) membership check with BitSet
                elseif node in customers_set
                    is_dominated = false
                    
                    # Check against processed labels
                    for proc_label in processedLabels[node]
                        if dominanceCheckSingle_optimized(new_label, proc_label) == 1
                            is_dominated = true
                            break
                        end
                    end
                    
                    # If not dominated, add to queue
                    if !is_dominated
                        enqueue!(label_queue, new_label, new_label.reduced_cost)
                    end
                end
            end
        end
    end
    # * PRINT
    # println("Completed: $num_iter_labelling iterations, $num_new_columns routes with negative reduced cost")
    
    return new_columns_found
end

"""
    ng_labelling_optimized_v2(π1, π2, π3, π4, selected_parkings, branchingInfo; rho=5)

Alternative optimized version that keeps LabelOptimized format in output.
Use this if you want to avoid conversion overhead.
"""
function ng_labelling_optimized_v2(π1, π2, π3, π4, selected_parkings, branchingInfo; rho=5)
    println("Starting OPTIMIZED ng-path labelling algorithm v2 (rho=$rho)")
    
    neighbours = get_neighbours_optimized(rho)
    processedLabels = Dict{Int, Vector{LabelOptimized}}()
    depotLabels = Vector{LabelOptimized}()
    active_nodes = vcat(collect(selected_parkings), customers)
    result = Vector{LabelOptimized}()
    
    for node in active_nodes
        processedLabels[node] = Vector{LabelOptimized}()
    end
    
    label_queue = PriorityQueue{LabelOptimized, Float64}()
    
    for parking in selected_parkings
        rc = π1[parking] - π3[parking] 
        l = LabelOptimized(parking, rc, 0, 0, 0, BitSet([parking]), [parking])
        enqueue!(label_queue, l, rc)
    end
    
    num_iter_labelling = 0
    max_queue_size = 0
    
    while !isempty(label_queue)
        num_iter_labelling += 1
        max_queue_size = max(max_queue_size, length(label_queue))
        
        min_label = dequeue!(label_queue)
        current_node = min_label.current_node
        push!(processedLabels[current_node], min_label)
        
        # More aggressive pruning: use neighbours directly
        feasible_nodes = Int[]
        for n in active_nodes
            if n ∉ min_label.M && n != current_node
                push!(feasible_nodes, n)
            end
        end
        
        for node in feasible_nodes
            new_label = extendLabel_optimized(π2, π3, π4, min_label, node, neighbours)
            
            if !isnothing(new_label)
                if node in satellites && new_label.reduced_cost < -1e-8 && length(new_label.visitedSequence) > 2
                    is_dominated = false
                    labels_to_remove = Int[]
                    
                    for (idx, depot_label) in enumerate(depotLabels)
                        dom_result = dominanceCheckSingle_optimized(new_label, depot_label)
                        if dom_result == 1
                            is_dominated = true
                            break
                        elseif dom_result == 2
                            push!(labels_to_remove, idx)
                        end
                    end
                    
                    if !is_dominated
                        for idx in reverse(labels_to_remove)
                            deleteat!(depotLabels, idx)
                        end
                        push!(depotLabels, new_label)
                        push!(result, new_label)
                    end
                    
                elseif node in customers
                    is_dominated = false
                    
                    for proc_label in processedLabels[node]
                        if dominanceCheckSingle_optimized(new_label, proc_label) == 1
                            is_dominated = true
                            break
                        end
                    end
                    
                    if !is_dominated
                        enqueue!(label_queue, new_label, new_label.reduced_cost)
                    end
                end
            end
        end
        
        if num_iter_labelling % 1000 == 0
            println("  Iteration $num_iter_labelling, queue: $(length(label_queue)), results: $(length(result))")
        end
    end
    
    println("Completed: $num_iter_labelling iterations, max queue size: $max_queue_size")
    println("Generated $(length(result)) routes with negative reduced cost")
    
    return result  # Returns LabelOptimized directly
end
