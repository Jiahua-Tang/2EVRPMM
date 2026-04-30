include("../BranchAndPrice/columnGeneration.jl")

mutable struct LabelTSP
    current_node::Int
    distance::Float64
    # microhubStatus::Int
    parkingStatus::Vector{Int}
    visitedNodes::Vector{Int}
    visitedSequence::Vector{Int} 
end

function solve_1e_tsp_labelling(selected_parkings)
    if length(selected_parkings) == 1
        selected_parking = selected_parkings[1]
        if parking_availability[selected_parking] == 1
            route_1e = generate1eRoute([1, selected_parking, 1])
        else
            min_distance = Inf
            solution = []
            for parking in satellites 
                if parking_availability[parking] == 1
                    distance = arc_cost[1, parking] + arc_cost[parking, selected_parking] + arc_cost[selected_parking, 1]
                    if distance < min_distance
                        solution = [1, parking, selected_parking, 1]
                    end
                end
            end
            route_1e = generate1eRoute(solution)
        end
    else
        satellites_set = BitSet(satellites)
        active_nodes_set = BitSet(vcat(1, collect(satellites)))

        processedLabels = Dict{Int, Vector{LabelTSP}}()
        result_labels = PriorityQueue{LabelTSP, Float64}()

        for node in active_nodes_set 
            processedLabels[node] = Vector{LabelTSP}()
            sizehint!(processedLabels[node], 100) 
        end

        # Use PriorityQueue for O(log n) label selection
        label_queue = PriorityQueue{LabelTSP, Float64}()

        num_iter_labelling = 0
        visit_nodes = zeros(Int, length(A1))
        parking_avail = zeros(Int, length(A1))
        new_label = LabelTSP(1, 0, parking_avail, visit_nodes, [1])
        enqueue!(label_queue, new_label, 0)

        while !isempty(label_queue) #&& num_iter_labelling < 16
            num_iter_labelling += 1

            # * choose a minimal travel distance label and set as processed
            min_label = dequeue!(label_queue)
            
            current_node = min_label.current_node
            push!(processedLabels[current_node], min_label)

            for node in active_nodes_set
                if node == current_node
                    continue
                end
                # * propagate new label to node
                new_label = extend_tsp_label(min_label, node, selected_parkings)

                if !isnothing(new_label)
                    # * check dominance relation between new label and existing labels

                    # * in stack for destination node and parking nodes
                    if node == 1
                        valide = true
                        for n in satellites
                            if n in selected_parkings && new_label.parkingStatus[n] == 0
                                valide = false
                                break
                            end
                        end
                        if valide
                            enqueue!(result_labels, new_label, new_label.distance)
                        end
                    else
                        enqueue!(label_queue, new_label, new_label.distance)
                    end
                end
            end
        end
        route_1e = generate1eRoute(dequeue!(result_labels).visitedSequence)
    end
    # println(route_1e.sequence)
    return route_1e
end

function dominance_check_tsp(label1, label2)
    visit_status = true
    for node in A1 
        if label1.visitedNodes[node] > label2.visitedNodes[node]
            visit_status = false
            break
        end
    end

    # println(label1.visitedSequence, "  ", label2.visitedSequence, "   ", visit_status)
    if label1.distance <= label2.distance && visit_status
        println("label $(label1.visitedSequence), $(round(label1.distance, digits=2)) dominates label $(label2.visitedSequence), $(round(label2.distance, digits=2)) ")
        return 2 # l1 dominates l2
    end

    visit_status = true
    for node in A1 
        if label2.visitedNodes[node] > label1.visitedNodes[node]
            visit_status = false
            break
        end
    end
    if label2.distance <= label1.distance && visit_status
        println("label $(label2.visitedSequence), $(round(label2.distance, digits=2))  dominates label $(label1.visitedSequence), $(round(label1.distance, digits=2)) ")
        return 1 # l2 dominates l1
    end
    
    return nothing # no dominance relation exists
end

function extend_tsp_label(label, next_node, selected_parkings)

    if next_node in label.visitedSequence && next_node != 1
        return nothing
    end

    # println("extend label to $next_node")
    current_node = next_node

    distance = label.distance + arc_cost[label.current_node, next_node]
    parking_avail = deepcopy(label.parkingStatus)
    if next_node != 1
        if parking_availability[label.current_node] == 0
            if parking_availability[next_node] == 0
                # move from empty parking to another empty parking : forbidden
                return nothing
            end
        elseif parking_availability[label.current_node] == 1
            if parking_availability[next_node] == 0
                if !(next_node in selected_parkings)
                    return nothing
                end
                # move from an occupied parking to a selected empty parking : replenish next
                parking_avail[label.current_node] = 0
                parking_avail[next_node] = 1
            elseif parking_availability[next_node] == 1
                if !(label.current_node in selected_parkings)
                    return nothing
                end
                # move from an occupied parking to another selected occupied parking : replenish previous
                parking_avail[label.current_node] = 1
            end 
        end
    elseif next_node == 1
        if parking_availability[label.current_node] == 1
            if !(label.current_node in selected_parkings)
                return nothing
            end
            # move from an occupied parking to depot: replenish
            parking_avail[label.current_node] = 1
        end
    end

    visitedNodes = deepcopy(label.visitedNodes)
    visitedNodes[next_node] = 1
    visitedSequence = vcat(label.visitedSequence, next_node)

    new_label = LabelTSP(current_node, 
                         distance,
                         parking_avail,
                         visitedNodes,
                         visitedSequence)
    # println("new label generated")
    return new_label
end

function get_sorted_2e_subproblems()
    
    function build_MDVRP_model()
        model = Model(CPLEX.Optimizer)
        set_silent(model)

        @variable(model, 1 >= x[A2, A2] >= 0)
        @variable(model, capacity_2e_vehicle >= f[A2, A2] >= 0)

        @objective(model, Min, sum(arc_cost[i,j] * x[i,j] for i in A2, j in A2))

        @constraint(model, [i in A2], x[i,i] == 0)

        @constraint(model, [i in A2], sum(x[i,j] for j in A2) == sum(x[j,i] for j in A2))
        @constraint(model, cov[i in customers], sum(x[i,j] for j in A2) == 1)
        @constraint(model, [i in customers], sum(f[j,i] for j in A2) - sum(f[i,j] for j in A2) == demands[i])
        @constraint(model, [i in A2, j in A2], f[i,j] <= capacity_2e_vehicle * x[i,j])
        @constraint(model, [i in satellites, j in satellites], x[i,j] == 0)
        @constraint(model, [i in A2, j in A2], f[i,j]>=x[i,j])
        # c_vehicle_limit = @constraint(model, [i in satellites], sum(x[i,j] for j in customers) <= 0)
        c_open_lb       = @constraint(model, [i in satellites], sum(x[i,j] for j in A2) >= 0)
        c_open_ub       = @constraint(model, [i in satellites], sum(x[i,j] for j in A2) <= 0)

        return (
            model = model,
            x = x,
            f = f,
            # c_vehicle_limit = c_vehicle_limit,
            c_open_lb = c_open_lb,
            c_open_ub = c_open_ub,
        )
    end

    function solve_MDVRP_LP!(data, selected_parkings)
        selected = Set(selected_parkings)

        for i in satellites
            if i in selected
                # for a selected parking, set the vehicle limit to capacity
                # set_normalized_rhs(data.c_vehicle_limit[i], nb_vehicle_per_satellite)

                # for a selected parking, set the out flow to at least 1
                set_normalized_rhs(data.c_open_lb[i], 1)
                # for a selected parking, set the out flow to at most the number of vehicles
                set_normalized_rhs(data.c_open_ub[i], nb_vehicle_per_satellite)
            else
                # set_normalized_rhs(data.c_vehicle_limit[i], 0)
                set_normalized_rhs(data.c_open_lb[i], 0)
                set_normalized_rhs(data.c_open_ub[i], 0)
            end
        end

        optimize!(data.model)
        #region:LP MDVRP output
        # println(selected_parkings)
        # for i in A2
        #     for j in A2
        #         if value(data.x[i,j]) > 1e-5
        #             println("x[$i,$j] = ", round(value(data.x[i,j]), digits=3), "   flow = ", round(value(data.f[i,j]), digits=3))
        #         else
        #             if value(data.f[i,j])>1e-5
        #                 println("f[$i,$j] = ", round(value(data.f[i,j]), digits=3))
        #             end
        #         end
        #     end 
        # end
        #endregion
        return objective_value(data.model)
    end

    lrp_subproblems = PriorityQueue()
    global routes_1e_complete = Vector{Route}()
    execution_time = @elapsed data = build_MDVRP_model()
    println("execution time building MDVRP model: ", round(execution_time, digits=2), " seconds")
    for num_parking in minimum_parkings_required:nb_microhub
        for parking_subset in combinations(satellites, num_parking)
            route_1e = solve_1e_tsp_labelling(parking_subset)
            push!(routes_1e_complete, route_1e)
            lower_bound_subproblem = route_1e.cost
            execution_time_lp = @elapsed lower_bound_subproblem += solve_MDVRP_LP!(data, parking_subset)
            println("parking subset: ", parking_subset, " 1e route cost: ", round(route_1e.cost, digits=2), " LP MDVRP cost: ", round(lower_bound_subproblem, digits=2), " execution time LP: ", round(execution_time_lp, digits=2), " seconds")
            enqueue!(lrp_subproblems, route_1e, lower_bound_subproblem)
        end
    end
    
    return lrp_subproblems
end