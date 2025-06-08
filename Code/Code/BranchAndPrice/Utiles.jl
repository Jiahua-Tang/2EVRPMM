using Combinatorics, CPLEX, JuMP


function generate1eRoute(route)
    cost = 0
    load = 0
    for i in 1:(length(route) - 1)
        if i!= 1
            load += demands[route[i]]
        end
        from_node = route[i]
        to_node = route[i + 1]
        cost+= arc_cost[from_node, to_node]
    end
    b1 = getB1(route)
    a =  Vector{}()
    b2in = Vector{}()
    b2out = Vector{}()
    return Route(cost, route, length(route), load, b1, a, b2in, b2out) 
end
function generate2eRoute(route::Vector{Int})
    cost = 0
    load = 0
    for i in 1:(length(route) - 1)
        if i!= 1
            load += demands[route[i]]
        end
        from_node = route[i]
        to_node = route[i + 1]
        cost+= arc_cost[from_node, to_node]
    end
    b1 = Vector{}()
    a = getA(route)
    b2in = getB2In(route)
    b2out = getB2Out(route)
    return Route(cost, route, length(route), load, b1, a, b2in, b2out) 
end


# Parameter if customer i is served by a route r
function getA(route)
    a = zeros(Int, length(customers))
    for (idx, cust) in enumerate(customers)
        a[idx] = cust in route ? 1 : 0          
    end
    return a
end

# Parameter if parking i is available by route r
function getB1(route)
    b = zeros(Int, length(parking_availability))
    for (idx, parking) in enumerate(parking_availability)
        if idx != 1
            if idx in route
                # route r passes parking, define the status of s modified by r
                if parking == 0
                    # if parking s is empty at the beginning, then it will change to Available
                    b[idx] = 1
                    # println("parking $s is empty and used by route $r ",route.sequence, " b[$s] = 1")
                else
                    # if parking s is occupied at the beginning, the status depends on next point visited
                    if parking_availability[route[findfirst(==(idx), route)+1]] == 0 && route[findfirst(==(idx), route)+1] != 1
                        # if next visited point is empty
                        b[idx] = 0
                        # println("parking $s is occupied and used by route $r",route.sequence, " b[$s] = 0")
                    else
                        # if next visited point is occupied
                        b[idx] = 1
                        # println("parking $s is occupied and used by route $r ",route.sequence, " b[$s] = 1")
                    end
                end
            else
                # route r doesn't pass parking, it will not change the status
                b[idx] = 0
                # println("b[$s] = ",b[s])
            end
        end
    end
    return b
end

# Parameter if route r starts from parking p
function getB2Out(route)
    b = zeros(Int, length(satellites)+1)
    for (s_idx, i) in enumerate(satellites)  # Loop over points (row index)
        b[s_idx + 1] = route[1] == i ? 1 : 0
    end
    return b
end

# Parameter if route r ends at parking p
function getB2In(route)
    b = zeros(Int, length(satellites)+1)
    for (s_idx, i) in enumerate(satellites)  # Loop over points (row index)
        b[s_idx + 1] = route[end] == i ? 1 : 0
    end
    return b
end



function getServedParking1eRoute(route::Route)
    served_parkings = Set{Int}()
    for (idx, v) in enumerate(route.b1) 
        if v == 1
            push!(served_parkings, idx)
        end
    end
    return served_parkings
end

function generateNonDominate1eRoutes(least_required_mm::Int)
    function generateAllFeasible1eRoute(least_required_mm)
        # least_required_mm = Int(least_required_mm)
        feasible_1e_routes = Vector{Route}()
        # Generate all permutations of customer subsets (excluding empty set)
        for num_parkings in least_required_mm:nb_microhub*2
            # println("$num_parkings parkings with ", length(combinations(2:length(parking_availability),num_parkings))," combinations" )
            # @info "Generate route visit $num_parkings parkings"
            for parking_subset in combinations(2:length(parking_availability),num_parkings)
                # @info "selected parking subset $parking_subset"
                for parking_permutation in permutations(parking_subset)
                    if parking_availability[parking_permutation[1]] == 1
                        valide = true
                        for i in 2:(length(parking_permutation) - 1)
                            if parking_availability[parking_permutation[i]] == 0 && parking_availability[parking_permutation[i+1]] == 0
                                valide = false
                                break
                            end
                        end
                        if valide == true
                            route = vcat(1, parking_permutation, 1)
                            # println("generated route= ",route)
                            route = generate1eRoute(route)
                            push!(feasible_1e_routes, route)
                        end                 
                    end
                end
            end
        end
        return feasible_1e_routes
    end
    # @info "Start generation of 1e route"
    result = Vector{Route}()
    routes = generateAllFeasible1eRoute(least_required_mm)
    # for route in routes 
    #     println(route.sequence, "   ", getServedParking1eRoute(route))
    # end
    least_required_mm = Int(least_required_mm)
    # generate all possible combination of served parkings
    # @info "Start filtering 1e route"
    for num_parkings in least_required_mm:nb_microhub
        # @info "Filtering route serving $num_parkings parkings"
        for served_parkings in combinations(2:length(parking_availability), num_parkings)
            served_parkings = sort(served_parkings)
            best_route = nothing 
            found = false
            for route in routes
                if sum(route.b1) == num_parkings
                    b1_value = sort(collect(getServedParking1eRoute(route)))
                    if b1_value == served_parkings
                        found = true
                        if isnothing(best_route)
                            best_route = route
                        else
                            if route.cost < best_route.cost
                                best_route = route
                            end
                        end
                    end
                end
            end
            # println(served_parkings)
            # println("Served parkings = ", served_parkings, "  with best route = ", best_route.sequence)
            if found
                push!(result, best_route)
            end
        end     
    end    
    # @info "End generation of 1e route"
    return result
end

function calculateLRPLowerBound(routes_1e_complete)
    lb_per_route = Dict{Int, Float64}()

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
        end
    end

    return lb_per_route
end

function displayLRPLowerBound(lb_lrp_per_route, routes_1e_complete)
    count = 1
    while !isempty(lb_lrp_per_route)
        min_value, min_idx = findmin(lb_lrp_per_route)
        println(count, ". ",routes_1e_complete[min_idx].sequence,"  ",getServedParking1eRoute(routes_1e_complete[min_idx]),"   lower bound= $(round(min_value, digits=2))")
        delete!(lb_lrp_per_route, min_idx)
        count += 1
    end
end

function generate2eInitialRoutes()
    function generate2eDummyRoute()
        routes = Vector{Route}()
        
        for startParking in satellites
            for endParking in satellites
                sequence = Vector{Int}()
                push!(sequence, startParking)
                for cust in customers 
                    push!(sequence, cust)
                end
                push!(sequence, endParking)
                route = generate2eRoute(sequence)
                route.cost = 1e5
                push!(routes, route)   
                # println("dummy route = ", route.sequence, " cost = ", route.cost)         
            end
        end
        return routes
    end

    result = Vector{Route}()
    global dummyRoutes
    dummyRoutes = generate2eDummyRoute()
    for (_, route) in enumerate(dummyRoutes)
        push!(result, route)
    end
    for cust in customers
        for parkingStart in satellites
            for parkingEnd in satellites
                if arc_cost[parkingStart, cust] + arc_cost[cust, parkingEnd] <= maximum_duration_2e_vehicle
                    push!(result, generate2eRoute([parkingStart, cust, parkingEnd]))
                end              
            end
        end
    end
    return result
end

function displayBranchingNode(branchingNode::BranchingNode)
    println("- Branching node in depth: $(branchingNode.branchingInfo.depth)")
    displayBranchingRule(branchingNode.branchingInfo)
    # println("  Branching node with CG result: ")
    # for (_, y) in enumerate([r for r in 1:length(branchingNode.y_value) if 0 < branchingNode.y_value[r]]) 
    #     println("   $(round(branchingNode.y_value[y],digits=2))")
    # end
    println("  Branching node with fractional score: $(round(branchingNode.fractionalScore, digits=2))")
    println("  Branching node with cg lower bound: $(round(branchingNode.cgLowerBound, digits=2))")    
    # println("  Branching node with LB change: $(round(branchingNode.gradientLB, digits=2))")
    println("  Branching node with LB reduction: $(round(branchingNode.gradientLB/branchingNode.branchingInfo.depth, digits=2))")
    # println("  Branching node with FS change: $(round(branchingNode.gradientFS, digits=2))")
    println("  Branching node with FS reduction: $(round(branchingNode.gradientFS/branchingNode.branchingInfo.depth, digits=2))")
    # println("  Branching node contains $(length(branchingNode.routes_pool)) routes")
    println("  Total reduction: $(round((branchingNode.gradientLB+branchingNode.gradientFS)/(1+log(branchingNode.branchingInfo.depth)),digits=2))")
    println("")
end


function checkExistanceDummyRoute(y, routes_2e)

    for (idx, y_value) in enumerate([r for r in 1:length(y) if 0 < y[r]]) 
        route = routes_2e[y_value]
        for dummyRoute in dummyRoutes
            if dummyRoute.sequence == route.sequence
                return true
            end
        end
    end
    return false
end

function checkOptimal(routes_1, routes_2, x, y, optimal_solution, optimal_solution_value)
    optimal_1e_route = routes_1[[r for r in 1:length(x) if x[r] == 1]]
    optimal_1e_route = optimal_1e_route[1]
    current_solution_value = optimal_1e_route.cost
    current_solution = Vector{Route}()
    push!(current_solution, optimal_1e_route)
    y = [r for r in 1:length(y) if y[r] == 1]
    for y_value in y
        current_solution_value += routes_2[y_value].cost
        push!(current_solution, routes_2[y_value])
    end
    println("Ongoing solution value = ", current_solution_value)
    println("Ongoing solution 1e route = ", optimal_1e_route.sequence)

    if current_solution_value <= optimal_solution_value
        optimal_solution_value = current_solution_value
        optimal_solution = current_solution
    end   

    return optimal_solution, optimal_solution_value
end