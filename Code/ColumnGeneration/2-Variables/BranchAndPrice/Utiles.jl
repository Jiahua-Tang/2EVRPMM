mutable struct Route
    cost::Float64
    sequence::Vector{Int}
    length::Int
    load::Int
    b1::Vector{Int}
    a::Vector{Int}
    b2in::Vector{Int}
    b2out::Vector{Int}
end

function calculate_arc_cost(points, nb_parking, PI)
    num_points = length(points)
    arc_cost = Array{Float64}(undef, num_points, num_points)
    for i in 1:num_points
        for j in 1:num_points
            if i == j
                arc_cost[i, j] = 100000.0  # Large value for self-loops
            else
                arc_cost[i, j] = sqrt(sum((points[i][k] - points[j][k])^2 for k in 1:2))
            end
        end
    end

    # for i in 1:num_points
    #     for j in 1:num_points
    #         print("  d[$i $j]=", round(arc_cost[i,j], digits=2))
    #     end
    #     println("")
    # end
    return arc_cost
end

function calculate_load(path, demands)
    load = 0
    for node in path[2:end-1]  # Exclude the start and end warehouses
        load += demands[node]
    end
    return load
end

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

function transformRoute(x)
    new_route = Vector{Int}()
    current_node = 0
    if length(x) > (1+length(satellites))^2 
        ## Transform a 2e route
        # Find the starting satellite
        for s in satellites
            for j in A2
                if round(value(x[s, j])) == 1
                    current_node = s
                    push!(new_route, s)
                    break
                end
            end
            if current_node != 0
                break
            end
        end
        while true
            for j in A2
                if round(value(x[current_node,j])) == 1
                    current_node = j
                    push!(new_route,current_node)
                    break
                end
            end
            if current_node in satellites
                break
            end
        end
    else
        ## Transform a 1e route
        current_node = 1 
        push!(new_route, current_node)
        while true
            for j in A1
                if round(value(x[current_node,j])) == 1
                    current_node = j
                    push!(new_route,current_node)
                    break
                end
            end
            if current_node == 1
                break
            end
        end
    end
    
    # generateRoute(new_route)
    # println("transformRoute TEST",new_route)
    return new_route
end

function generateParkingCoor(coor_customers, nb_microhub, nb_parking)
    Random.seed!(42)
    parking_availability = vcat(ones(Int, nb_microhub),zeros(Int, nb_parking-nb_microhub))
    shuffle!(parking_availability)
    parking_availability = vcat(0,parking_availability)
    x_coor_parkings = []
    y_coor_parkings = []
    x_coor_customers = [point[1] for point in coor_customers]
    y_coor_customers = [point[2] for point in coor_customers]
    min_x,min_y = minimum(x_coor_customers),minimum(y_coor_customers)
    max_x,max_y = maximum(x_coor_customers),maximum(y_coor_customers)
    for i in 1:nb_parking
        push!(x_coor_parkings, rand(min_x: max_x))
        push!(y_coor_parkings, rand(min_y: max_y))
    end
    coor_parkings = [[x,y] for (x,y) in zip(x_coor_parkings , y_coor_parkings)]
    return parking_availability, coor_parkings
end

function loadData()
    
end

function initializeData(nb_parking, nb_microhub, nb_customers)

    Random.seed!(42)
    coor_cust = [[rand(1:50), rand(1:50)] for _ in 1:nb_customers]
    demands = vcat(zeros(Int, nb_parking + 1) , rand(1:15, nb_customers))

    parking_availability, coor_parkings = generateParkingCoor(coor_cust, nb_microhub, nb_parking)
    coor = vcat([[0,0]], coor_parkings, coor_cust)

    arc_cost = calculate_arc_cost(coor, nb_parking, parking_availability)
    
    return coor, nb_parking, nb_microhub, parking_availability, demands, arc_cost

end

function generateAllRoutes()
    feasible_1e_routes = generateBest1eRoutes(minimum_parkings_required)
    # feasible_1e_routes = generateBestFeasible1eRoutes(minimum_parkings_required)
    feasible_2e_routes = generateAllFeasible2eRoute()
    return feasible_1e_routes, feasible_2e_routes
end

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

function generateBest1eRoutes(least_required_mm::Int)
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

function generateBestFeasible1eRoutes(least_required_mm::Int)
    # @info "Start generation of best feasible 1e routes"
    least_required_mm = Int(least_required_mm)
    result = Vector{Route}()

    # Map to hold best route for each served parking set (sorted vector)
    best_route_map = Dict{Vector{Int}, Route}()

    parking_indices = 2:length(parking_availability)

    for num_parkings in least_required_mm:nb_microhub*2
        # @info "Processing subsets with $num_parkings parkings"
        for parking_subset in combinations(parking_indices, num_parkings)
            println(parking_subset)
            # Only allow permutations that start with an occupied parking
            occupied_firsts = filter(p -> parking_availability[p] == 1, parking_subset)
            for first in occupied_firsts
                remaining = setdiff(parking_subset, [first])
                for rest_perm in permutations(remaining)
                    parking_perm = (first, rest_perm...)
                    # Validate the permutation: no two empty parkings in a row
                    valid = true
                    for i in 2:(length(parking_perm)-1)
                        if parking_availability[parking_perm[i]] == 0 &&
                           parking_availability[parking_perm[i+1]] == 0
                            valid = false
                            break
                        end
                    end
                    if valid
                        # Ensure last point before returning to depot is occupied if needed
                        if parking_availability[parking_perm[end]] == 0
                            continue  # last must be occupied
                        end
                        route_seq = vcat(1, collect(parking_perm), 1)
                        route = generate1eRoute(route_seq)
                        served = sort(collect(getServedParking1eRoute(route)))

                        if !haskey(best_route_map, served) || route.cost < best_route_map[served].cost
                            best_route_map[served] = route
                        end
                    end
                end
            end

        end
    end

    # Collect results
    for route in values(best_route_map)
        push!(result, route)
    end

    # @info "Finished generating best feasible 1e routes"
    return result
end
function generateAllFeasible2eRoute()
    # @info "Start generation of feasible 2e routes"
    feasible_2e_routes = Vector{Route}()

    customer_indices = (length(parking_availability)+1):length(coor)

    for num_customers in 1:length(customer_indices)
        # println("Generate route of $num_customers customers")
        for customer_subset in combinations(customer_indices, num_customers)
            total_load = sum(demands[cust] for cust in customer_subset)

            if total_load <= capacity_2e_vehicle
                customer_perms = num_customers == 1 ? [customer_subset] : permutations(customer_subset)

                for perm in customer_perms
                    for start_parking in 2:length(parking_availability)
                        for end_parking in 2:length(parking_availability)
                            full_route = vcat(start_parking, perm, end_parking)
                            route = generate2eRoute(full_route)

                            if route.cost <= maximum_duration_2e_vehicle
                                push!(feasible_2e_routes, route)
                            end
                        end
                    end
                end
            end
            # println("$(length(feasible_2e_routes)) 2e routes generated")
        end
    end

    # @info "End generation of feasible 2e routes"
    return feasible_2e_routes
end

function generate1eInitialSolition()
    solution = Vector{Route}()
    for (idx, p) in enumerate(parking_availability)
        if idx != 1
            if p == 1
                route = vcat(1, idx, 1)
                # println("Occupied parking: ", route)
                route = generate1eRoute(route)
                push!(solution, route)
            else
                min = maximum(arc_cost)
                min_value = 0
                for (i, v) in enumerate(parking_availability)
                    if v == 1 
                        # println("$i, $idx, ",arc_cost[i,idx])
                        if arc_cost[i, idx] < min
                            min_value = i
                            min = arc_cost[i, idx]
                        end
                    end
                    # println("$min, $min_value")
                end
                route = vcat(1, min_value, idx , 1)
                # println(route)
                route = generate1eRoute(route)
                push!(solution, route)            
            end
        end

    end
    return solution
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

function getSortedServedParking1eRoute(route::Route)
    return sort(collect(getServedParking1eRoute(route)))
end

function generate2eInitialRoutes()
    result = Vector{Route}()
    for cust in customers
        for parking in satellites
            if arc_cost[parking, cust] <= maximum_duration_2e_vehicle/2
                push!(result, generate2eRoute([parking, cust, parking]))
            end
        end
    end
    return result
end

# Function to generate corresponding optimal 1e route when all 2e variables are integer
function findOptimal1eRoute(routes_1, routes_2, y_value)
    # println(y_value)
    # println(length(routes_2))
    required_parkings = Set{Int}()

    for v in y_value
        push!(required_parkings, routes_2[v].sequence[1])
        push!(required_parkings, routes_2[v].sequence[end])
    end

    # println(required_parkings)

    optimal_1e_route = nothing
    for route in routes_1
        # println(getSortedServedParking1eRoute(route), "   ",sort(collect(required_parkings)))
        if getSortedServedParking1eRoute(route) == sort(collect(required_parkings))
            if isnothing(optimal_1e_route)
                optimal_1e_route = route
            else
                if optimal_1e_route.cost > route.cost
                    optimal_1e_route = route
                end
            end
        end
    end
    # println("Result of optimal 1e route function: ", optimal_1e_route)
    return optimal_1e_route
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