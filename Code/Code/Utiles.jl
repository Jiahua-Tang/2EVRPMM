struct Dataset
    arc_cost::Matrix{Float64}
    coor::Vector{Vector{Float64}}
    nb_parking::Int
    nb_microhub::Int
    nb_customer::Int
    parking_availability::Array{Int}
    demands::Array{Int}

    nb_vehicle_per_satellite::Int
    capacity_1e_vehicle::Int
    capacity_2e_vehicle::Int
    capacity_microhub::Int
    maximum_duration_2e_vehicle::Int

    minimum_parkings_required::Int
    minimum_2e_vehicle_required::Int

    points::Vector{Int}
    customers::Vector{Int}
    satellites::Vector{Int}
    A2::Vector{Int}
    A1::Vector{Int}
end

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

mutable struct BranchingInfo
    must_include_combinations::Set{Tuple{Int, Int}}  # combination of parking-customer that must be in a path
    forbidden_combinations::Set{Tuple{Int, Int}}   # combination of parking-customer that cannot be in a path
    
    must_served_together::Set{Tuple{Int, Int}}   # combination of customers that must be in a path
    forbidden_served_together::Set{Tuple{Int, Int}}   # combination of customers that cannot be in a path
    
    must_include_parkings::Set{Int}   # parking that must be used in solution
    forbidden_parkings::Set{Int}   # parking that cannot be used in solution
    
    upper_bound_number_2e_routes::Set{Int} # upper bound of number of total 2e routes
    lower_bound_number_2e_routes::Set{Int} # lower bound of number of total 2e routes
    
    special_order_set_must_include::Set{Route}  # set of 2e route where all variables are 0
    special_order_set_forbidden_include::Set{Route}  # set of 2e route with sum = 0 

    depth::Int
end

mutable struct BranchingNode
    branchingInfo::BranchingInfo
    cgLowerBound::Float64
    y_value::Vector{Float64}
    isLeaf::Bool
    fractionalScore::Float64
    routes_pool::Vector{Route} ## Routes pool before filter (of father node)
    gradientLB::Float64
    gradientFS::Float64
end

mutable struct Label
    origin_node::Int
    current_node::Int
    reduced_cost::Float64
    accumulated_capacity::Int
    accumulated_duration::Float64
    visitedNodes::Vector{Int}
end

mutable struct LabelLRP
    origin_node::Int
    current_node::Int
    reduced_cost::Float64
    visitedNodes::Vector{Int}
end

function generateData()
    Random.seed!(42)
 
    nb_parking = 6
    nb_microhub = 3
    nb_customer = 15
    
    coor_cust = [[rand(1:50), rand(1:50)] for _ in 1:nb_customer]
    result = generateParkingCoor(coor_cust, nb_microhub, nb_parking)
    parking_availability = result[1]
    coor_parkings = result[2]
    coor = vcat([[0.0,0.0]], coor_parkings, coor_cust)
    arc_cost = calculate_arc_cost(coor)   
    # display(typeof(arc_cost))
    demands = vcat(zeros(Int, nb_parking + 1) , rand(1:15, nb_customer))
    points = 1:length(demands)
    customers = 2 + nb_parking : length(coor)
    satellites = 2:1 + nb_parking
    nb_customer = length(customers)
    A2 = 2:length(coor)
    A1 = 1:1+length(satellites)

    nb_vehicle_per_satellite = 10
    capacity_1e_vehicle = sum(demands)
    capacity_2e_vehicle = 60
    capacity_microhub = 150
    maximum_duration_2e_vehicle = 50

    data = Dataset(
        arc_cost,
        coor,
        nb_parking,
        nb_microhub,
        nb_customer,
        parking_availability,
        demands,

        nb_vehicle_per_satellite,
        capacity_1e_vehicle,
        capacity_2e_vehicle,
        capacity_microhub,
        maximum_duration_2e_vehicle,

        Int(ceil(sum(demands) / capacity_microhub)),
        Int(ceil(sum(demands) / capacity_2e_vehicle)),

        points,
        customers,
        satellites,
        A2,
        A1
    )

    transformDatasetToGlobals(data)

    global parkingGenerationRule = "random"
    global fileName = "randomSeed"
    global execution_time_limit = 120 # seconds

    # return data
end

function transformDatasetToGlobals(data::Dataset)
    global arc_cost = data.arc_cost
    global coor = data.coor
    global nb_parking = data.nb_parking
    global nb_microhub = data.nb_microhub
    global nb_customer = data.nb_customer
    global parking_availability = data.parking_availability
    global demands = data.demands

    global nb_vehicle_per_satellite = data.nb_vehicle_per_satellite
    global capacity_1e_vehicle = data.capacity_1e_vehicle
    global capacity_2e_vehicle = data.capacity_2e_vehicle
    global capacity_microhub = data.capacity_microhub
    global maximum_duration_2e_vehicle = data.maximum_duration_2e_vehicle    

    global minimum_parkings_required = data.minimum_parkings_required
    global minimum_2e_vehicle_required = data.minimum_2e_vehicle_required

    global points = data.points
    global customers = data.customers 
    global satellites = data.satellites
    global A2 = data.A2
    global A1 = data.A1
    
end

function generateParkingCoor(coor_customers, nb_microhub, nb_parking)
    Random.seed!(22)
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

function readData(fileName, ARGS)
    readBasicParameter(fileName, ARGS)
    readDataSetElion()
end

function readBasicParameter(filename, ARGS)

    if length(ARGS) >= 1
        global fileName = ARGS[1]
        global execution_time_limit = ARGS[2]
        global parkingGenerationRule = ARGS[3]
        if parkingGenerationRule == "specified"
            global specifiedParkings = Vector{Int}()
            for i in ARGS[4:end] 
                push!(specifiedParkings, parse(Int, i))
            end
            global nb_parking = length(specifiedParkings) * 2
        end
    else
        global fileName = filename
        global execution_time_limit = 120 # seconds
        global parkingGenerationRule = "random"
    end
end

function readDataSetElion()
    nb_parking = 6
    nb_microhub = 3    
    nb_vehicle_per_satellite = 10
    capacity_microhub = 100000
    maximum_duration_2e_vehicle = 100000
    
    # Read the file lines
    lines = open(root * "Data/" * fileName, "r") do file
        readlines(file)
    end

    # Find the index where the customer data starts
    coord_start_index = findfirst(contains.(lines, "NODE_COORD_SECTION")) + 1
    demand_start_index = findfirst(contains.(lines, "DEMAND_SECTION")) 
    end_index = findfirst(contains.(lines, "DEPOT_SECTION")) -1
    capacity_index = findfirst(contains.(lines, "CAPACITY"))
    line = lines[capacity_index]
    parts = split(line, ":")
    capacity_2e_vehicle = parse(Int, strip(parts[end]))

    x_coor_customers = []
    y_coor_customers = []
    x_coor_depot = []
    y_coor_depot = []
    demands = zeros(Float64, 1+nb_parking)
    for i in coord_start_index : end_index
        line = strip(lines[i])
        if !isempty(line)
            fields = split(line) 
            if i == coord_start_index
            # Depot
                push!(x_coor_depot, parse(Int, fields[2]))
                push!(y_coor_depot, parse(Int, fields[3]))
            elseif i> coord_start_index && i < demand_start_index
            # Customers
                push!(x_coor_customers, parse(Int, fields[2]))
                push!(y_coor_customers, parse(Int, fields[3]))
            elseif i > demand_start_index + 1
            # Demands           
                push!(demands, parse(Int, fields[2]))
            end
        end
    end
    # display(demands)
    coor_depot = [x_coor_depot[1], y_coor_depot[1]]
    coor_customers = [[x,y] for (x,y) in zip(x_coor_customers , y_coor_customers)]
    capacity_1e_vehicle = sum(demands)

    nb_customer = length(demands)
    if parkingGenerationRule == "random"
        parking_availability, coor_parkings = generateParkingCoor(coor_customers, nb_microhub, nb_parking)
    end
    coor = vcat([coor_depot], coor_parkings, coor_customers)
    arc_cost = calculate_arc_cost(coor)
    
    points = 1:length(demands)
    customers = 2 + nb_parking : length(coor)
    satellites = 2:1 + nb_parking
    nb_customer = length(customers)
    A2 = 2:length(coor)
    A1 = 1:1+length(satellites)

    data = Dataset(
        arc_cost,
        coor,
        nb_parking,
        nb_microhub,
        nb_customer,
        parking_availability,
        demands,

        nb_vehicle_per_satellite,
        capacity_1e_vehicle,
        capacity_2e_vehicle,
        capacity_microhub,
        maximum_duration_2e_vehicle,

        Int(ceil(sum(demands) / capacity_microhub)),
        Int(ceil(sum(demands) / capacity_2e_vehicle)),

        points,
        customers,
        satellites,
        A2,
        A1
    )
    transformDatasetToGlobals(data)
end

function runCompactModel(data::Dataset)
    buildModel(data::Dataset)
end

function calculate_arc_cost(points)
    num_points = length(points)
    arc_cost = zeros(Float64, num_points)
    for idx in 2:length(points)
        arc_cost = hcat(arc_cost, zeros(Float64, num_points))
    end
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

function solveMasterProblem()
    feasible_1e_routes, feasible_2e_routes = generateAllRoutes()
    
    @info ("Number of 1e routes: ", length(feasible_1e_routes))
    @info ("Number of 2e routes: ", length(feasible_2e_routes))

    routes_originated_p = Vector{Vector{Int}}()
    for s in satellites 
        routes = Vector{Int}()
        for (r,route) in enumerate(feasible_2e_routes)
            if route.sequence[1] == s
                push!(routes, r)
            end
        end
        push!(routes_originated_p, routes)
    end

    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, 1>=x[1:length(feasible_1e_routes)]>=0, Int)
    @variable(model, 1>=y[1:length(feasible_2e_routes)]>=0, Int)

    @constraint(model, sync[s in satellites], sum(route.b2out[s] * y[r] for (r, route) in enumerate(feasible_2e_routes))
    -nb_vehicle_per_satellite*sum(route.b1[s] * x[r] for (r, route) in enumerate(feasible_1e_routes))<=0)
    @constraint(model, custVisit[i in customers], 1 - sum(route.a[i-1-length(satellites)] * y[r] for (r, route) in enumerate(feasible_2e_routes)) <= 0 )
    @constraint(model, number2evfixe[s in satellites], sum(route.b2in[s] * y[r] for (r, route) in enumerate(feasible_2e_routes)) == sum(route.b2out[s] * y[r] for (r, route) in enumerate(feasible_2e_routes)))
    @constraint(model, maxVolumnMM[s in satellites], sum( feasible_2e_routes[r].a[i-1-length(satellites
    )]*demands[i]*y[r] for r in routes_originated_p[s-1] for i in customers) - capacity_microhub <= 0)
    @constraint(model, single1eV, sum(x[r] for (r,_) in enumerate(feasible_1e_routes))==1)

    @objective(model, Min, sum(y[r] * route.cost for (r, route) in enumerate(feasible_2e_routes)) + sum(x[r] * route.cost for (r, route) in enumerate(feasible_1e_routes)))

    optimize!(model)

    println("Objective value of master problem is: ",value(objective_value(model)))

    println("Status of 1e route: ")
    for (i,r) in enumerate(feasible_1e_routes)
        if value(x[i]) != 0
            print("   x=",round(value(x[i]),digits=2),"   1e Route ", r.sequence, "   Load= ", r.load ,  "   Cost= ", round(r.cost,digits=2), "   Available parking:")
            for s in satellites
                if r.b1[s] == 1
                    print(" ",s)
                end
            end
            println("")
        end 
    end

    println("Status of 2e route: ")
    for (i,r) in enumerate(feasible_2e_routes)
        if value(y[i]) != 0
            println("   y=",round(value(y[i]),digits=2),"   2e Route ", r.sequence, "   Load= ", r.load , "   Cost= ", round(r.cost,digits=2))
            # tc += routes_2[r].cost
        end
    end
    println("Status of satellite")
    for s in satellites
        freight = 0
        for (r,route) in enumerate(feasible_2e_routes)
            if route.sequence[1] == s && value(y[r])!=0
                freight += route.load
            end
        end
        if freight != 0
            println("   parking $s stores $freight freight")
        end
    end
    return objective_value(model)
end

function generateAllRoutes()
    feasible_1e_routes = generateNonDominate1eRoutes(minimum_parkings_required)
    # feasible_1e_routes = generateBestFeasible1eRoutes(minimum_parkings_required)
    feasible_2e_routes = generateAllFeasible2eRoute()
    return feasible_1e_routes, feasible_2e_routes
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
    
    return new_route
end
