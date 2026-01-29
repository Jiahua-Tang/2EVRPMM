
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
    id::Int
    load::Int
    b1::Vector{Int}
    a::Vector{Int}
    b2in::Vector{Int}
    b2out::Vector{Int}
    arrival_time::Vector{Float64}
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

    depth::Int
end

mutable struct BranchingNode
    branchingInfo::BranchingInfo
    cgLowerBound::Float64
    y_value::Vector{Float64}
    isLeaf::Bool
    fractionalScore::Float64
    gradientLB::Float64
    gradientFS::Float64
    id::Int
    parent_id::Int
end


function generateData(nb_customer::Int, random_seed::Int)
    Random.seed!(random_seed)
 
    nb_parking = 4
    nb_microhub = 2
    # nb_customer = 10
    
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

    nb_vehicle_per_satellite = 5
    capacity_1e_vehicle = sum(demands)
    capacity_2e_vehicle = 50
    capacity_microhub = 500
    maximum_duration_2e_vehicle = 50000

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

        max(Int(ceil(sum(demands) / capacity_microhub)), Int(ceil(Int(ceil(sum(demands) / capacity_2e_vehicle))/nb_vehicle_per_satellite))),
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
    global execution_time_limit = 3600 # seconds

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

function read_Solomon_Dataset_TW(filename, execution_time_limit_param::Int)
    
    #==========================================================================
        SECTION 1: INITIALIZE PARAMETERS
    ==========================================================================#
    
    global parkingGenerationRule = "random"
    
    # Vehicle and capacity parameters
    CAPACITY_2E = 100                 # Second-echelon vehicle capacity
    NB_VEHICLES_PER_SATELLITE = 5     # Vehicles available per satellite
    MAX_DURATION_2E = 10000           # Maximum route duration for 2E vehicles
    
    #==========================================================================
        SECTION 2: READ FILE AND PARSE CUSTOMER DATA
    ==========================================================================#
    
    lines = readlines(filename)
    
    # Find data section locations
    vehicle_section = findfirst(x -> occursin("VEHICLE", x), lines)
    customer_section = findfirst(x -> occursin("CUSTOMER", x), lines)
    
    # Parse customer data
    coor_cust_x = Int[]
    coor_cust_y = Int[]
    demands_data = Int[]
    time_window_a = Int[]
    time_window_b = Int[]
    
    # Read all customer records (depot + customers)
    for i in (customer_section + 3):length(lines)
        line = strip(lines[i])
        if !isempty(line)
            parts = split(line)
            push!(coor_cust_x, parse(Int, parts[2]))
            push!(coor_cust_y, parse(Int, parts[3]))
            push!(demands_data, parse(Int, parts[4]))
            push!(time_window_a, parse(Int, parts[5]))
            push!(time_window_b, parse(Int, parts[6]))
        end
    end
    
    # Construct customer coordinates
    coor_cust = [[x, y] for (x, y) in zip(coor_cust_x, coor_cust_y)]
    nb_customer_count = length(demands_data) - 1  # Exclude depot
    
    
    #==========================================================================
        SECTION 3: CALCULATE PARKING REQUIREMENTS
    ==========================================================================#
    
    total_demand = sum(demands_data)
    capacity_microhub = CAPACITY_2E * NB_VEHICLES_PER_SATELLITE
    
    # Calculate minimum parkings
    min_parkings_by_demand = Int(ceil(total_demand / capacity_microhub))
    min_vehicles_needed = Int(ceil(total_demand / CAPACITY_2E))
    min_parkings_by_vehicles = Int(ceil(min_vehicles_needed / NB_VEHICLES_PER_SATELLITE))
    minimum_parkings = max(min_parkings_by_demand, min_parkings_by_vehicles)
    
    # Add buffer for flexibility
    nb_microhub_count = minimum_parkings + 2
    nb_parking_count = nb_microhub_count + 3
    
    println("  Instance: $(basename(filename))")
    println("  Customers: $nb_customer_count")
    println("  Total demand: $total_demand")
    println("  Minimum parkings required: $minimum_parkings")
    println("  Microhubs: $nb_microhub_count, Total parkings: $nb_parking_count")
    
    
    #==========================================================================
        SECTION 4: GENERATE PARKING LOCATIONS AND NETWORK
    ==========================================================================#
    
    # Generate parking coordinates
    result = generateParkingCoor(coor_cust, nb_microhub_count, nb_parking_count)
    parking_avail = result[1]
    coor_parkings = result[2]
    
    # Construct full coordinate list: [depot, parkings, customers]
    coor_full = vcat(coor_cust[1:1], coor_parkings, coor_cust[2:end])
    
    # Construct demand vector: [depot=0, parkings=0, customers]
    demands_full = vcat(demands_data[1], zeros(Int, nb_parking_count), demands_data[2:end])
    
    # Extend time windows for depot and parkings
    max_time = maximum(time_window_b)
    time_window_a_full = vcat(zeros(Int, 1 + nb_parking_count), time_window_a[2:end])
    time_window_b_full = vcat(fill(max_time, 1 + nb_parking_count), time_window_b[2:end])
    time_window_full = [[a, b] for (a, b) in zip(time_window_a_full, time_window_b_full)]
    
    # Calculate arc costs (distances)
    arc_cost_matrix = calculate_arc_cost(coor_full)
    
    
    #==========================================================================
        SECTION 5: DEFINE NODE SETS
    ==========================================================================#
    
    n_nodes = length(demands_full)
    
    # Node indices
    points_set = 1:n_nodes
    satellites_set = 2:(1 + nb_parking_count)                      # Parking/satellite nodes
    customers_set = (2 + nb_parking_count):n_nodes                  # Customer nodes
    A2_set = 2:n_nodes                                              # All nodes except depot
    A1_set = 1:(1 + length(satellites_set))                         # Depot + satellites
    
    
    #==========================================================================
        SECTION 6: SET GLOBAL VARIABLES
    ==========================================================================#
    
    # Network structure
    global nb_customer = nb_customer_count
    global nb_parking = nb_parking_count
    global nb_microhub = nb_microhub_count
    global coor = coor_full
    global arc_cost = arc_cost_matrix
    global demands = demands_full
    global parking_availability = parking_avail
    
    # Time windows
    global time_window = time_window_full
    global planning_horizon = max_time
    
    # Vehicle capacities
    global capacity_2e_vehicle = CAPACITY_2E
    global capacity_1e_vehicle = total_demand  # First-echelon serves all demand
    global capacity_microhub = capacity_microhub
    global maximum_duration_2e_vehicle = MAX_DURATION_2E
    global nb_vehicle_per_satellite = NB_VEHICLES_PER_SATELLITE
    
    # Minimum requirements
    global minimum_parkings_required = minimum_parkings
    global minimum_2e_vehicle_required = Int(ceil(total_demand / CAPACITY_2E))
    
    # Node sets
    global points = points_set
    global customers = customers_set
    global satellites = satellites_set
    global A2 = A2_set
    global A1 = A1_set
    
    # Setup virtual node arcs after satellites are defined
    virtual_start_idx = Base.size(arc_cost, 1) - 1  # second-to-last index
    virtual_end_idx = Base.size(arc_cost, 1)         # last index
    
    # One-direction arcs: Virtual start → Satellites (cost = 0)
    for sat in satellites_set
        arc_cost[virtual_start_idx, sat] = 0.0
        arc_cost[sat, virtual_start_idx] = 100000.0  # No reverse arc
    end
    
    # One-direction arcs: Satellites → Virtual end (cost = 0)
    for sat in satellites_set
        arc_cost[sat, virtual_end_idx] = 0.0
        arc_cost[virtual_end_idx, sat] = 100000.0  # No reverse arc
    end
    
    # Export virtual node indices as global variables
    global virtual_start = virtual_start_idx
    global virtual_end = virtual_end_idx
    
    # Execution parameters
    global execution_time_limit = execution_time_limit_param
    
    #region: output
    #==========================================================================
        SECTION 7: DEBUG OUTPUT (OPTIONAL)
    ==========================================================================#
    
    # println("\nVirtual Nodes Configuration:")
    # println("  Virtual Start Node: $(virtual_start)")
    # println("  Virtual End Node: $(virtual_end)")
    # println("  Arcs from Virtual Start to Satellites: cost = 0.0")
    # println("  Arcs from Satellites to Virtual End: cost = 0.0")
    # println("  Arc cost matrix size: $(Base.size(arc_cost))")
    
    # # Print arc costs involving virtual nodes
    # println("\nArc costs from Virtual Start ($(virtual_start)) to all nodes:")
    # for i in 1:Base.size(arc_cost, 2)
    #     cost = arc_cost[virtual_start, i]
    #     if cost < 100000.0
    #         node_type = i == 1 ? "Depot" : i in satellites_set ? "Satellite" : i in customers_set ? "Customer" : "Virtual"
    #         println("  $(virtual_start) → $(i) ($node_type): $(round(cost, digits=2))")
    #     end
    # end
    
    # println("\nArc costs from all nodes to Virtual End ($(virtual_end)):")
    # for i in 1:Base.size(arc_cost, 1)
    #     cost = arc_cost[i, virtual_end]
    #     if cost < 100000.0
    #         node_type = i == 1 ? "Depot" : i in satellites_set ? "Satellite" : i in customers_set ? "Customer" : "Virtual"
    #         println("  $(i) ($node_type) → $(virtual_end): $(round(cost, digits=2))")
    #     end
    # end
    
    # # Verify that virtual nodes are NOT connected to customers
    # println("\nVerification - Virtual nodes are NOT connected to customers:")
    # has_customer_connection = false
    # for cust in customers_set
    #     if arc_cost[virtual_start, cust] < 100000.0 || arc_cost[cust, virtual_start] < 100000.0 ||
    #        arc_cost[virtual_end, cust] < 100000.0 || arc_cost[cust, virtual_end] < 100000.0
    #         has_customer_connection = true
    #         println("  WARNING: Found connection between virtual nodes and customer $(cust)")
    #     end
    # end
    # if !has_customer_connection
    #     println("  ✓ Confirmed: No direct arcs between virtual nodes and customers (all costs = 100000.0)")
    # end
    #endregion
    
    println("\nCustomer Details:")
    for cust in customers_set
        cust_id = cust - 1 - nb_parking_count
        demand = demands_full[cust]
        tw = time_window_full[cust]
        println("  Customer $(1+nb_parking+rpad(cust_id, 3)):  q=$(rpad(demand, 2)), tw= [$(rpad(tw[1], 3)), $(rpad(tw[2], 3))]")
    end
    println()
end

function read_nico_dataset(filename)
    
    #==========================================================================
        SECTION 1: INITIALIZE PARAMETERS
    ==========================================================================#
    CAPACITY_1E = 200                 # First-echelon vehicle capacity
    CAPACITY_2E = 50                  # Second-echelon vehicle capacity
    NB_VEHICLES_PER_SATELLITE = 50    # Vehicles available per satellite
    MAX_DURATION_2E = 10000           # Maximum route duration for 2E vehicles

    #==========================================================================
        SECTION 2: READ FILE AND PARSE ALL CUSTOMER DATA
    ==========================================================================#
    lines = readlines(filename)

    # Parse ALL customer data first
    coor_cust_x = Int[]
    coor_cust_y = Int[]
    demands_data = Int[]
    time_window_a = Int[]
    time_window_b = Int[]

    # Read all lines
    for line in lines
        line = strip(line)
        if !isempty(line)
            parts = split(line)
            if length(parts) == 6
                # Read customer info
                
            else
                if parse(Int, parts[3]) != 0
                    # Read satellite info


                else
                    # Read depot info
                    
                end
            end
            
        end
    end
    


end

function retrieve_solomon_random_data(filename, execution_time_limit_param::Int, size::Int)
    
    #==========================================================================
        SECTION 1: INITIALIZE PARAMETERS
    ==========================================================================#
    
    global parkingGenerationRule = "random"
    
    # Vehicle and capacity parameters
    CAPACITY_2E = 100                 # Second-echelon vehicle capacity
    NB_VEHICLES_PER_SATELLITE = 5     # Vehicles available per satellite
    MAX_DURATION_2E = 10000           # Maximum route duration for 2E vehicles
    
    
    #==========================================================================
        SECTION 2: READ FILE AND PARSE ALL CUSTOMER DATA
    ==========================================================================#
    
    lines = readlines(filename)
    
    # Find data section locations
    vehicle_section = findfirst(x -> occursin("VEHICLE", x), lines)
    customer_section = findfirst(x -> occursin("CUSTOMER", x), lines)
    
    # Parse ALL customer data first
    coor_cust_x = Int[]
    coor_cust_y = Int[]
    demands_data = Int[]
    time_window_a = Int[]
    time_window_b = Int[]
    
    # Read all customer records (depot + customers)
    for i in (customer_section + 3):length(lines)
        line = strip(lines[i])
        if !isempty(line)
            parts = split(line)
            push!(coor_cust_x, parse(Int, parts[2]))
            push!(coor_cust_y, parse(Int, parts[3]))
            push!(demands_data, parse(Int, parts[4]))
            push!(time_window_a, parse(Int, parts[5]))
            push!(time_window_b, parse(Int, parts[6]))
        end
    end
    
    total_customers_in_file = length(demands_data) - 1  # Exclude depot
    
    # Validate subset size
    if size > total_customers_in_file
        error("Requested $size customers but file only has $total_customers_in_file customers")
    end
    
    
    #==========================================================================
        SECTION 3: RANDOMLY SELECT CUSTOMER SUBSET
    ==========================================================================#
    
    # Set random seed if global random_seed exists
    if @isdefined(random_seed)
        Random.seed!(random_seed)
        # println("  Using random seed: $random_seed for reproducibility")
    end
    
    # Randomly select customer indices (excluding depot at index 1)
    available_customer_indices = 2:length(demands_data)
    selected_indices = sort(shuffle(collect(available_customer_indices))[1:size])
    
    # Keep depot (index 1) and selected customers
    keep_indices = vcat([1], selected_indices)
    
    # Extract data for selected customers
    coor_cust_x_subset = coor_cust_x[keep_indices]
    coor_cust_y_subset = coor_cust_y[keep_indices]
    demands_data_subset = demands_data[keep_indices]
    time_window_a_subset = time_window_a[keep_indices]
    time_window_b_subset = time_window_b[keep_indices]
    
    coor_cust = [[x, y] for (x, y) in zip(coor_cust_x_subset, coor_cust_y_subset)]
    
    println("  Dataset: $(basename(filename))")
    println("  Total customers in file: $total_customers_in_file")
    println("  Selected customers: $size (randomly chosen)")
    println("  Selected customer IDs: $(selected_indices .- 1)")  # Show original IDs
    
    
    #==========================================================================
        SECTION 4: CALCULATE PARKING REQUIREMENTS
    ==========================================================================#
    
    total_demand = sum(demands_data_subset)
    capacity_microhub = CAPACITY_2E * NB_VEHICLES_PER_SATELLITE
    
    # Calculate minimum parkings
    min_parkings_by_demand = Int(ceil(total_demand / capacity_microhub))
    min_vehicles_needed = Int(ceil(total_demand / CAPACITY_2E))
    min_parkings_by_vehicles = Int(ceil(min_vehicles_needed / NB_VEHICLES_PER_SATELLITE))
    minimum_parkings = max(min_parkings_by_demand, min_parkings_by_vehicles)
    
    # Add buffer for flexibility
    nb_microhub_count = minimum_parkings + 2
    nb_parking_count = nb_microhub_count + 3
    
    println("  Total demand: $total_demand")
    println("  Minimum parkings required: $minimum_parkings")
    println("  Microhubs: $nb_microhub_count, Total parkings: $nb_parking_count")
    
    
    #==========================================================================
        SECTION 5: GENERATE PARKING LOCATIONS AND NETWORK
    ==========================================================================#
    
    # Generate parking coordinates
    result = generateParkingCoor(coor_cust, nb_microhub_count, nb_parking_count)
    parking_avail = result[1]
    coor_parkings = result[2]
    
    # Construct full coordinate list: [depot, parkings, customers]
    coor_full = vcat(coor_cust[1:1], coor_parkings, coor_cust[2:end])
    
    # Construct demand vector: [depot=0, parkings=0, customers]
    demands_full = vcat(demands_data_subset[1], zeros(Int, nb_parking_count), demands_data_subset[2:end])
    
    # Extend time windows for depot and parkings
    max_time = maximum(time_window_b_subset)
    time_window_a_full = vcat(zeros(Int, 1 + nb_parking_count), time_window_a_subset[2:end])
    time_window_b_full = vcat(fill(max_time, 1 + nb_parking_count), time_window_b_subset[2:end])
    time_window_full = [[a, b] for (a, b) in zip(time_window_a_full, time_window_b_full)]
    
    # Calculate arc costs (distances)
    arc_cost_matrix = calculate_arc_cost(coor_full)
    
    
    #==========================================================================
        SECTION 6: DEFINE NODE SETS
    ==========================================================================#
    
    n_nodes = length(demands_full)
    
    # Node indices
    points_set = 1:n_nodes
    satellites_set = 2:(1 + nb_parking_count)                      # Parking/satellite nodes
    customers_set = (2 + nb_parking_count):n_nodes                  # Customer nodes
    A2_set = 2:n_nodes                                              # All nodes except depot
    A1_set = 1:(1 + length(satellites_set))                         # Depot + satellites
    
    
    #==========================================================================
        SECTION 7: SET GLOBAL VARIABLES
    ==========================================================================#
    
    # Network structure
    global nb_customer = size
    global nb_parking = nb_parking_count
    global nb_microhub = nb_microhub_count
    global coor = coor_full
    global arc_cost = arc_cost_matrix
    global demands = demands_full
    global parking_availability = parking_avail
    
    # Time windows
    global time_window = time_window_full
    global planning_horizon = max_time
    
    # Vehicle capacities
    global capacity_2e_vehicle = CAPACITY_2E
    global capacity_1e_vehicle = total_demand  # First-echelon serves all demand
    global capacity_microhub = capacity_microhub
    global maximum_duration_2e_vehicle = MAX_DURATION_2E
    global nb_vehicle_per_satellite = NB_VEHICLES_PER_SATELLITE
    
    # Minimum requirements
    global minimum_parkings_required = minimum_parkings
    global minimum_2e_vehicle_required = Int(ceil(total_demand / CAPACITY_2E))
    
    # Node sets
    global points = points_set
    global customers = customers_set
    global satellites = satellites_set
    global A2 = A2_set
    global A1 = A1_set
    
    # Setup virtual node arcs after satellites are defined
    virtual_start_idx = Base.size(arc_cost, 1) - 1  # second-to-last index
    virtual_end_idx = Base.size(arc_cost, 1)         # last index
    
    # One-direction arcs: Virtual start → Satellites (cost = 0)
    for sat in satellites_set
        arc_cost[virtual_start_idx, sat] = 0.0
        arc_cost[sat, virtual_start_idx] = 100000.0  # No reverse arc
    end
    
    # One-direction arcs: Satellites → Virtual end (cost = 0)
    for sat in satellites_set
        arc_cost[sat, virtual_end_idx] = 0.0
        arc_cost[virtual_end_idx, sat] = 100000.0  # No reverse arc
    end
    
    # Export virtual node indices as global variables
    global virtual_start = virtual_start_idx
    global virtual_end = virtual_end_idx
    
    # Execution parameters
    global execution_time_limit = execution_time_limit_param
    
    #region : DEBUG OUTPUT
    #==========================================================================
        SECTION 8: DEBUG OUTPUT (OPTIONAL)
    ==========================================================================#
    
    # println("\nVirtual Nodes Configuration:")
    # println("  Virtual Start Node: $(virtual_start)")
    # println("  Virtual End Node: $(virtual_end)")
    # println("  Arcs from Virtual Start to Satellites: cost = 0.0")
    # println("  Arcs from Satellites to Virtual End: cost = 0.0")
    # println("  Arc cost matrix size: $(Base.size(arc_cost))")
    
    # # Print arc costs involving virtual nodes
    # println("\nArc costs from Virtual Start ($(virtual_start)) to all nodes:")
    # for i in 1:Base.size(arc_cost, 2)
    #     cost = arc_cost[virtual_start, i]
    #     if cost < 100000.0
    #         node_type = i == 1 ? "Depot" : i in satellites_set ? "Satellite" : i in customers_set ? "Customer" : "Virtual"
    #         println("  $(virtual_start) → $(i) ($node_type): $(round(cost, digits=2))")
    #     end
    # end
    
    # println("\nArc costs from all nodes to Virtual End ($(virtual_end)):")
    # for i in 1:Base.size(arc_cost, 1)
    #     cost = arc_cost[i, virtual_end]
    #     if cost < 100000.0
    #         node_type = i == 1 ? "Depot" : i in satellites_set ? "Satellite" : i in customers_set ? "Customer" : "Virtual"
    #         println("  $(i) ($node_type) → $(virtual_end): $(round(cost, digits=2))")
    #     end
    # end
    
    # # Verify that virtual nodes are NOT connected to customers
    # println("\nVerification - Virtual nodes are NOT connected to customers:")
    # has_customer_connection = false
    # for cust in customers_set
    #     if arc_cost[virtual_start, cust] < 100000.0 || arc_cost[cust, virtual_start] < 100000.0 ||
    #        arc_cost[virtual_end, cust] < 100000.0 || arc_cost[cust, virtual_end] < 100000.0
    #         has_customer_connection = true
    #         println("  WARNING: Found connection between virtual nodes and customer $(cust)")
    #     end
    # end
    # if !has_customer_connection
    #     println("  ✓ Confirmed: No direct arcs between virtual nodes and customers (all costs = 100000.0)")
    # end
    #endregion
    
    println("\nCustomer Details (remapped IDs):")
    for cust in customers_set
        cust_id = cust - 1 - nb_parking_count
        demand = demands_full[cust]
        tw = time_window_full[cust]
        println("  Customer $(rpad(cust_id, 3)):  q= $(rpad(demand, 2)), tw= [$(rpad(tw[1], 3)), $(rpad(tw[2], 3))]")
    end
    println()
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
    nb_vehicle_per_satellite = 4
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
        nb_vehicle_per_satellite*capacity_2e_vehicle,
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
    
    # Add 2 more positions for virtual start and end nodes
    total_size = num_points + 2
    arc_cost = zeros(Float64, total_size, total_size)
    
    # Calculate distances for original points
    for i in 1:num_points
        for j in 1:num_points
            if i == j
                arc_cost[i, j] = 100000.0  # Large value for self-loops
            else
                arc_cost[i, j] = sqrt(sum((points[i][k] - points[j][k])^2 for k in 1:2))
            end
        end
    end
    
    # Initialize virtual nodes with infinite costs
    virtual_start = num_points + 1
    virtual_end = num_points + 2
    
    # Set all arcs to/from virtual nodes to infinity initially
    arc_cost[virtual_start, :] .= 100000.0
    arc_cost[:, virtual_start] .= 100000.0
    arc_cost[virtual_end, :] .= 100000.0
    arc_cost[:, virtual_end] .= 100000.0
    
    # Virtual nodes' self-loops
    arc_cost[virtual_start, virtual_start] = 100000.0
    arc_cost[virtual_end, virtual_end] = 100000.0
    
    # One-direction arcs from virtual start node TO satellites (will be set later with actual satellite indices)
    # One-direction arcs from satellites TO virtual end node (will be set later with actual satellite indices)
    # Note: Satellite indices are typically 2:1+nb_parking
    # These will be set to 0 cost later once satellites are defined
    
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
