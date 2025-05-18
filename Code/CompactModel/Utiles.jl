include("Globals.jl")

using Plots
using Random
using Plots.PlotMeasures

# Function to calculate distance matrix
function calculate_distance_matrix(x,y,nc)
    function calculate_distance(x1, y1, x2, y2)
        return sqrt((x1 - x2)^2 + (y1 - y2)^2)
    end

    distances = zeros(length(x),length(x))
    for i in 1:length(x)
        for j in 1:length(y)
            if(i==j)
                distances[i,j] = 10000.00
            elseif((i==1 || j==1)&&(length(x)-i<nc || length(y)-j<nc))
                distances[i, j] = 10000.00
            else
                distances[i, j] = (calculate_distance(x[i], y[i], x[j], y[j]))
            end
        end
    end
    
    return distances

end

# Define a structure for the customer data
struct Customer
    cust_no::Int
    xcoord::Int
    ycoord::Int
    demand::Int
    ready_time::Int
    due_date::Int
    service_time::Int
end

function readArgument(value)
    try
        parsedInt = parse(Int, value)
        # println("The argument is as interger as string: ", value)
        return parsedInt
    catch e
        # println("The argument is a true string: ",  value)
        return 0
    end
end

function readFileElison(file_path)
    # Read the file lines
    lines = open(file_path, "r") do file
        readlines(file)
    end

    # Find the index where the customer data starts
    coord_start_index = findfirst(contains.(lines, "NODE_COORD_SECTION")) + 1
    demand_start_index = findfirst(contains.(lines, "DEMAND_SECTION")) 
    end_index = findfirst(contains.(lines, "DEPOT_SECTION")) -1
    capacity_index = findfirst(contains.(lines, "CAPACITY"))
    line = lines[capacity_index]
    parts = split(line, ":")
    capacity = parse(Int, strip(parts[end]))


    x_coor_customers = []
    y_coor_customers = []
    x_coor_depot = []
    y_coor_depot = []
    demands = []

    for i in coord_start_index : end_index
        line = strip(lines[i])
        if !isempty(line)
            fields = split(line) 
            if i == coord_start_index
            # Depot
                push!(x_coor_depot, parse(Int, fields[2]))
                push!(y_coor_depot, parse(Int, fields[3]))
                # zeta = parse(Int, fields[6])
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
    
    return x_coor_customers, y_coor_customers, x_coor_depot, y_coor_depot, demands, capacity
    
end


function readFileSolomon(file_path)
    # Read the file lines
    lines = open(file_path, "r") do file
        readlines(file)
    end

    # Find the index where the customer data starts
    start_index = findfirst(contains.(lines, "CUST NO.")) + 1

    x_coor_customers = []
    x_coor_depot = []
    y_coor_customers = []
    y_coor_depot = []
    demands = []
    time_windows = []
    zeta = 0

    # Parse the customer data
    customers = []

    for i in start_index+1:length(lines)
        line = strip(lines[i])
        if !isempty(line)
            fields = split(line)
            # Depot
            if i == start_index + 1
                push!(x_coor_depot, parse(Int, fields[2]))
                push!(y_coor_depot, parse(Int, fields[3]))
                zeta = parse(Int, fields[6])
            else
                # Customers
                customer = Customer(
                    parse(Int, fields[1]),
                    parse(Int, fields[2]),
                    parse(Int, fields[3]),
                    parse(Int, fields[4]),
                    parse(Int, fields[5]),
                    parse(Int, fields[6]),
                    parse(Int, fields[7])
                )
                push!(customers, customer)
            end
        end
    end
    
    # Fill in the attributes
    for customer in customers
        push!(x_coor_customers, customer.xcoord)
        push!(y_coor_customers, customer.ycoord)
        push!(demands, customer.demand)
        push!(time_windows, (customer.ready_time, customer.due_date))
    end
    
    return x_coor_customers, y_coor_customers, x_coor_depot, y_coor_depot, demands
    
end

function displayMap()

    default(size=(1200, 800))
    # gr()
    ENV["GKSwstype"] = "100" 
 
    # Add the depot point in a different color
    plt = scatter!(x_coor[1:1], y_coor[1:1], 
            markersize = 8, markershape=:square, markercolor = :yellow) 

    # Create a parking scatter plot
    scatter!(plt, x_coor[2:1+np], y_coor[2:1+np], 
            legend = false)      

    # Add the initial parking place in a different color
    for p in 2 : np+1
        if PI[p]==1
            scatter!(plt, [x_coor[p]], [y_coor[p]], 
                markersize = 8, markercolor = :lightblue)
        else
            scatter!(plt, [x_coor[p]], [y_coor[p]], 
            markersize = 6, markercolor = :white)
        end
    end

    # Create a customers scatter plot
    scatter!(plt, x_coor[2+np:1+nc+np], y_coor[2+np:1+nc+np],
                title = "Coordinate Plot",
                legend = false, markersize = 6, markercolor = :pink, 
                marker=:utriangle, markerstrokecolor = :transparent, 
                markerstrokewidth=0, label = "Customers",right_margin=100mm)
    
    return plt
end

function randomGenerateParking(x_coor_customers,y_coor_customers)   
    # Random.seed!(42)

    # Get bounding box
    x_min, x_max = minimum(x_coor_customers), maximum(x_coor_customers)
    y_min, y_max = minimum(y_coor_customers), maximum(y_coor_customers)

    # Generate random nodes within bounding box
    x_coor_parkings = rand(nmm*2) .* (x_max - x_min) .+ x_min
    y_coor_parkings = rand(nmm*2) .* (y_max - y_min) .+ y_min

    PI = vcat(ones(nmm),zeros(nmm))
    shuffle!(PI)
    PI = vcat(0, PI)

    return x_coor_parkings, y_coor_parkings, PI
end

function fixedGenerateParking(x_coor_customers,y_coor_customers)    
    PI = vcat(0, ones(Int, length(specifiedParkings)), zeros(Int, length(specifiedParkings)))

    customers = setdiff!(Set(1:length(x_coor_customers)),Set(specifiedParkings))
    parking_customers = rand(collect(customers), length(specifiedParkings))
    parkings = vcat(specifiedParkings, parking_customers)

    x_coor_parkings = []
    y_coor_parkings = []
    # display(parkings)
    for parking in parkings 
        push!(x_coor_parkings, x_coor_customers[parking])
        push!(y_coor_parkings, y_coor_customers[parking])
    end
    # display(x_coor_parkings)
    return x_coor_parkings, y_coor_parkings, PI
end


function printText(plt, num_y,text::String)
    annotate!(plt, maximum(x_coor[2+np:1+nc+np]) +5,
                                num_y,
                                Plots.text(
                                text,
                                :left,
                                color=:black,
                                6))
    return num_y - 1.5
end


function totalDuration(z, x, itinerary=[])
    # Add the current point to the itinerary
    push!(itinerary, x)
    
    # Check if the vehicle arrives at a parking (assumed to be points in `2:1+np`)
    if x in 2:1+np
        return (0, itinerary)  # Return 0 distance and the itinerary
    end
    
    for k in 2:1+np+nc
        if round(value(z[x, k])) == 1
            # Recur to the next point and add the distance
            distance, sub_itinerary = totalDuration(z, k, itinerary)
            return (distances[x, k] + distance, sub_itinerary)
        end
    end
    
    # If no next point is found, return 0 distance and the current itinerary
    return (0, itinerary)
end


function backTracking(z, colorR, x)
    for k in 2:1+np+nc
        if round(value(z[x,k])) == 1
            plot!([x_coor[x], x_coor[k]], [y_coor[x], y_coor[k]], line=:arrow, color = colorR)

            cox = (x_coor[x] + x_coor[k]) / 2
            coy = (y_coor[x] + y_coor[k]) / 2  # Use y_coor here
            # annotate!(cox, coy, text(round(distances[x, k], digits=2), :center, 6)) 

            if k in 2:1+np   return  end
            backTracking(z, colorR, k)           
            colorR = RGBA(rand(),rand(),rand(),1)
        end
    end
end