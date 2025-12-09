include("Globals.jl")

using Plots
using Random

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

function readFile(file_path)
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

    default(size=(1000, 800))
    # gr()
    ENV["GKSwstype"] = "100" 
    
    # Create a customers scatter plot
    scatter(x_coor[2+np:1+nc+np], y_coor[2+np:1+nc+np],
                title = "Coordinate Plot",
                xlabel = "X-axis",
                ylabel = "Y-axis",
                legend = false, markersize = 6, markercolor = :pink, 
                marker=:utriangle, markerstrokecolor = :transparent, 
                markerstrokewidth=0, label = "Customers")

    # Create a parking scatter plot
    scatter!(x_coor[2:1+np], y_coor[2:1+np], 
            title = "Coordinate Plot",
            xlabel = "X-axis", ylabel = "Y-axis",
            legend = false)

    # Add the depot point in a different color
    scatter!(x_coor[1], y_coor[1], 
            markersize = 8, markercolor = :yellow)

    # Add the initial parking place in a different color
    for p in 2 : np+1
        if PI[p-1]==1
            scatter!([x_coor[p]], [y_coor[p]], 
                markersize = 8, markercolor = :lightblue)
        else
            scatter!([x_coor[p]], [y_coor[p]], 
            markersize = 6, markercolor = :white)
        end
    end
end

function randomGenerateParking(x_coor_customers,y_coor_customers) 
    Random.seed!(42)  
    function generatePI(num :: Int)
        PI = vcat(ones(2),zeros(num-2))
        shuffle!(PI)
        for i in 1:num-1
            p = vcat(ones(2),zeros(num-2))
            shuffle!(p)
            PI = vcat(PI,p)
        end    
        i=1
        if num == 4     l = 2
        else   l = 6     end
        while i<=l
            p = rand(1:num^2)
            if PI[p] == 0
                PI[p] = 1
                i = i + 1
            end
        end
        return PI
    end

    # Define the boundaries of the customer area
    x_min, x_max = minimum(x_coor_customers), maximum(x_coor_customers)
    y_min, y_max = minimum(y_coor_customers), maximum(y_coor_customers)

    # Define the number of divisions (4x4 grid for <=50, 5x5 grid for >50 <=100)
    # if length(x_coor_customers) <=50
        num_divisions = 4
    # else
    #     if length(x_coor_customers) <=100
    #         num_divisions = 5      
    #     end
    # end
    PI = generatePI(num_divisions)

    x_step = (x_max - x_min) / num_divisions
    y_step = (y_max - y_min) / num_divisions

    # Initialize empty arrays to store the parking coordinates
    x_coor_parkings = []
    y_coor_parkings = []

    # Loop over the grid and generate a random point in each sub-area
    for i in 0:num_divisions-1
        for j in 0:num_divisions-1
            # Define the boundaries of the current sub-area
            x_lower = x_min + i * x_step
            x_upper = x_min + (i + 1) * x_step
            y_lower = y_min + j * y_step
            y_upper = y_min + (j + 1) * y_step

            # Generate a random point within the current sub-area
            push!(x_coor_parkings, rand(x_lower:x_upper))
            push!(y_coor_parkings, rand(y_lower:y_upper))
        end
    end
    
    # println("PI : ", PI)

    return x_coor_parkings, y_coor_parkings, PI
end

function backTracking(z, colorR, x)
    for k in 2:1+np+nc
        if round(value(z[x,k])) == 1
            plot!([x_coor[x], x_coor[k]], [y_coor[x], y_coor[k]], line=:arrow, color = colorR)

            cox = (x_coor[x] + x_coor[k]) / 2
            coy = (y_coor[x] + y_coor[k]) / 2  # Use y_coor here
            annotate!(cox, coy, text(round(distances[x, k], digits=2), :center, 6)) 

            if k in 2:1+np   return  end
            backTracking(z, colorR, k)           
            colorR = RGBA(rand(),rand(),rand(),1)
        end
    end

end