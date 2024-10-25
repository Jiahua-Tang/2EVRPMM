using JuMP, CPLEX
model = Model(CPLEX.Optimizer)
set_attribute(model, "CPX_PARAM_EPINT", 1e-8)
set_optimizer_attribute(model, "CPX_PARAM_MIPSEARCH", 1)
using Printf
using Plots
using FileIO
using Random
using CSV
using Dates

x_coor = []
y_coor = []
np = 0
nc = 0
PI = []
V2 = 1:10

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
    
    return x_coor_customers, y_coor_customers, x_coor_depot, y_coor_depot, demands, time_windows, zeta
    
end

function displayMap()
   
    default(size=(1000, 800))
    gr()
    
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
    scatter!([x_coor[1]], [y_coor[1]], 
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
    # plot!() 
end

function randomGenerateParking(x_coor_customers,y_coor_customers)   
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
    if length(x_coor_customers) <=50
        num_divisions = 4
    else
        if length(x_coor_customers) <=100
            num_divisions = 5      
        end
    end
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

function fixedGenerateParking(x_coor_customers,y_coor_customers)    
    # Define the boundaries of the customer area
    x_min, x_max = minimum(x_coor_customers), maximum(x_coor_customers)
    y_min, y_max = minimum(y_coor_customers), maximum(y_coor_customers)

    # Define the number of divisions (4x4 grid for <=50, 5x5 grid for >50 <=100)
    if length(x_coor_customers) <=50
        num_divisions = 5
        PI = [0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1] #10 MM
    else
        if length(x_coor_customers)<=100
            num_divisions = 6
            PI = [1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1] #16 MM
        end
    end
    x_step = (x_max - x_min) / num_divisions
    y_step = (y_max - y_min) / num_divisions

    # Initialize empty arrays to store the parking coordinates
    x_coor_parkings = []
    y_coor_parkings = []

    for j in 1:num_divisions-1
        for i in 1:num_divisions-1
            push!(x_coor_parkings, x_min + i * x_step)
            push!(y_coor_parkings, y_min + j * y_step)
        end
    end

    return x_coor_parkings, y_coor_parkings, PI
end

function backTracking(z, colorR, x)
    for k in 2:1+np+nc
        if round(value(z[x,k])) == 1
            plot!([x_coor[x], x_coor[k]], [y_coor[x], y_coor[k]], line=:arrow, color = colorR)
            if k in 2:1+np   return  end
            backTracking(z, colorR, k)           
            colorR = RGBA(rand(),rand(),rand(),1)
        end
    end
end

function runModel(filePath::String, Q1::Int, Q2::Int, minutes::Int, case::String)

    #===========SET==============================================================#
    x_coor_customers, y_coor_customers, x_coor_depot, y_coor_depot, demands, time_windows, zeta = readFile(filePath)
    # Number of customers
    global nc 
    nc = length(x_coor_customers)
    # Coordinate (Parking node)
    global PI 
    if case == "r"
        x_coor_parkings, y_coor_parkings, PI = randomGenerateParking(x_coor_customers, y_coor_customers)
    else
        x_coor_parkings, y_coor_parkings, PI = fixedGenerateParking(x_coor_customers, y_coor_customers)
    end
    # Number of parking places
    global np 
    np = length(x_coor_parkings)
    P = 2 : np+1 #Set of parking place
    C = np+2 : np+nc+1 #Set of customers
    A1 = 1 : 1+np #Set of FE arcs
    A2 = 2 : 1+np+nc #Set of SE arcs
    V1 = 1:1 #Set of FEV
    V2 = 1:10 #Set of SEV
    N = 1:np+nc+1 #Set of nodes

    #===========PARAMETER========================================================#
    Q0 = 10000 #Capacity of FEV
    # Q1 = 6000 #Capacity of MM
    # Q2 = 2000 #Capacity of SEV
    M = 10000

    eta1 = 2
    eta2 = 1

    global x_coor
    x_coor = vcat(x_coor_depot, x_coor_parkings, x_coor_customers)
    global y_coor
    y_coor = vcat(y_coor_depot, y_coor_parkings, y_coor_customers)

    node_labels = [string("N.", i) for i in N]
    demand_labels = [string(demands[i-1-np]," ",time_windows[i-1-np]) for i in C]

    # Disatance matrix
    distances = calculate_distance_matrix(x_coor,y_coor,nc)
    Vitesse1 = 1
    Vitesse2 = 1
    TT1 = (distances / Vitesse1)
    TT2 = (distances / Vitesse2)

    println("Number of customers: ",nc)
    displayMap()

    demand_labels = [string(demands[i-1-np]) for i in C]
    tw_labels = [string(time_windows[i-1-np]) for i in C]
    
    for i in N
        annotate!(x_coor[i], y_coor[i]+0.3, text(node_labels[i], :center, 4))
        if i in C
            annotate!(x_coor[i]-0.3, y_coor[i]-0.3, text(demand_labels[i-1-np], :center, 6)) 
            annotate!(x_coor[i]-0.3, y_coor[i]-2, text(tw_labels[i-1-np], :center, 4)) 
        end 
    end 

   # Extract the file name (without extension) and create the desired fileName
    baseName = splitext(basename(filePath))[1]  # This extracts the base name without extension, e.g., C101-10
    fileName = baseName * "-Q" * string(Q2) *"-" *case # e.g., C101-10-Q100

    # Create the save directory, avoiding duplication of the fileName inside the path
    save_dir = joinpath("./Result", baseName,fileName)  # Only use baseName for the directory structure
    println(save_dir)
    # Paths for saving files
    data_path_svg = joinpath(save_dir, fileName * "-data.svg")
    data_path_png = joinpath(save_dir, fileName * "-data.png")

    # Check if the directory exists, if not, create it (use mkpath instead of mkdir)
    if !isdir(save_dir)
        mkpath(save_dir)  # mkpath will create all necessary directories along the path
    end

    # Print paths to verify correctness
    # println("SVG path: ", data_path_svg)
    # println("PNG path: ", data_path_png)
    savefig(data_path_png)
    savefig(data_path_svg)
     #=================================================================================================#
     model=Model(CPLEX.Optimizer)

     # Decision variable
     @variable(model, x[A1,A1], Bin)#Arc(x,y) traversed by FEV
     @variable(model, y[A1,A1], Bin)#Arc(x,y) traversed by MM
     for i in 1:np+1
         @constraint(model, x[i, i] == 0)
         @constraint(model, y[i, i] == 0)
     end
     @variable(model, t[A2]>=0, Int) #Arrival time
     @variable(model, w[P]>=0, Int) #Amount of freight transported from the depot to parking node
     @variable(model, z[A2,A2], Bin)#Arc(x,y) traversed by SEV
     for i in A2
         @constraint(model, z[i,i] == 0)
         for j in A2
             if i in P && j in P
                 @constraint(model, z[i,j]==0)
             end
         end
     end
     @variable(model, f[A2,A2]>=0,Int) #Load of SEV
     #======================================================================#
     @objective(model, Min,
         sum(distances[i, j] * x[i, j] for i in A1, j in A1 if i != j) +
         sum(distances[i, j] * y[i, j] for i in A1, j in A1 if i != j) +
         sum(distances[i, j] * z[i, j] for i in A2, j in A2 if i != j))
     #======================================================================#
     #1 #2
     #Flow conservation at parking for FEV
     @constraint(model, [i in P], sum(x[j,i] for j in A1 if i != j) == sum(x[i,j] for j in A1 if i != j))
     @constraint(model, [i in P], sum(x[i,j] for j in A1 if i != j) <= 1)
     #3
     #Flow conservation of MM
     @constraint(model, [i in P], sum(y[i,j] for j in A1 if i != j) + sum(y[j,i] for j in A1 if i != j)<=1)
     #4
     #Limit for mobile microhub
     @constraint(model, [i in A1, j in A1], y[i,j] <= x[i,j])
     #5 #6
     #Flow conservation at depot
     @constraint(model, sum(x[1,j] for j in A1 if j !=1)==1)
     @constraint(model, sum(x[j,1] for j in A1 if j !=1)==1)
     @constraint(model, [i in A1], y[1,i] ==0)
     @constraint(model, [i in A1], y[i,1] ==0)
     #7
     #Capacity limit for FEV
     @constraint(model, sum(w[p] for p in P)<=Q0)
     #8
     #Can't tow a MM from parking without MM
     @constraint(model, [i in P], sum(y[i,j] for j in A1)<= PI[i-1])
     #9
     #Can't tow a MM to a parking occupied
     @constraint(model, [j in P], sum(y[i,j] for i in A1)<=1-PI[j-1])
     #10
     #If MM leaves a site, the freight to the site should be zero, otw could be positive
     @constraint(model, [p in P], w[p] <= Q1 * (1-sum(y[p,j] for j in A1)))
     #11
     #Link 1st and 2nd
     @constraint(model, [p in P], w[p] == sum(f[p,j] for j in A2 if p !=j))
     #12
     #Capacity limit for MM and connection of FEV
     @constraint(model, [p in P], w[p] <= Q1 * sum(x[i,p] for i in A1))
     #13
     #Can't distribute from a site without MM
     @constraint(model, [p in P], w[p]<=Q1*(sum(y[i,p] for i in A1)+PI[p-1]))
     
     #14
     #Flow consercvation at parking and customer for SEV
     @constraint(model, [i in A2], sum(z[i,j] for j in A2) == sum(z[j,i] for j in A2))
     #15
     #Flow consercvation at parking node for SEV
     @constraint(model, [p in P], sum(z[p,j] for j in A2) <= length(V2))
     #16
     #Each SEV departs from parking node at most once
    #  @constraint(model, [p in P], sum(z[p,j] for j in A2) <= 1)
     
     #17
     #Flow conservation at customer node for SEV
     @constraint(model, [i in C], sum(z[i,j] for j in A2) == 1)
     #18
     #Customer demand met
     @constraint(model, [i in C], sum(f[j,i] for j in A2)-sum(f[i,j] for j in A2) == demands[i-1-np])
     #19
     #Connection and capacity limit for SEV
     @constraint(model, [i in A2, j in A2], f[i,j] <= Q2 * sum(z[i,j]))
     #20 #21
     #Total working time cannot exceed the length of planning horizon
     @constraint(model, sum(TT1[i,j]*x[i,j] for i in A1 for j in A1) + eta1*sum(PI[p-1]*x[i,p] for p in P for i in A1)<= zeta)
     @constraint(model, [i in C, j in P], t[i]+TT2[i,j]+eta2 <= zeta + M*(1 - z[i,j]))
     #22
     #Time constraint for FEV and MTZ
     @constraint(model, [i in P, j in P], t[i] + eta1*(1-x[i,j]) + TT1[i,j]*x[i,j] <= t[j] + M*(1 - x[i,j]))
     #23
     #Time constraint for SEV and MTZ
     @constraint(model, [i in C, j in C], t[i]+eta2*(1-z[i,j])+TT2[i,j]*z[i,j] <= t[j]+M * (1 - z[i,j]))
     #24
     @constraint(model, [i in C], t[i] >= time_windows[i-1-np][1])
     @constraint(model, [i in C], t[i] <= time_windows[i-1-np][2])
     #25 26
     #Arrival time initialization
     @constraint(model, [i in P], TT1[1,i] * x[1,i] <= t[i])
     @constraint(model, [p in P, j in C], t[p] + TT2[p,j] * z[p,j] <= t[j])
     
 #=================================================================================================#

    set_optimizer_attribute(model, "CPX_PARAM_TILIM", 60 * minutes)
    # Solve the model
    total_time = @elapsed optimize!(model)
    resultStatus = ""
    currentTime = Dates.format(now(), "dd-mm-yyyy HH:MM")
    # Check solver status and print results
    if termination_status(model) == MOI.OPTIMAL
        println("Optimal solution found!")
        resultStatus = "-O"
    elseif primal_status(model) == MOI.FEASIBLE_POINT
        println("Feasible solution found within the time limit!")
        resultStatus = "-F"
    else
        println("No feasible solution found.")
        row_data = [currentTime, fileName, Q0, Q1, Q2, np, sum(PI), case, minutes, "No feasible solution found"]
        open("./Result/output.csv", "a") do file
            println(file, join(row_data, ",")) 
        end
        return
    end
    println("Total execution time: $total_time seconds")
    println("Total distance traveled: ", objective_value(model))
    println("File name: ", fileName)
    println("Number of customers: ", nc)
    println("Capacity of FEV: ", Q0)
    println("Capacity of Microhub: ", Q1)
    println("Capacity of SEV: ", Q2)
    println("Number of parkings: ", np)
    println("Number of microhubs: ", sum(PI))
    println("Number of robots/MM: ", length(V2))
    println("Parking generation rule: ", case)
   

    # New data to append
    # Time / Filename / Cap V1 / Cap MM / Cap V2 / #Parking / #MM / #Robot / Parking generation rule / Limit time / Total Distance / Execution time 
    row_data = [currentTime, fileName, Q0, Q1, Q2, np, sum(PI), length(V2), case, minutes, objective_value(model), total_time]
    open("./Result/output.csv", "a") do file
        println(file, join(row_data, ",")) 
    end


   println("==========================================================================")
    if primal_status(model) == MOI.FEASIBLE_POINT
        light_green = RGBA(0.5, 1.0, 0.5, 1.0)
        displayMap()
#     time_labels = [string("t= ", round(value(t[i]))) for i in A2]
#     node_labels = [string("N.", i) for i in N]
        for i in N
            # Add node number lable
            # annotate!(x_coor[i], y_coor[i]+0.3, text(node_labels[i], :center, 4))
            
            if i in A1
                # Add FEV arcs between the locations if they are traversed
                for j in A1         
                    if round(value(y[i, j])) == 1
                        plot!([x_coor[i], x_coor[j]], [y_coor[i], y_coor[j]],line=:arrow,color = light_green, linealpha=4, lw=4)
                    end
                    if round(value(x[i, j])) == 1
                        plot!([x_coor[i], x_coor[j]], [y_coor[i], y_coor[j]],line=:arrow,color = :black,linestyle=:dash)
                    end
                end
                # Add SEV arcs between the locations if they are traverse
                if i in P
                    colorR = RGBA(rand(),rand(),rand(),1)
                    backTracking(z, colorR, i)
                end
            end
            
            if i in A2
    #         #     # Add arriving time lable
    #         #     annotate!(x_coor[i]+1.5, y_coor[i]-0.3, text(time_labels[i-1], :center, 4))
                if i in C
                    # Add customer demand lable
                    annotate!(x_coor[i]-0.3, y_coor[i]-0.3, text(demand_labels[i-1-np], :center, 6)) 
                end 
            end
            plot!()
        end     
    end

    result_path_svg = joinpath(save_dir, fileName * resultStatus *string(minutes)* "min-result.svg")
    result_path_png = joinpath(save_dir, fileName * resultStatus *string(minutes)* "min-result.png")
    
    savefig(result_path_svg)
    savefig(result_path_png)
end

#=================================================================================================#

println("Number of arguments: ", length(ARGS))

filePath = "../Data/Demo/Test.txt" # use in command
# filePath = "Data/Demo/C101-20.txt" # use in VSCode
case = "f"
Q1 = 600
Q2 = 100
runningTime = 10

if length(ARGS) >= 1
    if length(ARGS[1])>1 filePath = "../Data/Demo/" * ARGS[1]     end
    if length(ARGS) >= 2
        if readArgument(ARGS[2]) != 0 Q1 = readArgument(ARGS[2])  end
        if length(ARGS) >= 3
            if readArgument(ARGS[3]) != 0 Q2 = readArgument(ARGS[3])  end
            if length(ARGS) >= 4
                if readArgument(ARGS[4]) != 0 runningTime = readArgument(ARGS[4])  end
                if length(ARGS) >= 5
                    if ARGS[5] == "r"||"f" case = ARGS[5]  end
                end
            end
        end
    end
else
    println("No arguments provided")
end

println("\n", "File path: ", filePath)
println("Capacity of FEV: ",Q1)
println("Capacity of SEV: ",Q2)
println("Execution time limit: ",runningTime)
println("Case: ",case,"\n")

model = runModel(filePath, Q1,Q2,runningTime,case) 