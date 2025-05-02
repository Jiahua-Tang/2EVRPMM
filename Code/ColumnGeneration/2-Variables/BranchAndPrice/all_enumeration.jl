using JuMP, CPLEX, Random, Combinatorics

include("Utiles.jl")
include("column_generation.jl")
include("branching_strategy.jl")


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
                push!(x_coor_customers, parse(Int, fields[2]))
                push!(y_coor_customers, parse(Int, fields[3]))
                push!(demands, parse(Int, fields[4]))
            end
        end
    end
    
    return x_coor_customers, y_coor_customers, x_coor_depot, y_coor_depot, demands
    
end


const nb_parking = 15
const nb_microhub = 7
data = initializeData(15)

const coor = data[1]
const parking_availability = data[4]
const demands = data[5]
const arc_cost = data[6]

# data = readFile("../Data/Demo/C101-20.txt")
# x_coor_customers, y_coor_customers = data[1], data[2]
# x_coor_depot, y_coor_depot = data[3], data[4]
# coor_depot = [[x,y] for (x,y) in zip(x_coor_depot, y_coor_depot)]
# coor_customers = [[x,y] for (x,y) in zip(x_coor_customers, y_coor_customers)]
# parking_info = generateParkingCoor(coor_customers, nb_microhub, nb_parking)
# coor_parking = parking_info[2]
# const parking_availability = parking_info[1]
# const coor = vcat(coor_depot, coor_parking, coor_customers)
# const arc_cost = calculate_arc_cost(coor, nb_parking, parking_availability)
# const demands = vcat(zeros(Int, nb_parking+1), data[5])

for (idx, p) in enumerate(parking_availability) 
    println("parking_availability[$idx] = ",Int(p))
end
println("")

# Define parameters
const nb_vehicle_per_satellite = 10
const capacity_2e_vehicle = 20
const capacity_microhub = 40
const maximum_duration_2e_vehicle = 200

# Define set
const points = 1:length(demands)
const customers = 2 + nb_parking : length(coor)
const satellites = 2:1 + nb_parking
const A2 = 2:length(coor)
const A1 = 1:1+length(satellites)

t1 = @elapsed begin
    feasible_1e_routes, feasible_2e_routes = generateAllRoutes()
end
t2 = @elapsed begin
    solveMasterProblem()
end
total_time = t1 + t2
println("Execution time: $(round(total_time, digits=4)) seconds")