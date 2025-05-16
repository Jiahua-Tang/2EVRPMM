using JuMP, CPLEX
import DataFrames
import HiGHS
import Plots
import SparseArrays
import Test  #src
using Combinatorics


function calculate_arc_cost(points::Vector{Vector{Int}})
    num_points = length(points)
    arc_cost = Array{Float64}(undef, num_points, num_points)
    for i in 1:num_points
        for j in 1:num_points
            if i == j
                arc_cost[i, j] = 10000.0  # Large value for self-loops
            else
                arc_cost[i, j] = sqrt(sum((points[i][k] - points[j][k])^2 for k in 1:length(points[i])))
            end
        end
    end
    return arc_cost
end

coor = [[0,0],  [20,20],[40,50],  [10,35],[10,40],[8,40],[8,45],[5,35]]
demands = [0, 0,0, 5,10,8,5,12]
v_capacity = 15
points = 1:length(demands)
nb_veh = 3
arc_cost = calculate_arc_cost(coor)

println(arc_cost)

# Define the range of customers
nb_customers = length(demands)
customers = 4:nb_customers
satellites = 2:3
M = sum(demands)  # capacity of a satellite
MM = 10000
struct Route
    cost::Float64
    sequence::Vector{Int}
    length::Int
    load::Int
 end
 
 function generateRoute(route::Vector{Int})
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

    # println("Route:",Route(cost,route,length(route),load) )
    return Route(cost,route,length(route),load) 
 end

# Function to calculate the total demand for a path
function calculate_load(path, demands)
    load = 0
    for node in path[2:end-1]  # Exclude the start and end warehouses
        load += demands[node]
    end
    return load
end

# Function to generate all paths with multiple customers
function generate_paths(warehouses, customers, demands, max_load)
    paths = []
    # Generate all permutations of customer subsets (excluding empty set)
    for num_customers in 1:length(customers)
        for customer_subset in combinations(customers, num_customers)
            for customer_permutation in permutations(customer_subset)
                for start_warehouse in warehouses
                    for end_warehouse in warehouses
                        path = vcat([start_warehouse], customer_permutation, [end_warehouse])
                        # Check if the load of the path exceeds the maximum allowed load
                        if calculate_load(path, demands) <= max_load
                            push!(paths, path)
                        end
                    end
                end
            end
        end
    end
    return paths
end

function generate_1e_paths(depot,satellites)
    routes = Vector{Vector{Int}}()

    # Iterate over all possible numbers of satellites to visit
    for r in 1:length(satellites)
        # Generate permutations of satellites of size r
        for perm in permutations(satellites, r)
            # Create a route starting and ending at the depot
            route = [depot; perm; depot]
            push!(routes, route)
        end
    end

    return routes
end

function getA(customers,routes_2)
    # Initialize a 2D array for a[i][r] with zeros
    a = zeros(Int, length(customers), length(routes_2))

    # Populate the array
    for (i_idx, i) in enumerate(customers)  # Loop over points (row index)
        for (r, route) in enumerate(routes_2)  # Loop over routes (column index)
            a[i_idx, r] = i in route.sequence ? 1 : 0
        end
    end
    return a
end

function getB1(satellites,routes_1)
    # Initialize a 2D array for b[s][r] with zeros
    b = zeros(Int, length(satellites))
    i = 2
    while i<=length(routes_1) 
        # println("add line")
        b = hcat(b,zeros(Int, length(satellites)))
        # println("$i: $b")
        i += 1
    end
    # Populate the array
    for (s_idx, i) in enumerate(satellites)  # Loop over points (row index)
        for (r, route) in enumerate(routes_1)  # Loop over routes (column index)
            b[s_idx, r] = i in route.sequence ? 1 : 0
        end
    end
    return b
end

function getB2(satellites,routes_2)
    # Initialize a 2D array for b[s][r] with zeros
    b = zeros(Int, length(satellites))
    i = 2
    while i <= length(routes_2) 
        # println("add line")
        # println("$i : $b")
        b = hcat(b,zeros(Int, length(satellites)))        
        i += 1 
    end
    # Populate the array
    for (s_idx, i) in enumerate(satellites)  # Loop over points (row index)
        for (r, route) in enumerate(routes_2)  # Loop over routes (column index)
            b[s_idx, r] = i in route.sequence ? 1 : 0
        end
    end
    return b
end

function runMasterProblem(a, b_1, b_2)
    
    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, x[1:length(routes_1)]>=0, Int)
    @variable(model, y[1:length(routes_2)]>=0, Int)
    
    @constraint(model, sync[s in satellites],sum(b_2[s-1,r] * y[r] for r in 1:length(routes_2)) <= M * sum(b_1[s-1,r] * x[r] for r in 1:length(routes_1)))
    @constraint(model, demandCons[i in customers], sum(a[i-1-length(satellites),r] * y[r] for r in 1:length(routes_2)) == 1 )
    # @constraint(model, totalVeh, sum(y[r] for r in 1:length(routes))<= nb_veh)
    @objective(model, Min, sum(y[r] * routes_2[r].cost for r in 1:length(routes_2))+
        sum(x[r] * routes_1[r].cost for r in 1:length(routes_1)))
    
    unset_integer.(x)
    unset_integer.(y)
    
    optimize!(model)

    @assert is_solved_and_feasible(model; dual=true)
    solution_summary(model)
    println("objective value of master problem is: ",value(objective_value(model)))

    for r in 1:length(routes_1)
        println("   x[$r]=",value(x[r]))
    end
    for r in 1:length(routes_1)
        if round(value(x[r]))==1
            println("   1e Route ", routes_1[r].sequence)
        end
    end
    tc = 0
    for r in 1:length(routes_2)
        if round(value(y[r]))==1
            println("   2e Route ", routes_2[r].sequence, "   Cost= ", routes_2[r].cost)
            tc += routes_2[r].cost
        end
    end
    println("   Total 2nd route cost = $tc")

    π1 = collect(dual.(sync))
    π2 = collect(dual.(demandCons))
    π1 = vcat(0,π1,zeros(length(customers)))
    π2 = vcat(zeros(1+length(satellites)),π2)
    # println("π1= ", π1)
    # println("π2= ", π2)
    return π1, π2
end

function updateParameters()
    ## Update parameters
    # println("a[i][r]:")
    a = getA(customers,routes_2)
    # println("   ",a)

    # println("b1[s][r]: number of 1e route: ",length(routes_1))
    b_1= getB1(satellites,routes_1)
    # println("   ",b_1)
    # println("b2[s][r]: number of 2e route: ", length(routes_2))
    b_2 = getB2(satellites,routes_2)
    # println("   ",b_2,"\n")

    return a, b_1, b_2
end

function transformRoute(x)
    new_route = Vector{Int}()
    current_node = 0

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
    return new_route
end
v_capacity = 50

x_coor_customers, y_coor_customers, x_coor_depot, y_coor_depot, demands = getInfo()
demands = vcat(0,0,0,0,0,demands)
# println(demands)
coor_x = vcat(x_coor_depot,15,15,45,45,x_coor_customers)
coor_y = vcat(y_coor_depot,15,45,45,15,y_coor_customers)
coor = [[x, y] for (x, y) in zip(coor_x, coor_y)]
# nb_veh = 3
arc_cost = calculate_arc_cost(coor)
points = 1:length(coor)
# # Define the range of customers
customers = 6:length(coor)
satellites = 2:5
A2 = 2:length(coor)
A1 = 1:1+length(satellites)
M = length(customers) # capacity of a satellite
MM = 10000

# Generate and display all paths
rs_2 = generate_paths(collect(satellites), collect(customers),demands, v_capacity)
# println(rs_2)
rs_1 = generate_1e_paths(1,satellites)
# println(rs_1)
routes_1 = Vector{Route}()
routes_2 = Vector{Route}()
for r in rs_1
    push!(routes_1, generateRoute(r))
end
for r in rs_2
    push!(routes_2, generateRoute(r))
end


a, b_1, b_2 = updateParameters()

model = Model(CPLEX.Optimizer)
set_silent(model)
@variable(model, x[1:length(routes_1)]>=0, Int)
@variable(model, y[1:length(routes_2)]>=0, Int)

@constraint(model, sync[s in satellites],sum(b_2[s-1,r] * y[r] for r in 1:length(routes_2)) <= M * sum(b_1[s-1,r] * x[r] for r in 1:length(routes_1)))
@constraint(model, demandCons[i in customers], sum(a[i-1-length(satellites),r] * y[r] for r in 1:length(routes_2)) == 1 )
# @constraint(model, totalVeh, sum(y[r] for r in 1:length(routes))<= nb_veh)
@objective(model, Min, sum(y[r] * routes_2[r].cost for r in 1:length(routes_2))+
    sum(x[r] * routes_1[r].cost for r in 1:length(routes_1)))


optimize!(model)

println("objective value is: ",value(objective_value(model)))

for r in 1:length(routes_1)
    if round(value(x[r]))==1
        println("   x[$r]=",value(x[r]))
    end
end
for r in 1:length(routes_1)
    if round(value(x[r]))==1
        println("   1e Route ", routes_1[r].sequence, "   Cost= ", round(routes_1[r].cost,digits=2))
    end
end
global tc = 0
for r in 1:length(routes_2)
    if round(value(y[r]))==1
        println("   2e Route ", routes_2[r].sequence, "   Cost= ", round(routes_2[r].cost,digits=2))
        global tc
        tc += routes_2[r].cost
    end
end
println("   Total 2nd route cost = $tc")
