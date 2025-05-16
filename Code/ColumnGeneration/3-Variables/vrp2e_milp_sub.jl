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

coor = [[20,10],  [25,0],[25,30],  [25,5],[5,10],[25,15],[5,20],[25,25]]
demands = [0, 0,0, 5,10,8,5,12]
v_capacity = 15
points = 1:length(demands)
nb_vehicle_per_satellite = 3
arc_cost = calculate_arc_cost(coor)

println(arc_cost)

# Define the range of customers
nb_customers = length(demands)
customers = 4:nb_customers
satellites = 2:3
M = 2 * length(customers)  # capacity of a satellite
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


# Generate and display all paths
rs_2 = generate_paths(collect(satellites), collect(customers),demands, v_capacity)
println(rs_2)

rs_1 = [[1,2,1],[1,3,1],[1,3,2,1],[1,2,3,1]]
routes_1 = Vector{Route}()
routes_2 = Vector{Route}()

for r in rs_1
    push!(routes_1, generateRoute(r))
end
for r in rs_2
    push!(routes_2, generateRoute(r))
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
    b = vcat(0 , b)
    i = 2
    while i<=length(routes_1) 
        # println("add line")
        b = hcat(b,zeros(Int, length(satellites)+1))
        i += 1
    end
    # Populate the array
    for (s_idx, i) in enumerate(satellites)  # Loop over points (row index)
        for (r, route) in enumerate(routes_1)  # Loop over routes (column index)
            b[s_idx+1, r] = i in route.sequence ? 1 : 0
        end
    end
    return b
end

function getB2Out(satellites,routes_2)
    # Initialize a 2D array for b[s][r] with zeros
    b = zeros(Int, length(satellites))
    b = vcat(0 , b)
    i = 2
    while i <= length(routes_2) 
        # println("add line")
        # println("$i : $b")
        b = hcat(b,zeros(Int, length(satellites)+1))        
        i += 1 
    end
    # Populate the array
    for (s_idx, i) in enumerate(satellites)  # Loop over points (row index)
        for (r, route) in enumerate(routes_2)  # Loop over routes (column index)
            b[s_idx + 1, r] = route.sequence[1] == i ? 1 : 0
        end
    end
    return b
end

function getB2(satellites,routes_2)
    # Initialize a 2D array for b[s][r] with zeros
    b = zeros(Int, length(satellites))
    b = vcat(0 , b)
    i = 2
    while i <= length(routes_2) 
        # println("add line")
        # println("$i : $b")
        b = hcat(b,zeros(Int, length(satellites)+1))        
        i += 1 
    end
    # Populate the array
    for (s_idx, i) in enumerate(satellites)  # Loop over points (row index)
        for (r, route) in enumerate(routes_2)  # Loop over routes (column index)
            b[s_idx + 1, r] = count(==(i), route.sequence)
        end
    end
    return b
end

println("a[i][r]:")
a = getA(customers,routes_2)
println("   ",a)
println("b1[s][r]:")
b_1= getB1(satellites,routes_1)
println("   ",b_1)
println("b2[s][r]:")
b_2 = getB2(satellites,routes_2)
println("   ",b_2,"\n")
println("b2_out[s][r]:")
b_2_out = getB2Out(satellites, routes_2)
println("   ",b_2_out,"\n")

model = Model(CPLEX.Optimizer)
set_silent(model)
@variable(model, x[1:length(routes_1)]>=0, Int)
@variable(model, y[1:length(routes_2)]>=0, Int)
@variable(model, z[satellites]>=0)
@variable(model, k[s in satellites], Int)  # Auxiliary integer variable


@constraint(model, sync1e[s in satellites], z[s] - sum(b_1[s,r] * x[r] for r in 1:length(routes_1))<=0)
@constraint(model, sync2e[s in satellites], sum(b_2[s,r] * y[r] for r in 1:length(routes_2)) - M * z[s]<=0)
@constraint(model, custVisit[i in customers], 1 - sum(a[i-1-length(satellites),r] * y[r] for r in 1:length(routes_2)) <= 0 )
@constraint(model, number2evfixe[s in satellites], sum(b_2[s, r] * y[r] for r in 1:length(routes_2)) == 2 * k[s])
@constraint(model, maxNb2e[s in satellites], sum(b_2_out[s,r] * y[r] for r in 1:length(routes_2)) - nb_vehicle_per_satellite <= 0)

@objective(model, Min, sum(y[r] * routes_2[r].cost for r in 1:length(routes_2))+
    sum(x[r] * routes_1[r].cost for r in 1:length(routes_1)))

# unset_integer.(x)
# unset_integer.(y)
# unset_integer.(z )

optimize!(model)


solution_summary(model)
println("objective value of master problem is: ",value(objective_value(model)))

for i in points
    for j in points
        print("d[$i $j]=",round(arc_cost[i,j],digits=2),"   ")
    end
    println("")
end

for r in 1:length(routes_1)
    if round(value(x[r]))==1
        println("1st echelon route: ", routes_1[r].sequence, " Cost: ", round(routes_1[r].cost, digits=2))
    end
end
println("")
for r in 1:length(routes_2)
    if round(value(y[r]))==1
        println("2nd echelon route: ", routes_2[r].sequence, " Cost: ", round(routes_2[r].cost, digits=2))
    end
end
for s in satellites
    println("z[$s]= ", value(z[s]))
end