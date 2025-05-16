using JuMP, CPLEX
import DataFrames
import HiGHS
import Plots
import SparseArrays
import Test  #src

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

rs_1 = [[1,2,1],[1,3,1],[1,3,2,1],[1,2,3,1]]
rs_2 = [[2,4,5,2],[3,6,3],[3,7,3],[2,8,2]] 
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
    b = zeros(Int, length(satellites), length(routes_1))

    # Populate the array
    for (i_idx, i) in enumerate(satellites)  # Loop over points (row index)
        for (r, route) in enumerate(routes_1)  # Loop over routes (column index)
            b[i_idx, r] = i in route.sequence ? 1 : 0
        end
    end
    return b
end

function getB2(satellites,routes_2)
    # Initialize a 2D array for b[s][r] with zeros
    b = zeros(Int, length(satellites), length(routes_2))

    # Populate the array
    for (i_idx, i) in enumerate(satellites)  # Loop over points (row index)
        for (r, route) in enumerate(routes_2)  # Loop over routes (column index)
            b[i_idx, r] = i in route.sequence ? 1 : 0
        end
    end
    return b
end

function runMasterProblem(routes_2, a, b_1, b_2)
    
    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, x[1:length(routes_1)]>=0, Int)
    @variable(model, y[1:length(routes_2)]>=0, Int)
    @variable(model, w[1:length(satellites)]>=0)
    
    @constraint(model, satelliteFlow[s in satellites], w[s-1] == sum( routes_2[r].load * b_2[s-1,r] * y[r] for r in 1:length(routes_2)))
    @constraint(model, satelliteSync[s in satellites], w[s-1] <= M * sum( b_1[s-1, r] * x[r] for r in 1:length(routes_1)))
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

    for r in 1:length(routes)
        if round(value(y[r]))==1
            println("   Route ", routes[r].sequence)
        end
    end

    π = collect(dual.(demandCons))
    π = vcat(0,0,0,π)
    # # println("Test dual value===== ", dual.(demandCons))
    println("π= ", π)
    return π
end

function solve_pricing(π)
    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, x[points, points]>=0, Int)
    @objective(model, Min, sum((arc_cost[i,j] - π[i]) * x[i,j] for i in points for j in points))

    @constraint(model, flowDepot, sum(x[1,j] for j in customers)==1)
    @constraint(model, [j in points], sum(x[i,j] for i in points) == sum(x[j,i] for i in points))
    # @constraint()
    @constraint(model, sum(demands[i] * x[i,j] for i in points for j in points) <= v_capacity)
    # println(model)
    optimize!(model)
    println("reduced cost= ", objective_value(model))

    if objective_value(model) < 0
        # Reduced cost negative, new column generated and added to master problem
        # println(Base.typesof(round.(Int,value.(x))))
        new_route = Vector{Int}()
        current_node = 1
        while true
            push!(new_route, current_node)
            next_node = 0
            for j in points
                if round(value(x[current_node,j]))==1
                    next_node = j
                end
            end
            if next_node == 0
                break
            end
            current_node = next_node
            if current_node == 1
                push!(new_route, current_node)
                break
            end
           
        end
        return new_route 
        
    end
    return nothing
end

global num_iter = 1

while true
    println("Interation $num_iter")

    ## Update parameters
    println("a[i][r]:")
    a = getA(customers,routes_2)
    println("   ",a)
    println("b1[s][r]:")
    b_1= getB1(satellites,routes_1)
    println("   ",b_1)
    println("b2[s][r]:")
    b_2 = getB2(satellites,routes_2)
    println("   ",b_2,"\n")

    ## Solve master problem and get dual variables
    π = runMasterProblem(routes_2,a,b_1,b_2)

    ## Solve Pricing problem
    new_route = solve_pricing(π)

    ## Check if there is new column generated
    if new_route == nothing
        @info "No new second routes generated, ternimating the algorithm"
        break
    end

    @info "New second ehcelon route added: $new_route"
    new_route = generateRoute(new_route)
    push!(routes_2, new_route) 
    
    global num_iter += 1
end


model = Model(CPLEX.Optimizer)
set_silent(model)
@variable(model, x[1:length(routes_1)]>=0, Int)
@variable(model, y[1:length(routes_2)]>=0, Int)
@variable(model, w[1:length(satellites)]>=0)

@constraint(model, satelliteFlow[s in satellites], w[s-1] == sum( routes_2[r].load * b_2[s-1,r] * y[r] for r in 1:length(routes_2)))
@constraint(model, satelliteSync[s in satellites], w[s-1] <= M * sum( b_1[s-1, r] * x[r] for r in 1:length(routes_1)))
@constraint(model, demandCons[i in customers], sum(a[i-1-length(satellites),r] * y[r] for r in 1:length(routes_2)) == 1 )
# @constraint(model, totalVeh, sum(y[r] for r in 1:length(routes))<= nb_veh)
@objective(model, Min, sum(y[r] * routes_2[r].cost for r in 1:length(routes_2))+
    sum(x[r] * routes_1[r].cost for r in 1:length(routes_1)))

unset_integer.(x)
unset_integer.(y)

optimize!(model)
for r in 1:length(routes_1)
    println("x[$r]=",value(x[r]))
end
for r in 1:length(routes_2)
    println("y[$r]=",value(y[r]))
end
# @assert is_solved_and_feasible(model; dual=true)
solution_summary(model)
println("objective value of master problem is: ",value(objective_value(model)))

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
for s in 1:length(satellites)
    println("w[$s]= ", value(w[s]))
end

π_1 = collect(dual.(demandCons))
π_1 = vcat(0,0,0, π_1)
# println("Test dual value===== ", dual.(demandCons))
println("π1= ", π_1)
# π_2 = collect(dual.(satelliteFlow))
# π_2 = vcat(0,π_2)
# # # println("Test dual value===== ", dual.(demandCons))
# println("pi2= ", π_2)
# π_3 = collect(dual.(satelliteSync))
# π_3 = vcat(0,π_3)
# # # println("Test dual value===== ", dual.(demandCons))
# println("pi3= ", π_3)

model = Model(CPLEX.Optimizer)
set_silent(model)
A2 = 2:length(coor)
@variable(model, y1[A2, A2]>=0, Int)
@variable(model, u[customers]>=0,Int)
@objective(model, Min, sum((arc_cost[i,j] - π_1[i]) * y1[i,j] for i in A2 for j in A2))

@constraint(model, flowSatelliteOut, sum(y1[i,j] for i in satellites, j in customers) ==1 )
@constraint(model, flowSatelliteIn, sum(y1[i,j] for i in customers, j in satellites) ==1 )
@constraint(model, flowPoints[i in customers], sum(y1[i,j] for j in A2) == sum(y1[j,i] for j in A2))
@constraint(model, capacityVehicle, sum(demands[i] * y1[i,j] for i in customers for j in A2) <= v_capacity)
@constraint(model, subtourElm[i in customers, j in customers], u[i] + arc_cost[i,j] * y1[i,j] <= u[j] + MM *(1-y1[i,j]))
# # println(model)
optimize!(model)
println("reduced cost= ", objective_value(model))

for i in A2
    for j in A2
        if round(value(y1[i,j]))==1 
            println("y1[$i,$j]= ", value(y1[i,j]))
        end
    end
end
println("")
for i in customers
    println("u[$i]= ", value(u[i]))
end

println("=================Iteration 2=================\n")


new_route = generateRoute([2,6,4,2])
push!(routes_2,new_route)
println("a[i][r]:")
a = getA(customers,routes_2)
println("   ",a)
println("b1[s][r]:")
b_1= getB1(satellites,routes_1)
println("   ",b_1)
println("b2[s][r]:")
b_2 = getB2(satellites,routes_2)
println("   ",b_2,"\n")

model = Model(CPLEX.Optimizer)
set_silent(model)
@variable(model, x[1:length(routes_1)]>=0, Int)
@variable(model, y[1:length(routes_2)]>=0, Int)
@variable(model, w[1:length(satellites)]>=0)

# @constraint(model, satelliteFlow[s in satellites], w[s-1] == sum( b_1[s-1, r] * x[r] for r in 1:length(routes_1)))
@constraint(model, satelliteFlow[s in satellites], w[s-1] == sum( routes_2[r].load * b_2[s-1,r] * y[r] for r in 1:length(routes_2)))
@constraint(model, satelliteSync[s in satellites], w[s-1] <= M * sum( b_1[s-1, r] * x[r] for r in 1:length(routes_1)))
@constraint(model, demandCons[i in customers], sum(a[i-1-length(satellites),r] * y[r] for r in 1:length(routes_2)) == 1 )
# @constraint(model, totalVeh, sum(y[r] for r in 1:length(routes))<= nb_veh)
@objective(model, Min, sum(y[r] * routes_2[r].cost for r in 1:length(routes_2))+
    sum(x[r] * routes_1[r].cost for r in 1:length(routes_1)))

unset_integer.(x)
unset_integer.(y)
# unset_integer.(w)

optimize!(model)


for r in 1:length(routes_1)
    println("x[$r]=",value(x[r]))
end
for r in 1:length(routes_2)
    println("y[$r]=",value(y[r]))
end
# @assert is_solved_and_feasible(model; dual=true)
solution_summary(model)
println("objective value of master problem is: ",value(objective_value(model)))

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
for s in 1:length(satellites)
    println("w[$s]= ", value(w[s]))
end

π_1 = collect(dual.(demandCons))
π_1 = vcat(0,0,0, π_1)
# println("Test dual value===== ", dual.(demandCons))
println("π1= ", π_1)


model = Model(CPLEX.Optimizer)
set_silent(model)
A2 = 2:length(coor)
@variable(model, y1[A2, A2]>=0, Int)
@variable(model, u[customers]>=0,Int)
@objective(model, Min, sum((arc_cost[i,j] - π_1[i]) * y1[i,j] for i in A2 for j in A2))

@constraint(model, flowSatelliteOut, sum(y1[i,j] for i in satellites, j in customers) ==1 )
@constraint(model, flowSatelliteIn, sum(y1[i,j] for i in customers, j in satellites) ==1 )
@constraint(model, flowPoints[i in customers], sum(y1[i,j] for j in A2) == sum(y1[j,i] for j in A2))
@constraint(model, capacityVehicle, sum(demands[i] * y1[i,j] for i in customers for j in A2) <= v_capacity)
@constraint(model, subtourElm[i in customers, j in customers], u[i] + arc_cost[i,j] * y1[i,j] <= u[j] + MM *(1-y1[i,j]))
# # println(model)
optimize!(model)
println("reduced cost= ", objective_value(model))

for i in A2
    for j in A2
        if round(value(y1[i,j]))==1 
            println("y1[$i,$j]= ", value(y1[i,j]))
        end
    end
end
println("")
for i in customers
    println("u[$i]= ", value(u[i]))
end



println("=================Iteration 3=================\n")


new_route = generateRoute([2,7,5,2])
push!(routes_2,new_route)
println("a[i][r]:")
a = getA(customers,routes_2)
println("   ",a)
println("b1[s][r]:")
b_1= getB1(satellites,routes_1)
println("   ",b_1)
println("b2[s][r]:")
b_2 = getB2(satellites,routes_2)
println("   ",b_2,"\n")

model = Model(CPLEX.Optimizer)
set_silent(model)
@variable(model, x[1:length(routes_1)]>=0, Int)
@variable(model, y[1:length(routes_2)]>=0, Int)
@variable(model, w[1:length(satellites)]>=0)

# @constraint(model, satelliteFlow[s in satellites], w[s-1] == sum( b_1[s-1, r] * x[r] for r in 1:length(routes_1)))
@constraint(model, satelliteFlow[s in satellites], w[s-1] == sum( routes_2[r].load * b_2[s-1,r] * y[r] for r in 1:length(routes_2)))
@constraint(model, satelliteSync[s in satellites], w[s-1] <= M * sum( b_1[s-1, r] * x[r] for r in 1:length(routes_1)))
@constraint(model, demandCons[i in customers], sum(a[i-1-length(satellites),r] * y[r] for r in 1:length(routes_2)) == 1 )
# @constraint(model, totalVeh, sum(y[r] for r in 1:length(routes))<= nb_veh)
@objective(model, Min, sum(y[r] * routes_2[r].cost for r in 1:length(routes_2))+
    sum(x[r] * routes_1[r].cost for r in 1:length(routes_1)))

unset_integer.(x)
unset_integer.(y)
# unset_integer.(w)

optimize!(model)


for r in 1:length(routes_1)
    println("x[$r]=",value(x[r]))
end
for r in 1:length(routes_2)
    println("y[$r]=",value(y[r]))
end
# @assert is_solved_and_feasible(model; dual=true)
solution_summary(model)
println("objective value of master problem is: ",value(objective_value(model)))

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
for s in 1:length(satellites)
    println("w[$s]= ", value(w[s]))
end

π_1 = collect(dual.(demandCons))
π_1 = vcat(0,0,0, π_1)
# println("Test dual value===== ", dual.(demandCons))
println("π1= ", π_1)


model = Model(CPLEX.Optimizer)
set_silent(model)
A2 = 2:length(coor)
@variable(model, y1[A2, A2]>=0, Int)
@variable(model, u[customers]>=0,Int)
@objective(model, Min, sum((arc_cost[i,j] - π_1[i]) * y1[i,j] for i in A2 for j in A2))

@constraint(model, flowSatelliteOut, sum(y1[i,j] for i in satellites, j in customers) ==1 )
@constraint(model, flowSatelliteIn, sum(y1[i,j] for i in customers, j in satellites) ==1 )
@constraint(model, flowPoints[i in customers], sum(y1[i,j] for j in A2) == sum(y1[j,i] for j in A2))
@constraint(model, capacityVehicle, sum(demands[i] * y1[i,j] for i in customers for j in A2) <= v_capacity)
@constraint(model, subtourElm[i in customers, j in customers], u[i] + arc_cost[i,j] * y1[i,j] <= u[j] + MM *(1-y1[i,j]))
# # println(model)
optimize!(model)
println("reduced cost= ", objective_value(model))

for i in A2
    for j in A2
        if round(value(y1[i,j]))==1 
            println("y1[$i,$j]= ", value(y1[i,j]))
        end
    end
end
println("")
for i in customers
    println("u[$i]= ", value(u[i]))
end



println("=================Iteration 4=================\n")


new_route = generateRoute([2,6,7,2])
push!(routes_2,new_route)
println("a[i][r]:")
a = getA(customers,routes_2)
println("   ",a)
println("b1[s][r]:")
b_1= getB1(satellites,routes_1)
println("   ",b_1)
println("b2[s][r]:")
b_2 = getB2(satellites,routes_2)
println("   ",b_2,"\n")

model = Model(CPLEX.Optimizer)
set_silent(model)
@variable(model, x[1:length(routes_1)]>=0, Int)
@variable(model, y[1:length(routes_2)]>=0, Int)
@variable(model, w[1:length(satellites)]>=0)

# @constraint(model, satelliteFlow[s in satellites], w[s-1] == sum( b_1[s-1, r] * x[r] for r in 1:length(routes_1)))
@constraint(model, satelliteFlow[s in satellites], w[s-1] == sum( routes_2[r].load * b_2[s-1,r] * y[r] for r in 1:length(routes_2)))
@constraint(model, satelliteSync[s in satellites], w[s-1] <= M * sum( b_1[s-1, r] * x[r] for r in 1:length(routes_1)))
@constraint(model, demandCons[i in customers], sum(a[i-1-length(satellites),r] * y[r] for r in 1:length(routes_2)) == 1 )
# @constraint(model, totalVeh, sum(y[r] for r in 1:length(routes))<= nb_veh)
@objective(model, Min, sum(y[r] * routes_2[r].cost for r in 1:length(routes_2))+
    sum(x[r] * routes_1[r].cost for r in 1:length(routes_1)))

unset_integer.(x)
unset_integer.(y)
# unset_integer.(w)

optimize!(model)


for r in 1:length(routes_1)
    println("x[$r]=",value(x[r]))
end
for r in 1:length(routes_2)
    println("y[$r]=",value(y[r]))
end
# @assert is_solved_and_feasible(model; dual=true)
solution_summary(model)
println("objective value of master problem is: ",value(objective_value(model)))

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
for s in 1:length(satellites)
    println("w[$s]= ", value(w[s]))
end

π_1 = collect(dual.(demandCons))
π_1 = vcat(0,0,0, π_1)
# println("Test dual value===== ", dual.(demandCons))
println("π1= ", π_1)


model = Model(CPLEX.Optimizer)
set_silent(model)
A2 = 2:length(coor)
@variable(model, y1[A2, A2]>=0, Int)
@variable(model, u[customers]>=0,Int)
@objective(model, Min, sum((arc_cost[i,j] - π_1[i]) * y1[i,j] for i in A2 for j in A2))

@constraint(model, flowSatelliteOut, sum(y1[i,j] for i in satellites, j in customers) ==1 )
@constraint(model, flowSatelliteIn, sum(y1[i,j] for i in customers, j in satellites) ==1 )
@constraint(model, flowPoints[i in customers], sum(y1[i,j] for j in A2) == sum(y1[j,i] for j in A2))
@constraint(model, capacityVehicle, sum(demands[i] * y1[i,j] for i in customers for j in A2) <= v_capacity)
@constraint(model, subtourElm[i in customers, j in customers], u[i] + arc_cost[i,j] * y1[i,j] <= u[j] + MM *(1-y1[i,j]))
# # println(model)
optimize!(model)
println("reduced cost= ", objective_value(model))

for i in A2
    for j in A2
        if round(value(y1[i,j]))==1 
            println("y1[$i,$j]= ", value(y1[i,j]))
        end
    end
end
println("")
for i in customers
    println("u[$i]= ", value(u[i]))
end




println("=================Iteration 5=================\n")


new_route = generateRoute([2,5,2])
push!(routes_2,new_route)
println("a[i][r]:")
a = getA(customers,routes_2)
println("   ",a)
println("b1[s][r]:")
b_1= getB1(satellites,routes_1)
println("   ",b_1)
println("b2[s][r]:")
b_2 = getB2(satellites,routes_2)
println("   ",b_2,"\n")

model = Model(CPLEX.Optimizer)
set_silent(model)
@variable(model, x[1:length(routes_1)]>=0, Int)
@variable(model, y[1:length(routes_2)]>=0, Int)
@variable(model, w[1:length(satellites)]>=0)

# @constraint(model, satelliteFlow[s in satellites], w[s-1] == sum( b_1[s-1, r] * x[r] for r in 1:length(routes_1)))
@constraint(model, satelliteFlow[s in satellites], w[s-1] == sum( routes_2[r].load * b_2[s-1,r] * y[r] for r in 1:length(routes_2)))
@constraint(model, satelliteSync[s in satellites], w[s-1] <= M * sum( b_1[s-1, r] * x[r] for r in 1:length(routes_1)))
@constraint(model, demandCons[i in customers], sum(a[i-1-length(satellites),r] * y[r] for r in 1:length(routes_2)) == 1 )
# @constraint(model, totalVeh, sum(y[r] for r in 1:length(routes))<= nb_veh)
@objective(model, Min, sum(y[r] * routes_2[r].cost for r in 1:length(routes_2))+
    sum(x[r] * routes_1[r].cost for r in 1:length(routes_1)))

unset_integer.(x)
unset_integer.(y)
# unset_integer.(w)

optimize!(model)


for r in 1:length(routes_1)
    println("x[$r]=",value(x[r]))
end
for r in 1:length(routes_2)
    println("y[$r]=",value(y[r]))
end
# @assert is_solved_and_feasible(model; dual=true)
solution_summary(model)
println("objective value of master problem is: ",value(objective_value(model)))

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
for s in 1:length(satellites)
    println("w[$s]= ", value(w[s]))
end

π_1 = collect(dual.(demandCons))
π_1 = vcat(0,0,0, π_1)
# println("Test dual value===== ", dual.(demandCons))
println("π1= ", π_1)


model = Model(CPLEX.Optimizer)
set_silent(model)
A2 = 2:length(coor)
@variable(model, y1[A2, A2]>=0, Int)
@variable(model, u[customers]>=0,Int)
@objective(model, Min, sum((arc_cost[i,j] - π_1[i]) * y1[i,j] for i in A2 for j in A2))

@constraint(model, flowSatelliteOut, sum(y1[i,j] for i in satellites, j in customers) ==1 )
@constraint(model, flowSatelliteIn, sum(y1[i,j] for i in customers, j in satellites) ==1 )
@constraint(model, flowPoints[i in customers], sum(y1[i,j] for j in A2) == sum(y1[j,i] for j in A2))
@constraint(model, capacityVehicle, sum(demands[i] * y1[i,j] for i in customers for j in A2) <= v_capacity)
@constraint(model, subtourElm[i in customers, j in customers], u[i] + arc_cost[i,j] * y1[i,j] <= u[j] + MM *(1-y1[i,j]))
# # println(model)
optimize!(model)
println("reduced cost= ", objective_value(model))

for i in A2
    for j in A2
        if round(value(y1[i,j]))==1 
            println("y1[$i,$j]= ", value(y1[i,j]))
        end
    end
end
println("")
for i in customers
    println("u[$i]= ", value(u[i]))
end



println("=================Iteration 6=================\n")


new_route = generateRoute([2,4,7,2])
push!(routes_2,new_route)
println("a[i][r]:")
a = getA(customers,routes_2)
println("   ",a)
println("b1[s][r]:")
b_1= getB1(satellites,routes_1)
println("   ",b_1)
println("b2[s][r]:")
b_2 = getB2(satellites,routes_2)
println("   ",b_2,"\n")

model = Model(CPLEX.Optimizer)
set_silent(model)
@variable(model, x[1:length(routes_1)]>=0, Int)
@variable(model, y[1:length(routes_2)]>=0, Int)
@variable(model, w[1:length(satellites)]>=0)

# @constraint(model, satelliteFlow[s in satellites], w[s-1] == sum( b_1[s-1, r] * x[r] for r in 1:length(routes_1)))
@constraint(model, satelliteFlow[s in satellites], w[s-1] == sum( routes_2[r].load * b_2[s-1,r] * y[r] for r in 1:length(routes_2)))
@constraint(model, satelliteSync[s in satellites], w[s-1] <= M * sum( b_1[s-1, r] * x[r] for r in 1:length(routes_1)))
@constraint(model, demandCons[i in customers], sum(a[i-1-length(satellites),r] * y[r] for r in 1:length(routes_2)) == 1 )
# @constraint(model, totalVeh, sum(y[r] for r in 1:length(routes))<= nb_veh)
@objective(model, Min, sum(y[r] * routes_2[r].cost for r in 1:length(routes_2))+
    sum(x[r] * routes_1[r].cost for r in 1:length(routes_1)))

unset_integer.(x)
unset_integer.(y)
# unset_integer.(w)

optimize!(model)


for r in 1:length(routes_1)
    println("x[$r]=",value(x[r]))
end
for r in 1:length(routes_2)
    println("y[$r]=",value(y[r]))
end
# @assert is_solved_and_feasible(model; dual=true)
solution_summary(model)
println("objective value of master problem is: ",value(objective_value(model)))

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
for s in 1:length(satellites)
    println("w[$s]= ", value(w[s]))
end

π_1 = collect(dual.(demandCons))
π_1 = vcat(0,0,0, π_1)
# # # println("Test dual value===== ", dual.(demandCons))
println("π1= ", π_1)


model = Model(CPLEX.Optimizer)
set_silent(model)
A2 = 2:length(coor)
@variable(model, y1[A2, A2]>=0, Int)
@variable(model, u[customers]>=0,Int)
@objective(model, Min, sum((arc_cost[i,j] - π_1[i]) * y1[i,j] for i in A2 for j in A2))

@constraint(model, flowSatelliteOut, sum(y1[i,j] for i in satellites, j in customers) ==1 )
@constraint(model, flowSatelliteIn, sum(y1[i,j] for i in customers, j in satellites) ==1 )
@constraint(model, flowPoints[i in customers], sum(y1[i,j] for j in A2) == sum(y1[j,i] for j in A2))
@constraint(model, capacityVehicle, sum(demands[i] * y1[i,j] for i in customers for j in A2) <= v_capacity)
@constraint(model, subtourElm[i in customers, j in customers], u[i] + arc_cost[i,j] * y1[i,j] <= u[j] + MM *(1-y1[i,j]))
# # println(model)
optimize!(model)
println("reduced cost= ", objective_value(model))

for i in A2
    for j in A2
        if round(value(y1[i,j]))==1 
            println("y1[$i,$j]= ", value(y1[i,j]))
        end
    end
end
println("")
for i in customers
    println("u[$i]= ", value(u[i]))
end


println("=================Iteration 7=================\n")


new_route = generateRoute([2,5,4,2])
push!(routes_2,new_route)
println("a[i][r]:")
a = getA(customers,routes_2)
println("   ",a)
println("b1[s][r]:")
b_1= getB1(satellites,routes_1)
println("   ",b_1)
println("b2[s][r]:")
b_2 = getB2(satellites,routes_2)
println("   ",b_2,"\n")

model = Model(CPLEX.Optimizer)
set_silent(model)
@variable(model, x[1:length(routes_1)]>=0, Int)
@variable(model, y[1:length(routes_2)]>=0, Int)
@variable(model, w[1:length(satellites)]>=0)

# @constraint(model, satelliteFlow[s in satellites], w[s-1] == sum( b_1[s-1, r] * x[r] for r in 1:length(routes_1)))
@constraint(model, satelliteFlow[s in satellites], w[s-1] == sum( routes_2[r].load * b_2[s-1,r] * y[r] for r in 1:length(routes_2)))
@constraint(model, satelliteSync[s in satellites], w[s-1] <= M * sum( b_1[s-1, r] * x[r] for r in 1:length(routes_1)))
@constraint(model, demandCons[i in customers], sum(a[i-1-length(satellites),r] * y[r] for r in 1:length(routes_2)) == 1 )
# @constraint(model, totalVeh, sum(y[r] for r in 1:length(routes))<= nb_veh)
@objective(model, Min, sum(y[r] * routes_2[r].cost for r in 1:length(routes_2))+
    sum(x[r] * routes_1[r].cost for r in 1:length(routes_1)))

unset_integer.(x)
unset_integer.(y)
# unset_integer.(w)

optimize!(model)


for r in 1:length(routes_1)
    println("x[$r]=",value(x[r]))
end
for r in 1:length(routes_2)
    println("y[$r]=",value(y[r]))
end
# @assert is_solved_and_feasible(model; dual=true)
solution_summary(model)
println("objective value of master problem is: ",value(objective_value(model)))

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
for s in 1:length(satellites)
    println("w[$s]= ", value(w[s]))
end

π_1 = collect(dual.(demandCons))
π_1 = vcat(0,0,0, π_1)
# println("Test dual value===== ", dual.(demandCons))
println("π1= ", π_1)


model = Model(CPLEX.Optimizer)
set_silent(model)
A2 = 2:length(coor)
@variable(model, y1[A2, A2]>=0, Int)
@variable(model, u[customers]>=0,Int)
@objective(model, Min, sum((arc_cost[i,j] - π_1[i]) * y1[i,j] for i in A2 for j in A2))

@constraint(model, flowSatelliteOut, sum(y1[i,j] for i in satellites, j in customers) ==1 )
@constraint(model, flowSatelliteIn, sum(y1[i,j] for i in customers, j in satellites) ==1 )
@constraint(model, flowPoints[i in customers], sum(y1[i,j] for j in A2) == sum(y1[j,i] for j in A2))
@constraint(model, capacityVehicle, sum(demands[i] * y1[i,j] for i in customers for j in A2) <= v_capacity)
@constraint(model, subtourElm[i in customers, j in customers], u[i] + arc_cost[i,j] * y1[i,j] <= u[j] + MM *(1-y1[i,j]))
# # println(model)
optimize!(model)
println("reduced cost= ", objective_value(model))

for i in A2
    for j in A2
        if round(value(y1[i,j]))==1 
            println("y1[$i,$j]= ", value(y1[i,j]))
        end
    end
end
println("")
for i in customers
    println("u[$i]= ", value(u[i]))
end









# ======================================================================

# new_route = Vector{Int}()
# current_node = 1
# while true
#     push!(new_route, current_node)
#     next_node = 0
#     for j in points
#         if round(value(x[current_node,j]))==1
#             next_node = j
#         end
#     end
#     if next_node == 0
#         break
#     end
#     current_node = next_node
#     if current_node == 1
#         push!(new_route, current_node)
#         break
#     end 
# end    


# while true
#     println("Iteration $num_iter")

#     println("a[i][r]:")
#     a = getA(customers,routes)
#     println(a,"\n")

#     # Solve the master problem and get the dual variable
#     π = runMasterProblem(routes,a)
#     # Solve the pricing problem
#     new_route = solve_pricing_2(π)
#     # Check if there is new column generated
#     if new_route == nothing
#         @info "No new routes, terminating the algorithm"
#         break
#     end

#     @info "New route added: $new_route"
#     new_route = generateRoute(new_route)
#     push!(routes, new_route)

#     global num_iter = num_iter + 1
# end
