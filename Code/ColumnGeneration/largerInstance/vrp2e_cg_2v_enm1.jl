using JuMP, CPLEX
include("../../3index/Utiles.jl")
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
    # for i in 1:num_points
    #     for j in 1:num_points
    #         print("  d[$i $j]=", round(arc_cost[i,j], digits=2))
    #     end
    #     println("")
    # end
    return arc_cost
end

function getInfo()
    # x_coor_customers, y_coor_customers, x_coor_depot, y_coor_depot, demands
    return  readFile("C:\\Users\\lenovo\\OneDrive - Ecole de Management de Normandie\\2EVRPMM\\Data\\Demo\\C101-30.txt")

end

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

    # for r in 1:length(routes_1)
    #     println("   x[$r]=",value(x[r]))
    # end
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

function solve_pricing_2e(π1,π2)
    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, y1[A2, A2]>=0, Int)
    @variable(model, u[customers]>=0,Int)
    @objective(model, Min, sum((arc_cost[i,j] - π2[i]) * y1[i,j] for i in A2 for j in A2)-sum(π1[s]*y1[s,j] for s in satellites for j in customers))

    @constraint(model, flowSatelliteOut, sum(y1[i,j] for i in satellites, j in customers) ==1 )
    @constraint(model, flowSatelliteIn, sum(y1[i,j] for i in customers, j in satellites) ==1 )
    @constraint(model, flowPoints[i in customers], sum(y1[i,j] for j in A2) == sum(y1[j,i] for j in A2))
    @constraint(model, capacityVehicle, sum(demands[i] * y1[i,j] for i in customers for j in A2) <= v_capacity)
    @constraint(model, subtourElm[i in customers, j in customers], u[i] + arc_cost[i,j] * y1[i,j] <= u[j] + MM *(1-y1[i,j]))

    optimize!(model)

    println("Subprobem 2e: Reduced cost= ", objective_value(model))
    new_route = nothing
    if objective_value(model) < 0
        # Reduced cost negative, new column generated and added to master problem
        new_route = transformRoute(y1) 
    end
    for i in A2
        for j in A2
            if round(value(y1[i,j]))==1
                println("   y1[$i, $j]=1")
            end
        end
    end
    return new_route, objective_value(model)
end

function solve_pricing_1e(π)
    println("π=",π)
    ## Construct model
    model = Model(CPLEX.Optimizer)

    set_silent(model)
    @variable(model, x1[A1,A1]>=0,Int)
    @variable(model, u[A1]>=0, Int)

    # @constraint(model, satelliteOblg, sum(x1[i,j] for i in A1 for j in subSatellites)>=1)
    @constraint(model, depart, sum(x1[1,i] for i in satellites) ==1)
    @constraint(model, ending, sum(x1[i,1] for i in satellites) ==1)
    @constraint(model, flow[i in A1], sum(x1[j,i] for j in A1)
                                        ==sum(x1[i,j] for j in A1))

    @objective(model, Min, sum(arc_cost[i,j] * x1[i,j] for i in A1 for j in A1)
                                -sum(π[s] * M * sum(x1[s,j] for j in A1) for s in satellites))

    optimize!(model)

    println("Subprobem 1e: Reduced cost= ", objective_value(model))
    for i in 1:length(A1)
        for j in 1:length(A1)
            if round(value(x1[i,j]))==1
                println("   x1[$i,$j]=",round(value(x1[i,j]),digits=2))
            end
        end
    end
end

# coor = [[0,0],  [20,20],[40,50],  [10,35],[10,40],[8,40],[8,45],[5,35]]
# demands = [0, 0,0, 5,10,8,5,12]
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
# println(customers)
rs_1 = generate_1e_paths(1,satellites)
# for r in rs_1 
#     println(r)
# end
rs_2 = [] 
for i in customers # 6+30 =36 14/23/29/
    if i<=14
        push!(rs_2, vcat(2,i,2))
    elseif i<=23
        push!(rs_2,vcat(3,i,3))
    elseif i<=29
        push!(rs_2,vcat(4,i,4))
    else
        push!(rs_2,vcat(5,i,5))
    end
end

routes_1 = Vector{Route}()
routes_2 = Vector{Route}()

for r in rs_1
    push!(routes_1, generateRoute(r))
end
for r in rs_2
    push!(routes_2, generateRoute(r))
end

# for r in routes_1
#     println(r)
# end
global num_iter = 1
# global π1 = nothing
# global π2 = nothing

global rc =[]
while true && num_iter<60
    println("\n=================Interation $num_iter=================")
    a,b_1, b_2 = updateParameters()

    ## Solve master problem and get dual variables
    pi1, pi2 = runMasterProblem(a,b_1,b_2)
#     println("pi1= ", pi1)
#     println("pi2= ", pi2)
#     global π1, π2, π3
    π1 = pi1
    π2 = pi2
    # println("Dual bound= ", sum(π2))
    global num_iter
    # if num_iter > 1
#         if π2 == pi2
#             @info "Subprobem SP_y : Reduced cost converges"
#             break
#         else
#             π2 = pi2    
#         end
#     else
#         π2 = pi2
#     end
    ## Solve Pricing problem
    new_route,rc_v = solve_pricing_2e(π1, π2)
    global rc
    push!(rc,rc_v)
    println("new route= $new_route")
    ## Check if there is new column generated
    if new_route == nothing
        @info "No new 2e routes generated, ternimating the algorithm"
        break
    else
        new_route = generateRoute(new_route)
        @info "New 2e route $new_route "
        push!(routes_2, new_route)
    end
#     # π1 = collect(dual.(satelliteSync))
#     # π2 = collect(dual.(demandCons))
#     # π3 = collect(dual.(satelliteFlow))
    # solve_pricing_1e(π1)
    num_iter += 1
end
println("")
for r in routes_2
    println(r)
end

for v in rc
    println(v)
end