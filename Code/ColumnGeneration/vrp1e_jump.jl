using JuMP, CPLEX
import DataFrames
import HiGHS
import Plots
import SparseArrays
import Test  #src

arc_cost = [
    10000 10 15 20;
    10 10000 35 25;
    15 35 10000 30;
    20 25 30 10000
]
# 1 -> 4 -> 2 -> 1 : 20+25+10 = 55
# 1 -> 3 -> 1 : 15+15 = 30

struct Route
    cost::Float64
    sequence::Vector{Int}
    length::Int
 end
 
 function generateRoute(route::Vector{Int})
    cost = 0
    for i in 1:(length(route) - 1)
        from_node = route[i]
        to_node = route[i + 1]
        cost+= arc_cost[from_node, to_node]
    end
    return Route(cost,  route, length(route)) 
 end

rs = [[1,2,1],[1,3,1],[1,4,1]]#,[1,4,2,1]]

routes = Vector{Route}()
for r in rs
    push!(routes, generateRoute(r))
end

demands = [0,5,10,8]
v_capacity = 15
points = 1:length(demands)
nb_veh = 3

# Define the range of customers
nb_customers = length(demands)
customers = 2:nb_customers


function getA(customers,routes)
    # Initialize a 2D array for a[i][r] with zeros
    a = zeros(Int, length(customers), length(routes))

    # Populate the array
    for (i_idx, i) in enumerate(customers)  # Loop over points (row index)
        for (r, route) in enumerate(routes)  # Loop over routes (column index)
            a[i_idx, r] = i in route.sequence ? 1 : 0
        end
    end
    return a
end

function runMasterProblem(routes, a)
    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, y[1:length(routes)]>=0,Int)
    @constraint(model, demandCons[i in customers], sum(a[i-1,r] * y[r] for r in 1:length(routes)) == 1 )
    # @constraint(model, totalVeh, sum(y[r] for r in 1:length(routes))<= nb_veh)
    @objective(model, Min, sum(y[r] * routes[r].cost for r in 1:length(routes)))

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
    π = vcat(0,π)
    # println("Test dual value===== ", dual.(demandCons))
    println("pi= ", π)
    return π
end


function solve_pricing_2(π)
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
    println("Iteration $num_iter")

    println("a[i][r]:")
    a = getA(customers,routes)
    println(a,"\n")

    # Solve the master problem and get the dual variable
    π = runMasterProblem(routes,a)
    # Solve the pricing problem
    new_route = solve_pricing_2(π)
    # Check if there is new column generated
    if new_route == nothing
        @info "No new routes, terminating the algorithm"
        break
    end

    @info "New route added: $new_route"
    new_route = generateRoute(new_route)
    push!(routes, new_route)

    global num_iter = num_iter + 1
end
