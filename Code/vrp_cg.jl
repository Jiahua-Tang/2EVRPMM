using JuMP, CPLEX
import DataFrames
import HiGHS
import Plots
import SparseArrays
import Test  #src

struct Route
    length::Int
    path::Vector{Int}        
end

nb_positions = 4
# facilities_fixed_costs = [120, 150]
depot = [1]
customers = [2, 3, 4, 5, 6, 7, 8, 9]
arc_costs =
    [
        0.0 25.3 25.4 25.4 35.4 37.4 31.9 24.6 34.2;
        25.3 0.0 21.2 16.2 27.1 26.8 17.8 16.7 23.2;
        25.4 21.2 0.0 14.2 23.4 23.8 18.3 17.0 21.6;
        25.4 16.2 14.2 0.0 28.6 28.8 22.6 15.6 29.5;
        35.4 27.1 23.4 28.6 0.0 42.1 30.4 24.9 39.1;
        37.4 26.8 23.8 28.8 42.1 0.0 32.4 29.5 38.2;
        31.9 17.8 18.3 22.6 30.4 32.4 0.0 22.5 30.7;
        24.6 16.7 17.0 15.6 24.9 29.5 22.5 0.0 21.4;
        34.2 23.2 21.6 29.5 39.1 38.2 30.7 21.4 0.0;
    ]
locations = vcat(depot, customers)
nb_customers = length(customers)
# nb_facilities = length(facilities)
positions = 1:nb_positions;

# We compute the minimum number of routes needed to visit all customers:
nb_routes = Int(ceil(nb_customers / nb_positions))
# We define the upper bound `nb_routes`:
nb_routes = 1000
# routes_per_facility = 1:nb_routes_per_facility;

K =1:10

# generate initial feasible routes
routes = Vector{Route}
r = Route(length = 10, path = vcat(1,2,3,4,5,6,7,8,9))
routes.push(r)

model = Model(CPLEX.Optimizer)
set_silent(model)
@variable(model, y[p in nb_routes],Bin)
# @variable(model, x[i in customers, j in facilities], Bin)
# @variable(model, z[u in locations, v in locations], Bin)


for i in customers
    a[i] = 0
    for j in length(routes)
        if routes[j].path.contain(i)
            a[i] += 1
        end
    end
end

for i in nb_routes
    
end

@constraint(model, cov[i in customers], sum(a[i]*y[p] for j in nb_routes)==1)
@constraint(model, open_facility[],
        sum(y[p] for p in nb_routes) <= length(K))

@objective(model, Min,
    sum(arc_costs[u, v] * z[u, v] for u in locations, v in locations) +
    sum(facilities_fixed_costs[j] * y[j] for j in facilities)
)

function solve_pricing(data::Data, π::Vector{Float64})
    I = length(π)
    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, )
end

while true
    optimize!(model)
    @assert is_solved_and_feasible(model)
    ## Obtain a new dual vector
    π = dual.(cov)
    ## Solve the pricing problem
    new_route = solve_pricing(_____,π)

end