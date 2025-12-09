facilities = [1,2] 
customers = [3,4,5,6,7,8,9]
demands = [6, 10, 4, 13, 5, 19]

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

nb_customers = length(customers)
nb_facilities = length(facilities)
capacity = 20

using BlockDecomposition, Coluna, CPLEX, JuMP

nb_routes = Int(ceil(sum(demands)/capacity))
nb_routes_per_facilities = 2
locations = vcat(facilities, customers)

coluna = optimizer_with_attributes(
    Coluna.Optimizer,
    "params" => Coluna.Params(
        solver=Coluna.Algorithm.TreeSearchAlgorithm( ## default branch-and-bound of Coluna
            maxnumnodes=100,
            conqueralg=Coluna.ColCutGenConquer() ## default column and cut generation of Coluna
        ) ## default branch-cut-and-price
    ),
    "default_optimizer" => CPLEX.Optimizer # GLPK for the master & the subproblems
);

mutable struct Route
    a::Vector{Int}
    path::Vector{Int}
    cost::Float64
end

routes_pool = Vector{Route}()
for i in facilities
    for j in customers 
        a = zeros(Int, nb_facilities+nb_customers)
        a[j] = 1
        push!(routes_pool, Route(a, [i,j,i], arc_costs[i,j]*2))
    end
end
nb_routes = length(routes_pool)

function create_model(optimizer, pricing_algorithms)

    model = BlockModel(optimizer)

    @axis(routes_axis, 1:nb_routes)

    @variable(model, x[r in 1:length(routes_pool)], Bin)

    @constraint(model, cov[i in customers], sum(x[r] * route.a[i] for (r,route) in enumerate(routes_pool))>=1)

    @objective(model, Min, sum(route.cost * x[r] for (r, route) in enumerate(routes_pool)))

    @dantzig_wolfe_decomposition(model, dec, routes_axis)

    subproblems = BlockDecomposition.getsubproblems(dec)

    specify!.(subproblems, lower_multiplicity=0, upper_multiplicity=50, solver = pricing_algorithms)

    return model, x
end

coluna = optimizer_with_attributes(
    Coluna.Optimizer,
    "params" => Coluna.Params(
        solver=Coluna.Algorithm.TreeSearchAlgorithm( ## default branch-and-bound of Coluna
            maxnumnodes=100,
            conqueralg=Coluna.ColCutGenConquer() ## default column and cut generation of Coluna
        ) ## default branch-cut-and-price
    ),
    "default_optimizer" => CPLEX.Optimizer # GLPK for the master & the subproblems
);


function pricing_callback(cbdata)
    ## Get the id of the facility.
    @info "Enter pricing algo"
    curr_route = BlockDecomposition.indice(BlockDecomposition.callback_spid(cbdata, model))
    @info "curr_route = $curr_route, routes_pool length = $(length(routes_pool))"
    
    # red_cost = [BlockDecomposition.callback_reduced_cost(cbdata, x[curr_route])]
    display(cbdata)
    
    # # Check bounds before accessing
    # if curr_route <= length(routes_pool)
    #     @info routes_pool[curr_route].path
    # else
    #     @info "curr_route $curr_route is out of bounds for routes_pool (length $(length(routes_pool)))"
    #     # Submit a safe dual bound indicating no profitable routes
    #     Coluna.MathOptInterface.submit(backend(model), BlockDecomposition.PricingDualBound(cbdata), 0.0)
    #     return
    # end

    ## For now, just submit a basic solution without accessing problematic master variables
    ## In a real implementation, this would solve a pricing subproblem and generate new routes
    
    # Submit a dummy solution indicating no profitable routes found
    # sol_vars = []  # No new variables to add
    # sol_vals = []  # No values
    # sol_cost = 0.0  # No improvement
    
    # Submit the solution to Coluna
    # Coluna.MathOptInterface.submit(backend(model), BlockDecomposition.PricingSolution(cbdata), sol_cost, sol_vars, sol_vals)
    
    # Submit the dual bound
    # Coluna.MathOptInterface.submit(backend(model), BlockDecomposition.PricingDualBound(cbdata), sol_cost)
    # x_red_costs = Dict(
    #     "x_$(i)_$(j)" => BlockDecomposition.callback_reduced_cost(cbdata, x[i, j]) for i in customers
    # )
    # # @info "display reduced cost"
    # # display(z_red_costs)

    # ## Keep route with minimum reduced cost.
    # red_costs_j = map(r -> (
    #         r,
    #         x_contribution(r, j, x_red_costs) + z_contribution(r, z_red_costs) # the reduced cost of a route is the sum of the contribution of the variables
    #     ), routes_per_facility[j]
    # )
    # min_index = argmin([x for (_, x) in red_costs_j])
    # (best_route, min_reduced_cost) = red_costs_j[min_index]

    # ## Retrieve the route's arcs.
    # best_route_arcs = Vector{Tuple{Int,Int}}()
    # for i in 1:(best_route.length-1)
    #     push!(best_route_arcs, (best_route.path[i], best_route.path[i+1]))
    # end
    # best_route_customers = best_route.path[2:best_route.length]

    # ## Create the solution (send only variables with non-zero values).
    # z_vars = [z[u, v] for (u, v) in best_route_arcs]
    # # @info "Display z_vars"
    # # display(z_vars)
    # x_vars = [x[i, j] for i in best_route_customers]
    # sol_vars = vcat(z_vars, x_vars)
    # sol_vals = ones(Float64, length(z_vars) + length(x_vars))
    # sol_cost = min_reduced_cost

    # ## Submit the solution to the subproblem to Coluna.
    # MOI.submit(model, BlockDecomposition.PricingSolution(cbdata), sol_cost, sol_vars, sol_vals)

    # ## Submit the dual bound to the solution of the subproblem.
    # ## This bound is used to compute the contribution of the subproblem to the lagrangian
    # ## bound in column generation.
    # MOI.submit(model, BlockDecomposition.PricingDualBound(cbdata), sol_cost) ## optimal solution

end

# Create the model:
model, x= create_model(coluna, pricing_callback);

# Solve:
JuMP.optimize!(model)