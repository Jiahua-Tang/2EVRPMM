using Coluna, Plots, Random, DataStructures, Combinatorics, Printf, 
    HiGHS, SparseArrays, Test, DataFrames, CPLEX, JuMP, BlockDecomposition

include("Utiles.jl")
include("BranchAndPrice/Utiles.jl")
include("BranchAndPrice/labelling.jl") 
include("BranchAndPrice/ngPathLabelling.jl")

"""
2E-VRP-MM (Two-Echelon Vehicle Routing Problem with Microhubs) using Coluna

This implementation transforms the custom branch-and-price algorithm into Coluna's framework.

Decomposition Strategy:
- Master Problem: 1st echelon routes (depot to satellites) and coordination constraints
- Subproblems: 2nd echelon routes for each satellite (satellite to customers to satellite)

The decomposition is performed over satellites, where each satellite has its own subproblem
to generate profitable 2nd echelon routes.
"""

# Global variables for Coluna callbacks
global current_model = nothing
global current_x_vars = nothing  
global current_customer_vars = nothing

function create_2evrp_coluna_model(optimizer)
    """
    Creates the 2E-VRP-MM model using Coluna's BlockModel with Dantzig-Wolfe decomposition.
    
    This follows the structure from your original column_generation function but adapts it for Coluna.
    The decomposition is performed over satellites for 2e route generation.
    """
    
    # Generate initial 1e routes (master problem)
    routes_1e = generateNonDominate1eRoutes(minimum_parkings_required)
    @info "Generated $(length(routes_1e)) first echelon routes"
    
    # Generate initial 2e routes for each satellite
    initial_routes_2e = generate2eInitialRoutes()
    @info "Generated $(length(initial_routes_2e)) initial second echelon routes"
    
    # Define axis over satellites for decomposition
    @axis(satellites_axis, collect(satellites))
    
    # Create BlockModel
    model = BlockModel(optimizer)
    
    # Master variables: 1st echelon routes
    @variable(model, x[1:length(routes_1e)] >= 0)
    
    # Subproblem variables: customer assignments for each satellite
    @variable(model, customer_served[s in satellites_axis, c in customers], Bin)
    
    # Master constraints - simplified to work with Coluna's structure
    
    # Customer coverage: each customer must be served exactly once across all satellites
    @constraint(model, custVisit[c in customers],
        sum(customer_served[s, c] for s in satellites) == 1
    )
    
    # Exactly one 1e route must be selected
    @constraint(model, single1eV,
        sum(x[r] for r in 1:length(routes_1e)) == 1
    )
    
    # Vehicle synchronization: ensure opened satellites have capacity
    # This links 1e routes (which open satellites) with 2e route generation
    @constraint(model, satellite_capacity[s in satellites],
        sum(customer_served[s, c] for c in customers) <= 
        length(customers) * sum(routes_1e[r].b1[s] * x[r] for r in 1:length(routes_1e))
    )
    
    # Objective: minimize 1e route costs + 2e route costs (from column generation)
    @objective(model, Min,
        sum(routes_1e[r].cost * x[r] for r in 1:length(routes_1e))
        # 2e route costs will be added through column generation
    )
    
    # Perform Dantzig-Wolfe decomposition over satellites
    @dantzig_wolfe_decomposition(model, decomposition, satellites_axis)
    
    # Configure subproblems for each satellite
    subproblems = BlockDecomposition.getsubproblems(decomposition)
    
    # Each satellite subproblem can generate multiple routes
    max_routes_per_satellite = nb_vehicle_per_satellite
    specify!.(subproblems, 
        lower_multiplicity = 0, 
        upper_multiplicity = max_routes_per_satellite,
        solver = pricing_callback_2e
    )
    
    # The customer_served variables are automatically assigned to subproblems based on the axis
    
    # Store references for pricing callback
    global current_model = model
    global current_x_vars = x  
    global current_customer_vars = customer_served
    
    return model, x, customer_served, routes_1e, initial_routes_2e
end

function pricing_callback_2e(cbdata)
    """
    Pricing callback for 2nd echelon routes using existing labeling algorithms.
    
    This callback retrieves dual prices and calls your solve_2e_labelling function
    to generate profitable routes for each satellite.
    """
    
    # Get the satellite index for this subproblem
    satellite_idx = BlockDecomposition.indice(BlockDecomposition.callback_spid(cbdata, current_model))
    
    # Retrieve reduced costs for customer assignment variables
    # This follows the pattern from your original column_generation function
    customer_red_costs = Dict()
    for c in customers
        red_cost = BlockDecomposition.callback_reduced_cost(cbdata, current_customer_vars[satellite_idx, c])
        customer_red_costs["customer_$(c)"] = red_cost
    end
    
    # Prepare dual prices in format expected by your labeling algorithm
    # These correspond to π1, π2, π3, π4 from your original implementation
    
    π1 = zeros(length(satellites) + 1)  # Vehicle sync duals
    π2 = zeros(length(points))          # Customer coverage duals
    π3 = zeros(length(satellites) + 1)  # Flow conservation duals  
    π4 = zeros(length(satellites) + 1)  # Microhub capacity duals
    
    # Map customer reduced costs to π2
    for c in customers
        if haskey(customer_red_costs, "customer_$(c)")
            π2[c] = abs(customer_red_costs["customer_$(c)"])
        end
    end
    
    # Create empty branching info (Coluna handles branching)
    branchingInfo = BranchingInfo(
        Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(), 
        Set{Tuple{Int, Int}}(), Set{Tuple{Int, Int}}(),
        Set{Int}(), Set{Int}(), Set{Int}(), Set{Int}(),
        Set{Route}(), Set{Route}(), 0
    )
    
    # Call your existing labeling algorithm
    try
        @info "Solving subproblem for satellite $satellite_idx"
        
        execution_time = @elapsed begin
            new_routes = solve_2e_labelling(π1, π2, π3, π4, satellite_idx, branchingInfo)
        end
        
        @info "Labeling took $(round(execution_time, digits=2))s, generated $(length(new_routes)) routes"
        
        if !isempty(new_routes)
            # Find the best route (most negative reduced cost)
            best_label = new_routes[1]
            min_reduced_cost = best_label.reduced_cost
            
            for label in new_routes
                if label.reduced_cost < min_reduced_cost
                    best_label = label
                    min_reduced_cost = label.reduced_cost
                end
            end
            
            if min_reduced_cost < -1e-6  # Profitable route found
                
                route_sequence = best_label.visitedNodes
                @info "Found profitable route for satellite $satellite_idx: $route_sequence (reduced cost: $(round(min_reduced_cost, digits=4)))"
                
                # Convert route to 2e Route structure
                route_2e = generate2eRoute(route_sequence)
                
                # Build solution for Coluna
                sol_vars = []
                sol_vals = []
                
                # Add customer assignment variables for this route
                customers_in_route = route_sequence[2:end-1]  # Exclude start/end satellites
                for c in customers_in_route
                    if c in customers  # Ensure it's actually a customer
                        push!(sol_vars, current_customer_vars[satellite_idx, c])
                        push!(sol_vals, 1.0)
                    end
                end
                
                # Calculate the route cost (this will be the column cost)
                route_cost = route_2e.cost
                
                # Submit solution to Coluna
                MOI.submit(current_model, 
                    BlockDecomposition.PricingSolution(cbdata), 
                    route_cost,  # Use actual route cost as column cost
                    sol_vars, 
                    sol_vals
                )
                
                # Submit dual bound (optimal since labeling is exact)
                MOI.submit(current_model, 
                    BlockDecomposition.PricingDualBound(cbdata), 
                    min_reduced_cost
                )
                
            else
                # No profitable routes
                MOI.submit(current_model, 
                    BlockDecomposition.PricingDualBound(cbdata), 
                    0.0
                )
                @info "No profitable routes found for satellite $satellite_idx (best reduced cost: $(round(min_reduced_cost, digits=4)))"
            end
        else
            # No routes generated - subproblem may be infeasible
            MOI.submit(current_model, 
                BlockDecomposition.PricingDualBound(cbdata), 
                0.0
            )
            @info "No routes generated for satellite $satellite_idx"
        end
        
    catch e
        @warn "Error in pricing callback for satellite $satellite_idx: $e"
        @warn "Stack trace:" exception=(e, catch_backtrace())
        
        # Submit a safe dual bound
        MOI.submit(current_model, 
            BlockDecomposition.PricingDualBound(cbdata), 
            0.0
        )
    end
end

function solve_2evrp_with_coluna()
    """
    Main function to solve 2E-VRP-MM using Coluna
    """
    
    # Setup Coluna optimizer
    coluna = optimizer_with_attributes(
        Coluna.Optimizer,
        "params" => Coluna.Params(
            solver = Coluna.Algorithm.TreeSearchAlgorithm(
                maxnumnodes = 10,  # Small for initial testing
                conqueralg = Coluna.ColCutGenConquer()
            )
        ),
        "default_optimizer" => CPLEX.Optimizer
    )
    
    @info "Creating 2E-VRP-MM model with Coluna..."
    
    # Create model
    model, x, customer_served, routes_1e, initial_routes_2e = create_2evrp_coluna_model(coluna)
    
    @info "Model created with $(length(routes_1e)) 1e routes and $(length(initial_routes_2e)) initial 2e routes"
    @info "Starting optimization..."
    
    # Optimize
    optimize!(model)
    
    # Display results
    status = termination_status(model)
    @info "Optimization finished with status: $status"
    
    if status == MOI.OPTIMAL || status == MOI.FEASIBLE_POINT
        try
            obj_val = objective_value(model)
            @info "Objective value: $(round(obj_val, digits=2))"
            
            # Display selected 1e routes
            println("\nSelected 1st echelon routes:")
            for r in 1:length(routes_1e)
                x_val = value(x[r])
                if x_val > 1e-6
                    route = routes_1e[r]
                    println("  x[$r] = $(round(x_val, digits=3)): Route $(route.sequence), cost = $(round(route.cost, digits=2))")
                    
                    # Show which satellites are served
                    served_satellites = [s for s in satellites if route.b1[s] == 1]
                    println("    Serves satellites: $served_satellites")
                end
            end
            
            # Display customer assignments
            println("\nCustomer assignments to satellites:")
            for s in satellites
                customers_served = []
                for c in customers
                    assign_val = value(customer_served[s, c])
                    if assign_val > 1e-6
                        push!(customers_served, c)
                    end
                end
                if !isempty(customers_served)
                    println("  Satellite $s serves customers: $customers_served")
                end
            end
            
            return obj_val
        catch e
            @warn "Error accessing solution values: $e"
            return nothing
        end
    else
        @warn "Optimization failed with status: $status"
        return nothing
    end
end

function test_coluna_vs_custom()
    """
    Compare Coluna implementation against the original custom branch-and-price
    """
    
    @info "Testing Coluna implementation vs custom branch-and-price..."
    
    # Solve with Coluna
    @info "Solving with Coluna..."
    coluna_result = solve_2evrp_with_coluna()
    
    # TODO: Compare with original custom implementation
    # This would require running the original branchAndPriceWithScore function
    
    if !isnothing(coluna_result)
        @info "Coluna solution found: $(round(coluna_result, digits=2))"
    else
        @warn "Coluna failed to find solution"
    end
    
    return coluna_result
end

# Main execution
if abspath(PROGRAM_FILE) == @__FILE__
    # Read data
    global root = "$(pwd())/TEST/"
    readData("E-n33-k4.txt", String[])
    
    # Test Coluna implementation
    test_coluna_vs_custom()
end