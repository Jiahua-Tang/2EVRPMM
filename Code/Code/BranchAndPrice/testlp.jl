# file: lp_min_2d_cplex.jl
using JuMP
using CPLEX

function main()
    model = Model(CPLEX.Optimizer)

    # (optional) CPLEX parameters
    # set_silent(model)                     # comment this out to see CPLEX logs
    # set_optimizer_attribute(model, "TimeLimit", 10.0)

    @variable(model, x >= 0)
    @variable(model, y >= 0)

    @objective(model, Min, x + y)

    con1 = @constraint(model, -x - 2y <= -6)
    con2 = @constraint(model, -3x - y <= -6)

    optimize!(model)

    term = termination_status(model)
    primal = primal_status(model)

    println("Termination status: ", term)
    println("Primal status: ", primal)

    if term == MOI.OPTIMAL
        println("Optimal objective value z* = ", objective_value(model))
        println("x* = ", value(x))
        println("y* = ", value(y))

        # Dual (shadow prices) of the constraints
        println("Dual of x + 2y ≥ 6: ", shadow_price(con1))
        println("Dual of 3x +  y ≥ 6: ", shadow_price(con2))
    else
        error("Solver did not return an optimal solution.")
    end
end

main()