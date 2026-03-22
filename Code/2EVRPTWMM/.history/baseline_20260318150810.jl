function solveBaseline()
    eta1 = 1
    eta2 = 1
    M = 1000000

    TT1 = arc_cost
    TT2 = arc_cost
    
    model = Model(CPLEX.Optimizer)

    @variable(model, x[A1, A1], Bin)
    @variable(model, z[A2, A2, satellites], Bin)
    @variable(model, t[A2]>=0)
    @variable(model, w[satellites]>=0)
    @variable(model, f[A2,A2]>=0)

    for i in A1
        @constraint(model, x[x,x] == 0)
    end
    for i in A2
        @constraint(model, x[x,x] == 0)
    end
end