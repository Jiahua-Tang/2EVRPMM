function solveBaseline()
    eta1 = 1
    eta2 = 1
    M = 1000000

    TT1 = arc_cost
    TT2 = arc_cost
    
    model = Model(CPLEX.Optimizer)

    active_satellites = satellites[parking_availability .== 1]
    display(active_satellites)
    
    @variable(model, x[A1, A1], Bin)
    @variable(model, z[A2, A2, satellites], Bin)
    @variable(model, t[A2]>=0)
    @variable(model, w[satellites]>=0)
    @variable(model, f[A2,A2]>=0)

    for i in A1
        @constraint(model, x[x,x] == 0)
    end
    for i in A2
        for s in satellites
            @constraint(model, x[x,x,s] == 0)
        end
    end

    @objective(model, Min, 
               sum(arc_cost[i,j] * x[i,j] for i in A1, j in A1 if i!=j))+
               sum(arc_cost[i,j] * z[i,j,s] for i in A2, j in A2, s in satellites if i != j)



end