function solveBaseline()
    eta1 = 1
    eta2 = 1
    M = 1000000

    TT1 = arc_cost
    TT2 = arc_cost
    
    model = Model(CPLEX.Optimizer)

    active_satellites = A1[parking_availability .== 1]
    
    @variable(model, x[A1, A1], Bin)
    @variable(model, z[A2, A2, active_satellites], Bin)
    @variable(model, t[A2]>=0)
    @variable(model, w[active_satellites]>=0)
    @variable(model, f[A2,A2]>=0)

    for i in A1
        @constraint(model, x[i,i] == 0)
    end
    for i in A2
        for s in active_satellites
            @constraint(model, z[i,i,s] == 0)
        end
    end

    @objective(model, Min, 
               sum(arc_cost[i,j] * x[i,j] for i in A1, j in A1 if i!=j))+
               sum(arc_cost[i,j] * z[i,j,s] for i in A2, j in A2, s in active_satellites if i != j)

     
    @constraint(model, [i in active_satellites], sum(x[j,i] for j in A1 if i!= j) == sum(x[i,hj] for j in A1 if i!= j))
    @constraint(model, [i in active_satellites], sum(x[i,j] for j in A1 if i != j) <= 1)

    #Flow conservation at depot
    @constraint(model, sum(x[1,j] for j in A1 if j !=1)==1)
    @constraint(model, sum(x[j,1] for j in A1 if j !=1)==1)


    optimize!(model)

    for i in A1
        for j in A1 
           if value(x[i,j])!= 0
                println("x[$i $j]=1")            
           end 
        end
    end

    for i in A2
        for j in A2
            for s in active_satellites
                if value(z[i,j,s])!= 0
                    println("z[$i $j $s]=1")            
                end                
            end
        end
    end

end