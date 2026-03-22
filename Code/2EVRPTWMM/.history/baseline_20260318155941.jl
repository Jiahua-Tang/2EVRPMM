function solveBaseline()
    eta1 = 1
    eta2 = 1
    M = 1000000

    TT1 = arc_cost
    TT2 = arc_cost
    
    model = Model(CPLEX.Optimizer)

    active_satellites = A1[parking_availability .== 1]
    active_A2 = union(active_satellites, customers)
    @variable(model, x[A1, A1], Bin)
    @variable(model, z[active_A2, active_A2, active_satellites], Bin)
    @variable(model, t[active_A2]>=0)
    @variable(model, w[active_satellites]>=0)
    @variable(model, f[active_A2,active_A2,active_satellites]>=0)

    for i in A1
        @constraint(model, x[i,i] == 0)
    end
    for i in active_A2
        for s in active_satellites
            @constraint(model, z[i,i,s] == 0)
        end
    end

    @objective(model, Min, 
               sum(arc_cost[i,j] * x[i,j] for i in A1, j in A1 if i!=j))+
               sum(arc_cost[i,j] * z[i,j,s] for i in active_A2, j in active_A2, s in active_satellites if i != j)

     
    @constraint(model, [i in active_satellites], sum(x[j,i] for j in A1 if i!= j) == sum(x[i,j] for j in A1 if i!= j))
    @constraint(model, [i in active_satellites], sum(x[i,j] for j in A1 if i != j) <= 1)

    @constraint(model, sum(x[1,j] for j in A1 if j !=1)==1)
    @constraint(model, sum(x[j,1] for j in A1 if j !=1)==1)


    @constraint(model, [s in active_satellites], w[s] <= capacity_1e_vehicle * sum(x[i,s] for i in A1))

    @constraint(model, [s in active_satellites], w[s] == sum(f[s, j, s] for j in active_A2))

    @constraint(model, [i in active_A2, s in active_satellites], sum(z[j,i,s] for j in active_A2 if i!= j) == 
                                                                 sum(z[i,j,s] for j in active_A2 if i!= j))
                                                                 
    @constraint(model, [i in active_satellites, j in active_satellites, s in active_satellites], z[i,j,s]==0)
    
    # @constraint(model, [s in active_satellites], sum(z[s,j,s] for j in active_A2)<=nb_vehicle_per_satellite)

    # @constraint(model, [i in active_satellites], sum(x[i,j] for j in A1 if i != j) <= 1)

    @constraint(model, [j in customers], sum(z[i,j,s] for i in active_A2, s in active_satellites) == 1)


    # @constraint(model, )

    optimize!(model)

    for i in A1
        for j in A1 
           if value(x[i,j])!= 0
                println("x[$i $j]=1")            
           end 
        end
    end

    for i in active_A2
        for j in active_A2
            for s in active_satellites
                if value(z[i,j,s])!= 0
                    println("z[$i $j $s]=", value(z[i,j,s]))        
                end                
            end
        end
    end

    for i in active_A2
        for j in active_A2
            for s in active_satellites
                if value(f[i,j,s])!= 0
                    println("f[$i $j $s]=", value(f[i,j,s]))            
                end                
            end
        end
    end

end