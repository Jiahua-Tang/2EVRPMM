using JuMP, CPLEX
model = Model(CPLEX.Optimizer)
set_attribute(model, "CPX_PARAM_EPINT", 1e-8)
set_optimizer_attribute(model, "CPX_PARAM_MIPSEARCH", 1)

function totalDuration(z, x, d)
    for k in 2:1+np+nc
        if round(value(z[x,k])) == 1
            d = d + distances[x,k]
            if k in 2:1+np   return d  end
            totalDuration(z, k)           
        end
    end
    return d
end

function buildModel()
    P = 2 : np+1 #Set of parking place
    C = np+2 : np+nc+1 #Set of customers
    A1 = 1 : 1+np #Set of FE arcs
    A2 = 2 : 1+np+nc #Set of SE arcs
    N = 1:np+nc+1 #Set of nodes
    M = 100000
    
    TT1 = distances
    TT2 = distances

    model=Model(CPLEX.Optimizer)

    # Decision variable
    @variable(model, x[A1,A1], Bin)#Arc(x,y) traversed by FEV
    @variable(model, y[A1,A1], Bin)#Arc(x,y) traversed by MM
    for i in 1:np+1
        @constraint(model, x[i, i] == 0)
        @constraint(model, y[i, i] == 0)
    end
    @variable(model, t[A2]>=0, Int) #Arrival time
    @variable(model, w[P]>=0, Int) #Amount of freight transported from the depot to parking node
    @variable(model, z[A2,A2], Bin)#Arc(x,y) traversed by SEV
    for i in A2
        @constraint(model, z[i,i] == 0)
        for j in A2
            if i in P && j in P
                @constraint(model, z[i,j]==0)
            end
        end
    end
    @variable(model, f[A2,A2]>=0,Int) #Load of SEV
    #======================================================================#
    @objective(model, Min,
        sum(distances[i, j] * x[i, j] for i in A1, j in A1 if i != j) +
        sum(distances[i, j] * y[i, j] for i in A1, j in A1 if i != j) +
        sum(distances[i, j] * z[i, j] for i in A2, j in A2 if i != j))
    #======================================================================#
    #1 #2
    #Flow conservation at parking for FEV
    @constraint(model, [i in P], sum(x[j,i] for j in A1 if i != j) == sum(x[i,j] for j in A1 if i != j))
    @constraint(model, [i in P], sum(x[i,j] for j in A1 if i != j) <= 1)
    #3
    #Flow conservation of MM
    @constraint(model, [i in P], sum(y[i,j] for j in A1 if i != j) + sum(y[j,i] for j in A1 if i != j)<=1)
    #4
    #Limit for mobile microhub
    @constraint(model, [i in A1, j in A1], y[i,j] <= x[i,j])
    #5 #6
    #Flow conservation at depot
    @constraint(model, sum(x[1,j] for j in A1 if j !=1)==1)
    @constraint(model, sum(x[j,1] for j in A1 if j !=1)==1)
    @constraint(model, [i in A1], y[1,i] ==0)
    @constraint(model, [i in A1], y[i,1] ==0)
    #7
    #Capacity limit for FEV
    @constraint(model, sum(w[p] for p in P)<=Q0)
    #8
    #Can't tow a MM from parking without MM
    @constraint(model, [i in P], sum(y[i,j] for j in A1)<= PI[i-1])
    #9
    #Can't tow a MM to a parking occupied
    @constraint(model, [j in P], sum(y[i,j] for i in A1)<=1-PI[j-1])
    #10
    #If MM leaves a site, the freight to the site should be zero, otw could be positive
    @constraint(model, [p in P], w[p] <= Q1 * (1-sum(y[p,j] for j in A1)))
    #11
    #Link 1st and 2nd
    @constraint(model, [p in P], w[p] == sum(f[p,j] for j in A2 if p !=j))
    #12
    #Capacity limit for MM and connection of FEV
    @constraint(model, [p in P], w[p] <= Q1 * sum(x[i,p] for i in A1))
    #13
    #Can't distribute from a site without MM
    @constraint(model, [p in P], w[p]<=Q1*(sum(y[i,p] for i in A1)+PI[p-1]))

    #14
    #Flow consercvation at parking and customer for SEV
    @constraint(model, [i in A2], sum(z[i,j] for j in A2) == sum(z[j,i] for j in A2))
    #15
    #Flow consercvation at parking node for SEV
    @constraint(model, [p in P], sum(z[p,j] for j in A2) <= length(V2))
    #16
    #Each SEV departs from parking node at most once
    #  @constraint(model, [p in P], sum(z[p,j] for j in A2) <= 1)

    #17
    #Flow conservation at customer node for SEV
    @constraint(model, [i in C], sum(z[i,j] for j in A2) == 1)
    #18
    #Customer demand met
    @constraint(model, [i in C], sum(f[j,i] for j in A2)-sum(f[i,j] for j in A2) == demands[i-1-np])
    #19
    #Connection and capacity limit for SEV
    @constraint(model, [i in A2, j in A2], f[i,j] <= Q2 * sum(z[i,j]))
    #  #20 #21
    #  #Total working time cannot exceed the length of planning horizon
    #  @constraint(model, sum(TT1[i,j]*x[i,j] for i in A1 for j in A1) + eta1*sum(PI[p-1]*x[i,p] for p in P for i in A1)<= zeta)
    #  @constraint(model, [i in C, j in P], t[i]+TT2[i,j]+eta2 <= zeta + M*(1 - z[i,j]))
    #22
    #Time constraint for FEV and MTZ
    @constraint(model, [i in P, j in P], t[i] + eta1*(1-x[i,j]) + TT1[i,j]*x[i,j] <= t[j] + M*(1 - x[i,j]))
    #23
    #Time constraint for SEV and MTZ
    @constraint(model, [i in C, j in C], t[i]+eta2*(1-z[i,j])+TT2[i,j]*z[i,j] <= t[j]+M * (1 - z[i,j]))
    #  #24
    #  @constraint(model, [i in C], t[i] >= time_windows[i-1-np][1])
    #  @constraint(model, [i in C], t[i] <= time_windows[i-1-np][2])
    #25 26
    #Arrival time initialization
    @constraint(model, [i in P], TT1[1,i] * x[1,i] <= t[i])
    @constraint(model, [p in P, j in C], t[p] + TT2[p,j] * z[p,j] <= t[j])

    #27
    #Max duration
    # @constraint(model, [i in P], totalDuration(z, i, 0)<=2000)

    return model, x, y, t, w, z, f

end

