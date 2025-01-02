using JuMP, CPLEX
using BlockDecomposition, Coluna

# coluna = optimizer_with_attributes(
#     Coluna.Optimizer,
#     "params" => Coluna.Params(
#         solver = Coluna.Algorithm.TreeSearchAlgorithm() # default branch-cut-and-price
#     ),
#     "default_optimizer" => CPLEX.Optimizer # GLPK for the master & the subproblems
# );
function buildModel() 
    P = 2 : np+1 #Set of parking place
    C = np+2 : np+nc+1 #Set of customers
    A1 = 1 : 1+np #Set of FE arcs
    A2 = 2 : 1+np+nc #Set of SE arcs
    N = 1:np+nc+1 #Set of nodes
    M = 100000
    MM = 1 : length(V2)*sum(PI)

    TT1 = distances
    TT2 = distances

    # @axis(M_axis, V2); #the index set of parking
    # model = BlockModel(coluna); 

    # Decision variable
    @variable(model, x[A1,A1], Bin)#Arc(x,y) traversed by FEV
    @variable(model, y[A1,A1], Bin)#Arc(x,y) traversed by MM
    for i in 1:np+1
        @constraint(model, x[i, i] == 0)
        @constraint(model, y[i, i] == 0)
    end
    @variable(model, t[A2]>=0, Int) #Arrival time
    @variable(model, w[P]>=0, Int) #Amount of freight transported from the depot to parking node
    @variable(model, z[A2,A2,V2], Bin)#Arc(x,y) traversed by SEV
    for k in V2
        for i in A2
            @constraint(model, z[i,i,k] == 0)
            for j in A2
                if i in P && j in P
                    @constraint(model, z[i,j,k]==0)
                end
            end
        end
    end
    @variable(model, f[A2,A2]>=0,Int) #Load of SEV
    #======================================================================#
    @objective(model, Min,
        sum(distances[i, j] * x[i, j] for i in A1, j in A1 if i != j) +
        sum(sum(distances[i, j] * z[i, j, k] for i in A2, j in A2 if i != j) for k in V2) )
    #======================================================================#
    #1 #2
    #Flow conservation at parking for FEV
    @constraint(model, flow_fev_equ[i in P], sum(x[j,i] for j in A1 if i != j) == sum(x[i,j] for j in A1 if i != j))
    @constraint(model, flow_fev_max[i in P], sum(x[i,j] for j in A1 if i != j) <= 1)
    #3
    #Flow conservation of MM
    @constraint(model, flow_mm[i in P], sum(y[i,j] for j in A1 if i != j) + sum(y[j,i] for j in A1 if i != j)<=1)
    #4
    #Limit for mobile microhub
    @constraint(model, mm_move[i in A1, j in A1], y[i,j] <= x[i,j])
    #5 #6
    #Flow conservation at depot
    @constraint(model, flow_fev_depot_out,sum(x[1,j] for j in A1 if j !=1)==1)
    @constraint(model, flow_fev_depot_in,sum(x[j,1] for j in A1 if j !=1)==1)
    @constraint(model, flow_mm_depot_out[i in A1], y[1,i] ==0)
    @constraint(model, flow_mm_depot_in[i in A1], y[i,1] ==0)
    #7
    #Capacity limit for FEV
    @constraint(model, cap_limit_fev,sum(w[p] for p in P)<=Q0)
    #8
    #Can't tow a MM from parking without MM
    @constraint(model, mm_disponibility_out[i in P], sum(y[i,j] for j in A1)<= PI[i-1])
    #9
    #Can't tow a MM to a parking occupied
    @constraint(model, mm_disponibility_in[j in P], sum(y[i,j] for i in A1)<=1-PI[j-1])
    #10
    #If MM leaves a site, the freight to the site should be zero, otw could be positive
    @constraint(model, freight_amount_mm[p in P], w[p] <= Q1 * (1-sum(y[p,j] for j in A1)))
    #11
    #Link 1st and 2nd
    @constraint(model, link_echelons[p in P], w[p] == sum(f[p,j] for j in A2 if p !=j))
    #12
    #Capacity limit for MM and connection of FEV
    @constraint(model, cap_limit_mm[p in P], w[p] <= Q1 * sum(x[i,p] for i in A1))
    #13
    #Can't distribute from a site without MM
    @constraint(model, cap_limit_mm_disponibility[p in P], w[p]<=Q1*(sum(y[i,p] for i in A1)+PI[p-1]))


    #14
    # Flow conservation at parking and customer for flow_sev
    @constraint(model, flow_sev[i in A2], sum(z[i,j,k] for j in A2 for k in V2) == sum(z[j,i,k] for j in A2 for k in V2) )
    @constraint(model, flow_sev_parking[i in A2, j in A2], sum(z[i,j,k] for k in V2)<=1)
    # Depart once
    @constraint(model, flow_sev_cus[k in V2], sum(z[i,j,k] for i in P for j in C) <= 1)
    #15
    # Flow consercvation for SEV at parking node
    @constraint(model, customer_demand[p in P], sum(z[p,j,k] for j in A2 for k in V2) <= length(K2))
    #16
    # Flow conservation for SEV at customer node
    @constraint(model, cap_limit_sev[i in C], sum(z[i,j,k] for j in A2 for k in V2) == 1)
    @constraint(model, time_mtz_fev[i in C, k in V2], sum(z[i,j,k] for j in A2) == sum(z[j,i,k] for j in A2))
    # Break symmetry
    # @constraint(model, [i in A2, j in A2, k in 1:(length(V2)-1)], z[i,j,k]>= z[i,j,k+1])
    # 17
    # Customer demand met
    @constraint(model, cus[i in C], sum(f[j,i] for j in A2)-sum(f[i,j] for j in A2) == demands[i-1-np])
    #18
    # Connection and capacity limit for SEV
    @constraint(model, cinnec[i in A2, j in A2], f[i,j] <= Q2 * sum(z[i,j,k] for k in V2))


    # 22
    # #Time constraint for FEV and MTZ
    @constraint(model, fevmzt[i in P, j in P], t[i] + eta1 * (1-x[i,j]) + TT1[i,j] * x[i,j] <= t[j] + M * (1 - x[i,j]))
    # #23
    # #Time constraint for SEV and MTZ
    @constraint(model, sevttz[i in C, j in C, k in V2], t[i] + eta2 * (1-z[i,j,k]) + TT2[i,j] * z[i,j,k] <= t[j] + M * (1 - z[i,j,k]))

    #25 26
    # #Arrival time initialization
    @constraint(model, arrivalFEV[i in P], TT1[1,i] * x[1,i] <= t[i])
    @constraint(model, arrivalSEV[p in P, j in C, k in V2], t[p] + TT2[p,j] * z[p,j,k] <= t[j])

    # # Max duration
    # @constraint(model, [k in MM], sum(z[i,j,k] for i in A2, j in A2) <= maxDuration)

    # @dantzig_wolfe_decomposition(model, decomposition, M_axis)
    # master = getmaster(decomposition)
    # subproblems = getsubproblems(decomposition)

    specify!.(subproblems, lower_multiplicity = 0, upper_multiplicity = 1)
    getsubproblems(decomposition)

    optimize!(model)

    #20 #21
    #  #Total working time cannot exceed the length of planning horizon
    #  @constraint(model, sum(TT1[i,j]*x[i,j] for i in A1 for j in A1) + eta1*sum(PI[p-1]*x[i,p] for p in P for i in A1)<= zeta)
    #  @constraint(model, [i in C, j in P], t[i]+TT2[i,j]+eta2 <= zeta + M*(1 - z[i,j]))   
     #  #24
    #  @constraint(model, [i in C], t[i] >= time_windows[i-1-np][1])
    #  @constraint(model, [i in C], t[i] <= time_windows[i-1-np][2])
    return model, x, y, t, w, z, f

end

