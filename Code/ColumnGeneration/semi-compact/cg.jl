using JuMP, CPLEX
import DataFrames
import HiGHS
import Plots
import SparseArrays
using Random
import Test  #src
include("../../../3index/Utiles.jl")


function calculate_arc_cost(points::Vector{Vector{Int}})
    num_points = length(points)
    arc_cost = Array{Float64}(undef, num_points, num_points)
    for i in 1:num_points
        for j in 1:num_points
            if i == j
                arc_cost[i, j] = 100000.0  # Large value for self-loops
            elseif i in satellites && PI[i-1]==0 && j in satellites && PI[j-1]==0
                arc_cost[i, j] = 100000.0 
            else
                arc_cost[i, j] = sqrt(sum((points[i][k] - points[j][k])^2 for k in 1:length(points[i])))
            end
        end
    end
    for i in satellites
        if PI[i-1] == 0
            arc_cost[1,i] = 100000.0
        end
    end
    # for i in 1:num_points
    #     for j in 1:num_points
    #         print("  d[$i $j]=", round(arc_cost[i,j], digits=2))
    #     end
    #     println("")
    # end
    return arc_cost
end
function getInfo()
    # x_coor_customers, y_coor_customers, x_coor_depot, y_coor_depot, demands
    return  readFile("/Users/lenovo1/Library/CloudStorage/OneDrive-EcoledeManagementdeNormandie/2EVRPMM/Data/Demo/50/C101_50.txt")
end
struct Route
    cost::Float64
    sequence::Vector{Int}
    length::Int
    load::Int
 end
 
 function generateRoute(route::Vector{Int})
    cost = 0
    load = 0
    for i in 1:(length(route) - 1)
        if i!= 1
            load += demands[route[i]]
        end
        from_node = route[i]
        to_node = route[i + 1]
        cost+= arc_cost[from_node, to_node]
    end

    # println("Route:",Route(cost,route,length(route),load) )
    return Route(cost,route,length(route),load) 
 end

# Function to calculate the total demand for a path
function calculate_load(path, demands)
    load = 0
    for node in path[2:end-1]  # Exclude the start and end warehouses
        load += demands[node]
    end
    return load
end
# Function to generate all paths with multiple customers
function generate_paths(warehouses, customers, demands, max_load)
    paths = []
    # Generate all permutations of customer subsets (excluding empty set)
    for num_customers in 1:length(customers)
        for customer_subset in combinations(customers, num_customers)
            for customer_permutation in permutations(customer_subset)
                for start_warehouse in warehouses
                    for end_warehouse in warehouses
                        path = vcat([start_warehouse], customer_permutation, [end_warehouse])
                        # Check if the load of the path exceeds the maximum allowed load
                        if calculate_load(path, demands) <= max_load
                            push!(paths, path)
                        end
                    end
                end
            end
        end
    end
    return paths
end
function getB1(satellites,routes_1)
    # Initialize a 2D array for b[s][r] with zeros
    b = zeros(Int, length(satellites))
    b = vcat(0 , b)
    i = 2
    while i<=length(routes_1) 
        # println("add line")
        b = hcat(b,zeros(Int, length(satellites)+1))
        i += 1
    end
    # println("satellites= $satellites")
    # println("routes_1= $routes_1")
    # Populate the array
    for (s_idx, i) in enumerate(satellites)  # Loop over points (row index)
        for (r, route) in enumerate(routes_1)  # Loop over routes (column index)
            b[s_idx+1, r] = i in route.sequence ? 1 : 0
        end
    end
    return b
end
function transformRoute(x)
    new_route = Vector{Int}()
    current_node = 0
    if length(x) > (1+length(satellites))^2 
        ## Transform a 2e route
        # Find the starting satellite
        for s in satellites
            for j in A2
                if round(value(x[s, j])) == 1
                    current_node = s
                    push!(new_route, s)
                    break
                end
            end
            if current_node != 0
                break
            end
        end
        while true
            for j in A2
                if round(value(x[current_node,j])) == 1
                    current_node = j
                    push!(new_route,current_node)
                    break
                end
            end
            if current_node in satellites
                break
            end
        end
    else
        ## Transform a 1e route
        current_node = 1 
        push!(new_route, current_node)
        while true
            for j in A1
                if round(value(x[current_node,j])) == 1
                    current_node = j
                    push!(new_route,current_node)
                    break
                end
            end
            if current_node == 1
                break
            end
        end
    end
    
    # generateRoute(new_route)
    # println("transformRoute TEST",new_route)
    return new_route
end
function solveRestrictedLinearMasterProblem(branchDecisions::Union{Dict, Nothing} = nothing, branchKey::Union{Vector{Int}, Nothing} = nothing)
    branchDecisions = isnothing(branchDecisions) ? Dict() : branchDecisions
    branchKey = isnothing(branchKey) ? Int[] : branchKey
    b1 = getB1(satellites, routes_1)
    ## Relaxed Restricted Master Problem
    model = Model(CPLEX.Optimizer)

    # set_silent(model)
    @variable(model, 1>=x[1:length(routes_1)]>=0) # Selection of 1e route
    @variable(model, 1>=y[A2, A2]>=0) # Selection of 2e arc
    @variable(model, capacity_microhub >= z[satellites] >=0) # Amount of freight in satellite s
    @variable(model, f[A2, A2]>=0) # Freight flow
    @variable(model, t[customers]>=0)

    if !isempty(branchKey)
        while length(branchKey) != 0
            # println("branchKey=  $branchKey")
            # println("branch on y[",branchDecisions[branchKey][1],",",branchDecisions[branchKey][2],"]")
            # println("branch value= ",branchKey[end])
            @constraint(model, y[branchDecisions[branchKey][1], branchDecisions[branchKey][2]] == branchKey[end])
            branchKey = branchKey[1:end-1]
        end       
    end
    @constraint(model, sync1e[s in satellites], z[s] - M * sum(b1[s,r] * x[r] for r in 1:length(routes_1))<=0)
    @constraint(model, freightOfSatellite[s in satellites], z[s]-sum(f[s,j] for j in customers) == 0)
    @constraint(model, flowConservation2e[i in A2], sum(y[i,j] for j in A2) == sum(y[j,i] for j in A2))
    @constraint(model, flowConservationSatellite[s in satellites], sum(y[s,j] for j in A2) <= capacity_microhub)
    @constraint(model, flowConservationCustomer[c in customers], sum(y[c,j] for j in A2) == 1)
    @constraint(model, customerDemand[c in customers], sum(f[i,c] for i in A2) - sum(f[c,i] for i in A2) == demands[c])
    @constraint(model, capacity2eVehicle[i in A2, j in A2], f[i,j] <= y[i,j] * capacity_2e_vehicle)
    @constraint(model, MTZ[i in customers, j in customers], t[i] + arc_cost[i,j] * y[i,j] <= t[j] + MM * (1-y[i,j]))
    @constraint(model, distanceInitialization[i in satellites, j in customers], arc_cost[i,j] <= t[j]+MM*(1-y[i,j]))
    @constraint(model, distanceEndding[i in customers, j in satellites], t[i] + arc_cost[i,j] <= maximum_duration_2e_vehicle+MM*(1-y[i,j]))
    @constraint(model, distanceLimit[i in customers], t[i]<= maximum_duration_2e_vehicle)

    @objective(model, Min, 
        sum(x[r] * routes_1[r].cost for r in 1:length(routes_1))+sum(y[i,j] * arc_cost[i,j] for i in A2 for j in A2))
    
    optimize!(model)
    π1 = vcat(0, abs.(collect(dual.(sync1e))))
    # @assert is_solved_and_feasible(model; dual=true)
    println("objective value of master problem is: ",value(objective_value(model)))
    
    println("Status of 1e route: ")
    # for r in 1:length(routes_1)
    #     println("   x[$r]=",value(x[r]))
    # end
    for r in 1:length(routes_1)
        if value(x[r]) != 0
            println("   x=", value(x[r]), "1e Route ", routes_1[r].sequence, "   Load= ", routes_1[r].load ,  "   Cost= ", round(routes_1[r].cost,digits=2))
        end 
    end
    # println("Status of satellite:")
    # for s in satellites
    #     if value(z[s])!=0
    #         println("   z[$s]=", value(z[s]))
    #     end
    # end
    # println("")
    # println("Selection of 2e arc:")
    # for i in A2
    #     for j in A2
    #         # if value(y[i,j])!=0
    #             print("   y[$i $j]=",round(value(y[i,j]), digits = 3))
    #         # end
    #     end
    #     print("\n")
    # end
    # println("")
    # println("Freight flow:")
    # for i in A2
    #     for j in A2
    #         if value(f[i,j])!=0
    #             println("   f[$i $j]=",value(f[i,j]))
    #         end
    #     end
    # end
    # println("")
    # println("Accumulate distance:")
    # for i in customers
    #     if value(t[i])!=0
    #         println("   t[$i]=",value(t[i]))
    #     end
    # end
    return π1, value.(y)
end
function solveRestrictedIntegerMasterProblem(branchDecisions::Union{Dict, Nothing} = nothing, branchKey::Union{Vector{Int}, Nothing} = nothing)
    branchDecisions = isnothing(branchDecisions) ? Dict() : branchDecisions
    branchKey = isnothing(branchKey) ? Int[] : branchKey
    b1 = getB1(satellites, routes_1)
    ## Relaxed Restricted Master Problem
    model = Model(CPLEX.Optimizer)

    # set_silent(model)
    @variable(model, 1>=x[1:length(routes_1)]>=0, Int) # Selection of 1e route
    @variable(model, 1>=y[A2, A2]>=0, Int) # Selection of 2e arc
    @variable(model, capacity_microhub >= z[satellites] >=0) # Amount of freight in satellite s
    @variable(model, f[A2, A2]>=0) # Freight flow
    @variable(model, t[customers]>=0)

    if !isempty(branchKey)
        while length(branchKey) != 0
            # println("branchKey=  $branchKey")
            # println("branch on y[",branchDecisions[branchKey][1],",",branchDecisions[branchKey][2],"]")
            # println("branch value= ",branchKey[end])
            @constraint(model, y[branchDecisions[branchKey][1], branchDecisions[branchKey][2]] == branchKey[end])
            branchKey = branchKey[1:end-1]
        end       
    end
    @constraint(model, sync1e[s in satellites], z[s] - M * sum(b1[s,r] * x[r] for r in 1:length(routes_1))<=0)
    @constraint(model, freightOfSatellite[s in satellites], z[s]-sum(f[s,j] for j in customers) == 0)
    @constraint(model, flowConservation2e[i in A2], sum(y[i,j] for j in A2) == sum(y[j,i] for j in A2))
    @constraint(model, flowConservationSatellite[s in satellites], sum(y[s,j] for j in A2) <= capacity_microhub)
    @constraint(model, flowConservationCustomer[c in customers], sum(y[c,j] for j in A2) == 1)
    @constraint(model, customerDemand[c in customers], sum(f[i,c] for i in A2) - sum(f[c,i] for i in A2) == demands[c])
    @constraint(model, capacity2eVehicle[i in A2, j in A2], f[i,j] <= y[i,j] * capacity_2e_vehicle)
    @constraint(model, MTZ[i in customers, j in customers], t[i] + arc_cost[i,j] * y[i,j] <= t[j] + MM * (1-y[i,j]))
    @constraint(model, distanceInitialization[i in satellites, j in customers], arc_cost[i,j] <= t[j]+MM*(1-y[i,j]))
    @constraint(model, distanceEndding[i in customers, j in satellites], t[i] + arc_cost[i,j] <= maximum_duration_2e_vehicle+MM*(1-y[i,j]))
    @constraint(model, distanceLimit[i in customers], t[i]<= maximum_duration_2e_vehicle)

    @objective(model, Min, 
        sum(x[r] * routes_1[r].cost for r in 1:length(routes_1))+sum(y[i,j] * arc_cost[i,j] for i in A2 for j in A2))
    
    optimize!(model)
    println("objective value of master problem is: ",value(objective_value(model)))
    
    println("Status of 1e route: ")
    # for r in 1:length(routes_1)
    #     println("   x[$r]=",value(x[r]))
    # end
    for r in 1:length(routes_1)
        if value(x[r]) != 0
            println("   x=", value(x[r]), "1e Route ", routes_1[r].sequence, "   Load= ", routes_1[r].load ,  "   Cost= ", round(routes_1[r].cost,digits=2))
        end 
    end
    println("Status of satellite:")
    for s in satellites
        if value(z[s])!=0
            println("   z[$s]=", value(z[s]))
        end
    end
    println("")
    println("Selection of 2e arc:")
    for i in A2
        for j in A2
            if value(y[i,j])!=0
                print("   y[$i $j]=",round(value(y[i,j]), digits = 3))
            end
        end
        print("\n")
    end
    # println("")
    # println("Freight flow:")
    # for i in A2
    #     for j in A2
    #         if value(f[i,j])!=0
    #             println("   f[$i $j]=",value(f[i,j]))
    #         end
    #     end
    # end
    # println("")
    # println("Accumulate distance:")
    # for i in customers
    #     if value(t[i])!=0
    #         println("   t[$i]=",value(t[i]))
    #     end
    # end
end
function solve_subproblem_milp(π)
    println("π1= ",π)
    ## Construct model
    model = Model(CPLEX.Optimizer)

    set_silent(model)
    @variable(model, 1 >= x1[A1,A1] >= 0,Int)
    @variable(model, length(A1)>=u[A1]>=0, Int)

    # @constraint(model, satelliteOblg, sum(x1[i,j] for i in A1 for j in subSatellites)>=1)
    @constraint(model, depart, sum(x1[1,i] for i in satellites) ==1)
    @constraint(model, ending, sum(x1[i,1] for i in satellites) ==1)
    @constraint(model, flow[i in A1], sum(x1[j,i] for j in A1)
                                        ==sum(x1[i,j] for j in A1))
    @constraint(model, subtourElm[i in satellites, j in satellites], 
                u[i] + 1 <= u[j] + MM *(1-x1[i,j]))


    @objective(model, Min, 
        sum(arc_cost[i,j] * x1[i,j] for i in A1 for j in A1) - 
        M * sum(π[s] * x1[s,j] for j in A1 for s in satellites))
    
    set_optimizer_attribute(model, "CPX_PARAM_TILIM", 120)
    optimize!(model)

    println("Subprobem 1e: Reduced cost= ", objective_value(model))
    # for i in 1:length(A1)
    #     for j in 1:length(A1)
    #         if round(value(x1[i,j]))==1
    #             println("   x1[$i,$j]=",round(value(x1[i,j]),digits=2))
    #         end
    #     end
    # end
    # println("")
    # for i in A1
    #     println("   u[$i]= ",value(u[i]))
    # end
    new_route = nothing
    if objective_value(model) < -1e-8
        # Reduced cost negative, new column generated and added to master problem
        # println("test  : $num_iter")
        new_route = transformRoute(x1) 
    end
    return new_route, objective_value(model)
end
function solve_subproblem_labelling(π)
    labels = Dict{Int , Vector{Label}}()
    E = Vector{Int}()
    for s in satellites
        if PI[s-1] == 1
            duration = arc_cost[1,s]
            reduced_cost = arc_cost[1,s] - π[s] * M
            unreachables = zeros(length(A1))
            unreachables[s] = 1
            sequence = Vector{Int}()
            push!(sequence, 1, s)
            l = Label(duration, reduced_cost, unreachables,sequence)
            push!(get!(labels, s, Vector{Label}()), l)
            push!(E, s)
        end
    end
    # for (idx, l) in labels
    #     println("$idx => $l")
    # end
    
    num_iter = 1
    start_time = time()
    while length(E) != 0 #&& num_iter < 15
        println("=================Iteration $num_iter of Labelling=================")
        @info "Start time = $start_time", time()-start_time
        if time()-start_time > 120
            break
        end
        vi = E[1]
        println("\nE= $E vi= $vi")
        succ = Vector{Int}()
        for j in A1
            if arc_cost[vi,j] <= 10000
                push!(succ, j)
            end
        end
        println("successor of $vi are: $succ")
        for vj in succ
            for label in labels[vi]
                if label.unreachables[vj] == 0
                    new_label = extend(label, vi, vj,π)
                    found = false
                    if haskey(labels, vj)
                        for l in labels[vj] 
                            # println(new_label.sequence , l.sequence, new_label.sequence == l.sequence)
                            if new_label.sequence == l.sequence
                                found = true
                                break
                            end
                        end
                    end
                    if !found
                        push!(get!(labels, vj, Vector{Label}()), new_label)
                        push!(E, vj) 
                    end
                   
                end
            end
        end
        setdiff!(E, vi)
        println("E at the end of interation $E")
        
        # for (idx, l) in labels
        #     println("$idx =>")
        #     for ele in l
        #         println(ele)
        #     end
        # end
        num_iter += 1
    end

    min_solution = labels[1][1]
    for l in labels[1]
        if l.reduced_cost<min_solution.reduced_cost
            min_solution = l
        end
    end

    println("Min RC solution= $min_solution")
    new_route = nothing
    if min_solution.reduced_cost < -1e-8
        # Reduced cost negative, new column generated and added to master problem
        new_route = min_solution.sequence
    end
    return new_route, min_solution.reduced_cost
end
function solve_column_generation_milp(rc1, routes_1,branchDecisions::Union{Dict, Nothing} = nothing, branchKey::Union{Vector{Int}, Nothing} = nothing)
    branchDecisions = isnothing(branchDecisions) ? Dict() : branchDecisions
    branchKey = isnothing(branchKey) ? Int[] : branchKey

    status = false
    num_iter = 1
    y = nothing
    if branchKey != nothing
        printText = branchKey
    else
        printText = ""
    end
    start_time = time()
    while !(status) && time()-start_time <200
        println("\n======================Iteration $num_iter $printText======================")
        println("1e route= $routes_1")
        global b1
        b1 = getB1(satellites, routes_1)
        π1, y = solveRestrictedLinearMasterProblem(branchDecisions, branchKey)
        
        new_route = nothing
        new_route,rc_v= solve_subproblem_milp(π1)
        if !(isempty(rc1)) && rc_v == rc1[end]
            @info "Reduced cost converged, terminating column generation for 1st echelon"
            status = true
        end
        push!(rc1,rc_v)
        if new_route == nothing
            @info "No new 1e routes generated"
            status = true
        else
            @info "New 1e route $new_route "
            push!(routes_1, generateRoute(new_route))
        end
        num_iter += 1
    end
    return rc1, routes_1, y
end
function solve_column_generation_labelling(rc1, routes_1)
    # branchDecisions = isnothing(branchDecisions) ? Dict() : branchDecisions
    # branchKey = isnothing(branchKey) ? Int[] : branchKey

    status = false
    num_iter = 1
    y = nothing
    # if branchKey != nothing
    #     printText = branchKey
    # else
    #     printText = ""
    # end
    start_time = time()
    while !(status) && time()-start_time <600#&& num_iter<2
        println("\n======================Iteration $num_iter at ",time()-start_time ,"s of column generation======================")
        println("1e route= $routes_1")
        global b1
        b1 = getB1(satellites, routes_1)
        π1, y = solveRestrictedLinearMasterProblem()
        
        new_route = nothing
        new_route,rc_v= solve_subproblem_labelling(π1)

        if !(isempty(rc1)) && rc_v == rc1[end]
            @info "Reduced cost converged, terminating column generation for 1st echelon"
            status = true
        end
        push!(rc1,rc_v)
        if new_route == nothing
            @info "No new 1e routes generated"
            status = true
        else
            @info "New 1e route $new_route "
            push!(routes_1, generateRoute(new_route))
        end
        num_iter += 1
    end

    return rc1, routes_1, y
end

function getBranchingNode(y)
    fractional_y = [(i,j) for i in A2, j in A2 if 0<y[i,j]<1]
    if isempty(fractional_y)
        println("Optimal Integer Solution Found")
        return nothing, nothing
    else
        (i_branch, j_branch) = fractional_y[1]
    end

    println("Branching on y[$i_branch, $j_branch]")
    return i_branch, j_branch
end

struct Label
    duration::Float64
    reduced_cost::Float64
    unreachables::Vector{Int}
    sequence::Vector{Int}
end

function extend(label::Label, vi::Int, vj::Int, π)
    duration = label.duration + arc_cost[vi,vj]
    reduced_cost = label.reduced_cost + arc_cost[vi,vj]
    # println("TESTTEST π=$π")
    sequence = copy(label.sequence)
    push!(sequence, vj)
    if vj != 1
        reduced_cost -=  M * π[vj] 
    end
    unreachables = zeros(nb_s+1)
    for i in sequence
        if vj == 1
            unreachables[i] = 1
        else
            if i != 1
                unreachables[i] = 1
            end
        end
    end
    for k in A1
        if duration + arc_cost[vj,k] >= maximum_duration_1e_vehicle && k!=1
            # @info "Can't propagate from $vj to $k due to duration limit"
            unreachables[k] = 1
        elseif duration + arc_cost[vj,k] > maximum_duration_1e_vehicle && k==1
            # @info "Can't propagate from $vj to $k due to duration limit"
            unreachables[k] = 1
        end
    end
    l = Label(duration, reduced_cost,unreachables,sequence)
    return l
end

nb_s = 15
nb_m = 6

x_coor_customers, y_coor_customers, x_coor_depot, y_coor_depot, demands = getInfo()
demands = vcat(zeros(nb_s+1),demands)

PI = vcat(ones(Int, nb_m),zeros(Int, nb_s-nb_m))
Random.seed!(42)
shuffle!(PI)
x_coor_parkings = []
y_coor_parkings = []
min_x,min_y = minimum(x_coor_customers),minimum(y_coor_customers)
max_x,max_y = maximum(x_coor_customers),maximum(y_coor_customers)
for i in 1:nb_s
    push!(x_coor_parkings, rand(min_x: max_x))
    push!(y_coor_parkings, rand(min_y: max_y))
end
coor_parkings = [[x,y] for (x,y) in zip(x_coor_parkings , y_coor_parkings)]
coor_customers = [[x,y] for (x,y) in zip(x_coor_customers, y_coor_customers)]
coor = vcat([[0,0]], coor_parkings, coor_customers)
nb_vehicle_per_satellite = 10
capacity_2e_vehicle = 50
capacity_microhub = 200
maximum_duration_2e_vehicle = 300
maximum_duration_1e_vehicle = 1000
points = 1:length(demands)

# Define the range of customers
customers = 2+nb_s : length(coor)
satellites = 2:1 + nb_s
arc_cost = calculate_arc_cost(coor)
A2 = 2:length(coor)
A1 = 1:1+length(satellites)
M = sum(demands)# capacity of a satellite
MM = 100

rs_1 = Vector{Vector{Int}}()
for idx in 1:length(PI)
    r = Vector{Int}()
    if PI[idx] == 1
        push!(r,1)
        push!(r, idx+1)
        push!(r,1)
        push!(rs_1, r)
    end

end
# rs_1 =  [[1,2,1],[1,5,1]]
routes_1 = Vector{Route}()
for r in rs_1
    push!(routes_1, generateRoute(r))
end
rc1 = []
branchingDecisions = Dict{Vector{Int}, Vector{Int}}()


# while true
#     1. solve restricted linear master problem
#     2. solve pricing subproblem till reduced cost non negative(column generation)
#     3. if solution integral break
#       else 
#     4. branch on y, Integerity force
#     5. back to 1. with generated route and branch constriant
# end

# global b1
# b1 = getB1(satellites, routes_1)
# π,_ = solveRestrictedLinearMasterProblem()
# solve_subproblem_labelling(π)

t1 = @elapsed begin
    global rc1, routes_1
    rc1, routes_1, y = solve_column_generation_milp(rc1, routes_1)
    # rc1, routes_1, y = solve_column_generation_labelling(rc1, routes_1)
    # println("")
    println("reduced costs: $rc1")
    println("1e routes:")
    for r in routes_1
        println(r)
    end
    solveRestrictedIntegerMasterProblem()
end

println("Total Execution Time: ", t1, " seconds")


# (i_branch, j_branch) = getBranchingNode(y)


# println(branchingDecisions)
# if isempty(branchingDecisions)
#     branchingDecisions[[0]] = [i_branch, j_branch]
#     branchingDecisions[[1]] = [i_branch, j_branch]
# else
#     for (k,v) in branchingDecisions
#         println("$k => $v")
#         new_key = push(k,0)
#         branchingDecisions[new_key] = [i_branch, j_branch]
#         new_key = push(k,1)
#         branchingDecisions[new_key] = [i_branch, j_branch]
#     end
# end
# for (k,v) in branchingDecisions
#     println("$k => $v")
# end

# global num_level = 1
# global status = true
# while status && num_level<16
#     println("\n#==================================================================================================#\n")
#     global new_branchs = Dict{Vector{Int}, Vector{Int}}()
#     for (k,v) in branchingDecisions
#         if length(k) == num_level
#             println("Work on key= $k")
#             global rc1, routes_1
#             rc1, routes_1, y = solve_column_generation(rc1, routes_1,branchingDecisions,k)

#             (i_branch, j_branch) = getBranchingNode(y)
#             if isnothing(i_branch)
#                 global status = false
#                 break
#             end

#             new_value = [i_branch, j_branch]
#             new_key_left = push!(copy(k),0)
#             new_key_right = push!(copy(k),1)

#             new_branchs[new_key_left]= new_value
#             new_branchs[new_key_right]= new_value
#         end

#     end

#     for (k,v) in new_branchs
#         # println("$k => $v")
#         branchingDecisions[k] = v
#     end
#     println("")
#     # for (k,v) in branchingDecisions
#     #     println("$k => $v")
#     # end
#     global num_level
#     num_level += 1
# end

# # println("\nReduced cost= \n$rc1")
# println("\n1e Routes= ")
# for r in routes_1
#     println(r)
# end