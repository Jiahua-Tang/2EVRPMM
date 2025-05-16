using JuMP, CPLEX
import DataFrames
import HiGHS
import Plots
import SparseArrays
using Random
import Test  #src

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

function getA(customers,routes_2)
    # Initialize a 2D array for a[i][r] with zeros
    a = zeros(Int, length(customers))
    i=1
    while i<=length(routes_2)
        a = hcat(a,zeros(Int, length(customers)))
        i += 1
    end
    # Populate the array
    for (i_idx, i) in enumerate(customers)  # Loop over points (row index)
        for (r, route) in enumerate(routes_2)  # Loop over routes (column index)
            a[i_idx, r] = i in route.sequence ? 1 : 0
        end
    end
    return a
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
    # Populate the array
    for (s_idx, i) in enumerate(satellites)  # Loop over points (row index)
        for (r, route) in enumerate(routes_1)  # Loop over routes (column index)
            b[s_idx+1, r] = i in route.sequence ? 1 : 0
        end
    end
    return b
end

function getB2Out(satellites,routes_2)
    # Initialize a 2D array for b[s][r] with zeros
    b = zeros(Int, length(satellites))
    b = vcat(0 , b)
    i = 2
    while i <= length(routes_2) 
        # println("add line")
        # println("$i : $b")
        b = hcat(b,zeros(Int, length(satellites)+1))        
        i += 1 
    end
    # Populate the array
    for (s_idx, i) in enumerate(satellites)  # Loop over points (row index)
        for (r, route) in enumerate(routes_2)  # Loop over routes (column index)
            b[s_idx + 1, r] = route.sequence[1] == i ? 1 : 0
        end
    end
    return b
end

function getB2(satellites,routes_2)
    # Initialize a 2D array for b[s][r] with zeros
    b = zeros(Int, length(satellites))
    b = vcat(0 , b)
    i = 2
    while i <= length(routes_2) 
        # println("add line")
        # println("$i : $b")
        b = hcat(b,zeros(Int, length(satellites)+1))        
        i += 1 
    end
    # Populate the array
    for (s_idx, i) in enumerate(satellites)  # Loop over points (row index)
        for (r, route) in enumerate(routes_2)  # Loop over routes (column index)
            b[s_idx + 1, r] = count(==(i), route.sequence)
        end
    end
    return b
end

function updateParameters()
    ## Update parameters
    a = getA(customers,routes_2)
    b_1= getB1(satellites,routes_1)
    b_2 = getB2(satellites,routes_2)
    b_2_out = getB2Out(satellites,routes_2)

    # println("a[i][r]:")
    # println("   ",a)
    # println("b1[s][r]: number of 1e route: ",length(routes_1))
    # println("   ",b_1)
    # println("b2[s][r]: number of 2e route: ", length(routes_2))
    # println("   ",b_2,"\n")

    return a, b_1, b_2, b_2_out
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

function solve_pricing_1e(π)
    println("π1= ",π)
    ## Construct model
    model = Model(CPLEX.Optimizer)

    # set_silent(model)
    @variable(model, 1 >= x1[A1,A1] >= 0,Int)
    @variable(model, u[A1]>=0, Int)

    # @constraint(model, satelliteOblg, sum(x1[i,j] for i in A1 for j in subSatellites)>=1)
    @constraint(model, depart, sum(x1[1,i] for i in satellites) ==1)
    @constraint(model, ending, sum(x1[i,1] for i in satellites) ==1)
    @constraint(model, flow[i in A1], sum(x1[j,i] for j in A1)
                                        ==sum(x1[i,j] for j in A1))
    @constraint(model, subtourElm[i in A1, j in satellites], 
                u[i] + arc_cost[i,j] * x1[i,j] <= u[j] + MM *(1-x1[i,j]))


    @objective(model, Min, sum(arc_cost[i,j] * x1[i,j] for i in A1 for j in A1)
                                -sum(π[s] * sum(x1[s,j] for j in A1) for s in satellites))
    
    # set_optimizer_attribute(model, "CPX_PARAM_TILIM", 120)
    optimize!(model)

    println("Subprobem 1e: Reduced cost= ", objective_value(model))
    for i in 1:length(A1)
        for j in 1:length(A1)
            if round(value(x1[i,j]))==1
                println("   x1[$i,$j]=",round(value(x1[i,j]),digits=2))
            end
        end
    end
    # for i in A1
    #     println("u[$i]= ",value(u[i]))
    # end
    new_route = nothing
    if objective_value(model) < 0
        # Reduced cost negative, new column generated and added to master problem
        # println("test  : $num_iter")
        new_route = transformRoute(x1) 
    end
    return generateRoute(new_route), objective_value(model)
end

function solve_1e_loop(num_iter, rc1, routes_1)
    status = false

    while !(status) 
        println("\n======================Iteration $num_iter======================")
        a, b_1, b_2, b_2_out = updateParameters()
        π1,_ = runMasterProblem(a, b_1, b_2, b_2_out)
        new_route = nothing
        new_route,rc_v = solve_pricing_1e(π1)
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
    return num_iter, rc1, routes_1
end

function solve_pricing_2e(π2, π3, π4, π5, π6)

    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, y1[A2, A2]>=0, Int)
    @variable(model, u[customers]>=0,Int)
    @variable(model, bl[satellites])

    @objective(model, Min, sum(arc_cost[i,j] * y1[i,j] for i in A2 for j in A2) + 
                            sum(π2[s] * (y1[s,j]+y1[j,s]) for s in satellites for j in customers)-
                            sum(π3[i] * y1[i,j] for i in customers for j in A2)+
                            sum(π4[s] * (y1[s,j]+y1[j,s]) for s in satellites for j in customers)+
                            sum(π5[s] * y1[s,j] for s in satellites, j in customers)+
                            sum(π6[s] * bl[s] for s in satellites)
              )
    @constraint(model, attributionBLoad[s in satellites], bl >= sum(y1[i,j] * demands[i] for i in customers for j in A2))
    @constraint(model, flowSatelliteOut, sum(y1[i,j] for i in satellites, j in customers) ==1 )
    @constraint(model, flowSatelliteIn, sum(y1[i,j] for i in customers, j in satellites) ==1 )
    @constraint(model, flowPoints[i in customers], sum(y1[i,j] for j in A2) == sum(y1[j,i] for j in A2))
    @constraint(model, capacityVehicle, sum(demands[i] * y1[i,j] for i in customers for j in A2) <= capacity_2e_vehicle)
    @constraint(model, subtourElm[i in customers, j in customers], u[i] + arc_cost[i,j] * y1[i,j] <= u[j] + MM *(1-y1[i,j]))

    optimize!(model)

    println("Subprobem 2e: Reduced cost= ", objective_value(model))
    new_route = nothing
    if objective_value(model) < 0
        # Reduced cost negative, new column generated and added to master problem
        new_route = transformRoute(y1) 
    end
    for i in A2
        for j in A2
            if round(value(y1[i,j]))==1
                println("   y1[$i, $j]=1")
            end
        end
    end
    return new_route,objective_value(model)
end
struct Label
    served::Bool
    duration::Float64
    load::Int
    reduced_cost::Float64
    s::Int 
    unreachables::Vector{Int} # unreachable
    sequence::Vector{Int}
end

function extend(lambda, i, j, cust)
    # Extend( lambdai, vj): Function that returns the label 
    # resulting from the extension of label i i towards 
    # node vj when the extension is possible, nothing 
    # otherwise. 
    # The function first updates the consumption 
    # of resources l 1, . . . , L. If the resource 
    # constraints are satisfied, it explores the set of 
    # outgoing arcs to update the vector of unreachable 
    # nodes and the number of unreachable nodes.

    served = copy(lambda.served)
    if j == cust
        served = true
    end
    

    reduced_cost = lambda.reduced_cost
    reduced_cost += arc_cost[i,j]
    if j in satellites
        reduced_cost += ( π2[j] + π4[j])
    else
        reduced_cost -= π3[j]
    end

    unreachables = copy(lambda.unreachables)
    unreachables[j] = 1

    sequence = copy(lambda.sequence)
    push!(sequence,j)

    duration = lambda.duration + arc_cost[i,j]
    load = lambda.load + demands[j]
    for k in A2
        if unreachables[k] == 0
            if duration + arc_cost[j,k] >= maximum_duration_2e_vehicle && !(k in satellites)
                unreachables[k] = 1
                # @info ("$j to $k : duration limit")
            elseif duration + arc_cost[j,k] > maximum_duration_2e_vehicle && k in satellites
                unreachables[k] = 1
                # @info ("$j to $k : duration limit")
            elseif load + demands[k] > capacity_2e_vehicle && k in customers
                unreachables[k] = 1
                # @info "$j to $k : load capacity limit"
            end
        end
    end

    s = sum(unreachables)

    new_label = Label(served, duration, load, reduced_cost, s, unreachables, sequence)
    return new_label
    # end
  
end

function dominates(l1,l2)
    result = false

    if l1.served == true && l2.served == false
        result = true
    # elseif l1.duration < l2.duration
    #     result = true
    # elseif l1.load < l2.load
    #     result = true
    elseif l1.reduced_cost < l2.reduced_cost
        result = true
    elseif l1.s > l2.s
        result = true
    else
        for i in 1:length(l1.unreachables)
            if l1.unreachables[i] < l2.unreachables[i]
                result = true
                break
            end
        end        
    end
    
    return result
end

function solve_pricing_labelling_2e(π2, π3, π4, π5, π6)
#     @objective(model, Min, sum(arc_cost[i,j] * y1[i,j] for i in A2 for j in A2) + 
#     sum(π2[s] * (y1[s,j]+y1[j,s]) for s in satellites for j in customers)-
#     sum(π3[i] * y1[i,j] for i in customers for j in A2)+
#     sum(π4[s] * (y1[s,j]+y1[j,s]) for s in satellites for j in customers)+
#     sum(π5[s] * y1[s,j] for s in satellites, j in customers)+
#     sum(π6[s] * y1[s,j] for s in satellites) * 
#                 sum(demands[i] * y1[i,j] for i in customers for j in A2)
#     )
   
    # Select three customers with largest dual prices to start labelling algo
    combinations = zeros(nb_s+1, nb_s+1)
    for s in 1:nb_s+1
        for s2 in 1:nb_s+1
            if s != 1 && s2 != 1
                combinations[s, s2] = π2[s] + π2[s2] + π4[s] + π4[s2] + π5[s] + π6[s]
                # println("combinations[$s $s2]= ", combinations[s,s2])
            end
        end
    end
    
    start_node = argmax(combinations)[1]
    end_node = argmax(combinations)[2]

    obl_served = argmax(π3)

    println("obl_served= $obl_served")
    
    labels = Dict{Int, Vector{Label}}()

    # Initialize starting labels
    vec = vcat(ones(nb_s+1) , zeros(length(customers))) # Initialize a zero vector
    vec[end_node] = 0               # Set the p-th element to 1
    sequence = Vector{Int}()
    push!(sequence, start_node)
    l = Label(false, 0, 0, π2[start_node] + π4[start_node] + π5[start_node] + π6[start_node], 1, vec, sequence)  # Create the Label
    labels[start_node] = [l]

    # println(l)

    if labels[end_node] == nothing
        labels[end_node] = []
    end
    for i in setdiff(A2, start_node)
        labels[i] = []
    end

    E = Vector{Int}()
    F = Vector{Int}()
    push!(E, start_node)


    # Begin loops
    num_iter = 1
    insert = true
    while num_iter < 18 && insert
        # println("\n================================================Iteration $num_iter================================================")
        current_node = E[1]
        
        # Define the successor of a node
        succ = Vector{Int}()
        for j in customers 
            if j != current_node
                push!(succ, j)
            end
        end 
        if !(current_node in satellites)
           push!(succ, end_node) 
        end
        # println("Existing labels before extension: ")
        # for (k, v) in labels
        #     if !isempty(v)
        #         println("   $k =>")
        #         for l in v
        #             println("   ", l)
        #         end
        #     end
        # end

        # println("\nSuccessor of $current_node= $succ\n")
    #     println("\nlabels[$cust1]= ", labels[cust1],"\n")

        # Exploration of the successors of a node
        for j in succ
            # Create labels from i to j
            # println("Show all labels of current node: label[$current_node]")
            for lambda in labels[current_node]

                # println("During iteration, $lambda of node $current_node, propagate to $j")
                if lambda.unreachables[j] == 0
                    # Extend label
                    new_label = extend(lambda, current_node, j, obl_served)
                    insert = true
                    for l in labels[j] 
                        if new_label.sequence == l.sequence
                            insert = false
                        end
                    end
                    if insert
                        push!(labels[j], new_label)
                        # println("NEW LABEL = ", new_label.sequence)
                    end
                    
                end
                if !(j in satellites) # && !(j in F)
                    push!(E,j)
                end
            end
            # println("Existing labels after extension from $current_node to $j: ")
            # for l in labels[j]
            #     println("   ",l)
            # end
            # println("")
        end

        setdiff!(E, current_node)
        push!(F, current_node)
        # println("E= $E")
        # println("F= $F")

        num_iter += 1
    end

    solutions = Vector{Route}()
    reduced_cost = Vector{Float64}()

    # println(length(labels[end_node]))
    for l in labels[end_node]
        if l.served == true && l.reduced_cost <= -1e-10
            push!(solutions, generateRoute(l.sequence))
            push!(reduced_cost, l.reduced_cost)
        end
    end
    println("number of new routes generated: ", length(solutions),"\n")
    return solutions, reduced_cost

end


function solve_pricing_labelling_last_version(π2, π3, π4, π5, π6)
    #     @objective(model, Min, sum(arc_cost[i,j] * y1[i,j] for i in A2 for j in A2) + 
    #     sum(π2[s] * (y1[s,j]+y1[j,s]) for s in satellites for j in customers)-
    #     sum(π3[i] * y1[i,j] for i in customers for j in A2)+
    #     sum(π4[s] * (y1[s,j]+y1[j,s]) for s in satellites for j in customers)+
    #     sum(π5[s] * y1[s,j] for s in satellites, j in customers)+
    #     sum(π6[s] * y1[s,j] for s in satellites) * 
    #                 sum(demands[i] * y1[i,j] for i in customers for j in A2)
    #     )
    
        println("π3= $π3")
       
        # Select three customers with largest dual prices to start labelling algo
        start_points = sort(partialsortperm(π3, 1:3, rev=true))
        println(start_points)
        labels = Dict{Int, Vector{Label}}()
    
        # Initialize starting labels
        for p in start_points[1]
            vec = zeros(length(coor))  # Initialize a zero vector
            vec[p] = 1               # Set the p-th element to 1
            sequence = Vector{Int}()
            push!(sequence, p)
            l = Label(false, 0, demands[p], 0, 1, vec, sequence)  # Create the Label
            labels[p] = [l]
        end
    
        for i in setdiff(A2, start_points[1])
            labels[i] = []
        end
    
        # E = Dict{Int, Vector{Label}}()
        E = Vector{Int}()
        E = start_points
        F = Vector{Int}()
    
    
        # Begin loops
        num_iter = 1
        while !isempty(E) && num_iter < 3
            println("\n\n========================Iteration $num_iter========================")
            cust1 = E[1]
            
            # Define the successor of a node
            succ1 = Vector{Int}()
            if cust1 in customers
                for j in A2
                    if arc_cost[cust1,j] <= 10000
                        push!(succ1, j)
                    end
                end
            end
    
            for (k, v) in labels
                if !isempty(v)
                    println("$k =>")
                    for l in v
                        println("   ", l)
                    end
                end
                # println(length(v))
            end
    
            println("succ of $cust1= $succ1\n")
            println("\nlabels[$cust1]= ", labels[cust1],"\n")
    
            # Exploration of the successors of a node
            for j in succ1 
    
                # Create labels from i to j
                for lambda in labels[current_node]
                    if lambda.unreachables[j] == 0
                        # Extend label
                        new_label = extend(lambda, cust1, j)
    
                        # # Check dominance
                        # if !isempty(labels[j])
                        #     println("size of labels[$j]= ", length(labels[j]))
                        #     if dominates(labels[j], new_label)
                        #         println("Dominance observed: ", labels[j], " doominates ", new_label)
                        #         labels[j] = new_label
                        #     else
                        #         println(labels[j], " doesn't doominate ", new_label)
                        #     end
                        # else
                            push!(labels[j], new_label)
                        # end
                        # println("lambda= ",lambda)
       
                    end
    
                end
                println("labels[$j]: ")
                for l in labels[j]
                    println("   ",l)
                end
                if !(j in F)
                    push!(E,j)
                end
            end
    
            solutions = Vector{Label}()
            for s in satellites 
                if !(isempty(labels[s])) 
                    l = labels[s][1]
                    for lb in labels[s]
                        if lb.reduced_cost < l.reduced_cost
                            l = lb
                        end
                    end
                    push!(solutions, l)
                end
            end
            best_route = [1,2]
            min_reduced_cost = solutions[best_route[1]].reduced_cost+solutions[best_route[2]].reduced_cost
            for (idx, s) in enumerate(solutions)
                for (idx2, s2) in enumerate(solutions)
                    if idx < idx2
                        if s.reduced_cost + s2.reduced_cost < min_reduced_cost
                            best_route = [idx, idx2]
                            min_reduced_cost = s.reduced_cost + s2.reduced_cost
                        end
                    end
                end
            end
    
            println("best_route= ", solutions[best_route[1]].sequence, solutions[best_route[2]].sequence,"min_reduced_cost= $min_reduced_cost")
    
            println("solutions:")
            for s in solutions
                println(s)
            end
    
            setdiff!(E, [cust1])
            push!(F, cust1)
            println("E= $E")
            println("F= $F")
    
            num_iter += 1
        end
    
    
    end


    function runMasterProblem(a, b_1, b_2, b_2_out)
    ## Relaxed Restricted Master Problem
 
    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, 1>=x[1:length(routes_1)]>=0, Int)
    @variable(model, 1>=y[1:length(routes_2)]>=0, Int)
    @variable(model, 1>=z[satellites]>=0, Int)
    @variable(model, k[s in satellites], Int)  # Auxiliary integer variable


    @constraint(model, sync1e[s in satellites], z[s] - sum(b_1[s,r] * x[r] for r in 1:length(routes_1))<=0)
    @constraint(model, sync2e[s in satellites], sum(b_2[s,r] * y[r] for r in 1:length(routes_2)) - M * z[s]<=0)
    @constraint(model, custVisit[i in customers], 
        1 - sum(a[i-1-length(satellites),r] * y[r] for r in 1:length(routes_2)) <= 0 )
    @constraint(model, number2evfixe[s in satellites], sum(b_2[s, r] * y[r] for r in 1:length(routes_2)) == 2 * k[s])
    @constraint(model, maxNb2e[s in satellites], sum(b_2_out[s,r] * y[r] for r in 1:length(routes_2)) - nb_vehicle_per_satellite <= 0)
    @constraint(model, capacityMicrohub[s in satellites], sum(b_2_out[s,r] * routes_2[r].load *y[r] for r in 1:length(routes_2)) - capacity_microhub <= 0)

    @objective(model, Min, sum(y[r] * routes_2[r].cost for r in 1:length(routes_2))+
    sum(x[r] * routes_1[r].cost for r in 1:length(routes_1)))

    unset_integer.(x)
    unset_integer.(y)
    unset_integer.(z)
    unset_integer.(k)
    
    optimize!(model)
  
    @assert is_solved_and_feasible(model; dual=true)
    solution_summary(model)
    println("objective value of master problem is: ",value(objective_value(model)))
    println("Status of 1e route: ")
    # for r in 1:length(routes_1)
    #     println("   x[$r]=",value(x[r]))
    # end
    for r in 1:length(routes_1)
        if value(x[r]) != 0
            println("   1e Route ", routes_1[r].sequence, "   Load= ", routes_2[r].load ,  "   Cost= ", round(routes_1[r].cost,digits=2))
        end 
    end
    println("Status of 2e route: ")
    # for r in 1:length(routes_2)
    #     println("   y[$r]=",value(y[r]))
    # end
    for r in 1:length(routes_2)
        if value(y[r]) != 0
            println("   2e Route ", routes_2[r].sequence, "   Load= ", routes_2[r].load , "   Cost= ", round(routes_2[r].cost,digits=2))
            # tc += routes_2[r].cost
        end
    end
    println("Status of satellite: ")
    # for s in 1:length(z)
    #     println("   z[$s]: ", value(z[s+1]))
    # end
    for s in satellites
        if value(z[s]) != 0
            print("   Satellite z[$s] selected,") 
            l = 0
            for r in 1:length(routes_2)
                if b_2_out[s,r] == 1 && value(y[r]) != 0
                    l += routes_2[r].load
                end
            end
            println("  load: $l")
        end
    end 

    # println("   Total 2nd route cost = $tc")

    π1 = collect(dual.(sync1e))
    π2 = collect(dual.(sync2e))
    π3 = collect(dual.(custVisit))
    π4 = collect(dual.(number2evfixe))
    π5 = collect(dual.(maxNb2e))
    π6 = collect(dual.(capacityMicrohub))

    π1 = abs.(vcat(0,π1))
    π2 = abs.(vcat(0,π2))
    π3 = abs.(vcat(zeros(1+length(satellites)),π3))
    π4 = abs.(vcat(0,π4))
    π5 = abs.(vcat(0,π5))
    π6 = abs.(vcat(0,π6))
    
    return π1, π2, π3, π4, π5, π6
end


# ==================================================================================== #

nb_s = 10
nb_m = 4

coor_cust = [[10,35],[10,40],[8,40],[8,45],[5,35]]
demands = vcat(zeros(Int, nb_s + 1) , [5,10,8,5,12])

PI = vcat(ones(nb_m),zeros(nb_s-nb_m))
Random.seed!(42)
shuffle!(PI)
x_coor_parkings = []
y_coor_parkings = []
x_coor_customers = [point[1] for point in coor_cust]
y_coor_customers = [point[2] for point in coor_cust]
min_x,min_y = minimum(x_coor_customers),minimum(y_coor_customers)
max_x,max_y = maximum(x_coor_customers),maximum(y_coor_customers)
for i in 1:nb_s
    push!(x_coor_parkings, rand(min_x: max_x))
    push!(y_coor_parkings, rand(min_y: max_y))
end
coor_parkings = [[x,y] for (x,y) in zip(x_coor_parkings , y_coor_parkings)]
coor = vcat([[0,0]], coor_parkings, coor_cust)
nb_vehicle_per_satellite = 10
capacity_2e_vehicle = 15
capacity_microhub = 50
maximum_duration_2e_vehicle = 30
points = 1:length(demands)


# Define the range of customers
customers = 2+nb_s : length(coor)
satellites = 2:1 + nb_s
arc_cost = calculate_arc_cost(coor)
A2 = 2:length(coor)
A1 = 1:1+length(satellites)
M = 2 * length(customers) # capacity of a satellite
MM = 10000

rs_1 =  [[1,2,1],[1,5,1]]
rs_2 = [[2,12,2], [2,13,2], [2,14,5], [5,15,5], [5,16,5]] 

routes_1 = Vector{Route}()
routes_2 = Vector{Route}()

for r in rs_1
    push!(routes_1, generateRoute(r))
end

for r in rs_2
    push!(routes_2, generateRoute(r))
end

# ==================================================================================== #

a, b_1, b_2, b_2_out = updateParameters()
π1, π2, π3, π4, π5, π6 = runMasterProblem(a, b_1, b_2, b_2_out)
# solve_pricing_1e(π1)


println("Dual price π1= ", π1)
println("Dual price π2= ", π2)
println("Dual price π3= ", π3)
println("Dual price π4= ", π4)
println("Dual price π5= ", π5)
println("Dual price π6= ", π6)

global num_iter = 1
global rc1 =[]
global rc2 =[]

# num_iter, rc1, routes_1 = solve_1e_loop(num_iter, rc1, routes_1)
new_routes, reduced_cost = solve_pricing_labelling_2e(π2, π3, π4, π5, π6)
routes_2 = vcat(routes_2, new_routes)
for rc in reduced_cost
    push!(rc2, rc)
end

a, b_1, b_2, b_2_out = updateParameters()
π1, π2, π3, π4, π5, π6 = runMasterProblem(a, b_1, b_2, b_2_out)
new_routes, reduced_cost = solve_pricing_labelling_2e(π2, π3, π4, π5, π6)
routes_2 = vcat(routes_2, new_routes)
for rc in reduced_cost
    push!(rc2, rc)
end

a, b_1, b_2, b_2_out = updateParameters()
π1, π2, π3, π4, π5, π6 = runMasterProblem(a, b_1, b_2, b_2_out)
new_routes, reduced_cost = solve_pricing_labelling_2e(π2, π3, π4, π5, π6)
routes_2 = vcat(routes_2, new_routes)
for rc in reduced_cost
    push!(rc2, rc)
end

a, b_1, b_2, b_2_out = updateParameters()
π1, π2, π3, π4, π5, π6 = runMasterProblem(a, b_1, b_2, b_2_out)
new_routes, reduced_cost = solve_pricing_1e(π1)
routes_1 = vcat(routes_1, new_routes)
for rc in reduced_cost
    push!(rc1, rc)
end

a, b_1, b_2, b_2_out = updateParameters()
π1, π2, π3, π4, π5, π6 = runMasterProblem(a, b_1, b_2, b_2_out)
new_routes, reduced_cost = solve_pricing_labelling_2e(π2, π3, π4, π5, π6)
routes_2 = vcat(routes_2, new_routes)
for rc in reduced_cost
    push!(rc2, rc)
end

a, b_1, b_2, b_2_out = updateParameters()
π1, π2, π3, π4, π5, π6 = runMasterProblem(a, b_1, b_2, b_2_out)
new_routes, reduced_cost = solve_pricing_labelling_2e(π2, π3, π4, π5, π6)
routes_2 = vcat(routes_2, new_routes)
for rc in reduced_cost
    push!(rc2, rc)
end

println("1e Routes:")
for r in routes_1
    println(r)
end

println("2e Routes:")
for r in routes_2
    println(r)
end

println("")
for rc in rc2
    println(rc)
end