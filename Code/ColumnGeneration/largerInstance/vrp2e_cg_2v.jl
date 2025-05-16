include("../../3index/Utiles.jl")
using JuMP, CPLEX
import DataFrames
import HiGHS
import Plots
import SparseArrays
import Test  #src

function calculate_arc_cost(points::Vector{Vector{Int}})
    num_points = length(points)
    arc_cost = Array{Float64}(undef, num_points, num_points)
    println("PI= $PI")
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


function getA(customers,routes_2)
    # Initialize a 2D array for a[i][r] with zeros
    a = zeros(Int, length(customers), length(routes_2))

    # Populate the array
    for (i_idx, i) in enumerate(customers)  # Loop over points (row index)
        for (r, route) in enumerate(routes_2)  # Loop over routes (column index)
            a[i_idx, r] = i in route.sequence ? 1 : 0
        end
    end
    return a
end

function getInfo()
    # x_coor_customers, y_coor_customers, x_coor_depot, y_coor_depot, demands
    # println(@__DIR__)
    return  readFile("/Users/lenovo1/Library/CloudStorage/OneDrive-EcoledeManagementdeNormandie/2EVRPMM/Data/Demo/C101-30.txt")

end

function getB1(satellites,routes_1)
    # Initialize a 2D array for b[s][r] with zeros
    b = zeros(Int, length(satellites))
    i = 2
    while i<=length(routes_1) 
        # println("add line")
        b = hcat(b,zeros(Int, length(satellites)))
        # println("$i: $b")
        i += 1
    end
    # Populate the array
    for (s_idx, i) in enumerate(satellites)  # Loop over points (row index)
        for (r, route) in enumerate(routes_1)  # Loop over routes (column index)
            b[s_idx, r] = i in route.sequence ? 1 : 0
        end
    end
    return b
end

function getB2(satellites,routes_2)
    # Initialize a 2D array for b[s][r] with zeros
    b = zeros(Int, length(satellites))
    i = 2
    while i <= length(routes_2) 
        # println("add line")
        # println("$i : $b")
        b = hcat(b,zeros(Int, length(satellites)))        
        i += 1 
    end
    # Populate the array
    for (s_idx, i) in enumerate(satellites)  # Loop over points (row index)
        for (r, route) in enumerate(routes_2)  # Loop over routes (column index)
            b[s_idx, r] = i in route.sequence ? 1 : 0
        end
    end
    return b
end


function updateParameters()
    ## Update parameters
    # println("a[i][r]:")
    a = getA(customers,routes_2)
    # println("   ",a)

    # println("b1[s][r]: number of 1e route: ",length(routes_1))
    b_1= getB1(satellites,routes_1)
    # println("   ",b_1)
    # println("b2[s][r]: number of 2e route: ", length(routes_2))
    b_2 = getB2(satellites,routes_2)
    # println("   ",b_2,"\n")

    return a, b_1, b_2
end

function transformRoute(x)
    # for i in A1
    #     for j in A1
    #         println("x[$i $j]=",round(value(x[i,j])))
    #     end
    # end
    new_route = Vector{Int}()
    current_node = 0
    if length(x) > (1+length(satellites))^2 
        ## Transform a 2e route
        ## Find the starting satellite
        println("Work on a 2nd route")
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
                # println(round(value(x[current_node,j])))
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
    return new_route
end

function runMasterProblem(a, b_1, b_2)
    
    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, x[1:length(routes_1)]>=0, Int)
    @variable(model, y[1:length(routes_2)]>=0, Int)
    
    @constraint(model, sync[s in satellites], M * sum(b_1[s-1,r] * x[r] for r in 1:length(routes_1))- sum(b_2[s-1,r] * y[r] for r in 1:length(routes_2)) >= 0 )
    @constraint(model, demandCons[i in customers], sum(a[i-1-length(satellites),r] * y[r] for r in 1:length(routes_2)) == 1 )
    # @constraint(model, totalVeh, sum(y[r] for r in 1:length(routes))<= nb_veh)
    @objective(model, Min, sum(y[r] * routes_2[r].cost for r in 1:length(routes_2))+
        sum(x[r] * routes_1[r].cost for r in 1:length(routes_1)))
    
    unset_integer.(x)
    unset_integer.(y)
    
    optimize!(model)

    @assert is_solved_and_feasible(model; dual=true)
    solution_summary(model)
    println("objective value of master problem is: ",value(objective_value(model)))

    for r in 1:length(routes_1)
        # if !(value(x[r])==1)
            println("   1e Route ", routes_1[r].sequence, " x= ", round(value(x[r]),digits=2), "   Cost= ", round(routes_1[r].cost, digits=2))
        # end
    end
    tc = 0
    println("")
    for r in 1:length(routes_2)
        if round(value(y[r]))==1
            println("   2e Route ", routes_2[r].sequence, "   Cost= ", round(routes_2[r].cost, digits=2))
            tc += routes_2[r].cost
        end
    end
    println("   Total 2nd route cost = $tc")

    π1 = collect(dual.(sync))
    π2 = collect(dual.(demandCons))
    π1 = vcat(0,π1,zeros(length(customers)))
    π2 = vcat(zeros(1+length(satellites)),π2)
    # println("π1= ", π1)
    # println("π2= ", π2)
    return π1, π2
end

function solve_pricing_1e(π)
    println("π1= ",π)
    ## Construct model
    model = Model(CPLEX.Optimizer)

    # set_silent(model)
    @variable(model, x1[A1,A1]>=0,Bin)
    @variable(model, u[A1]>=0, Int)

    # @constraint(model, satelliteOblg, sum(x1[i,j] for i in A1 for j in subSatellites)>=1)
    @constraint(model, depart, sum(x1[1,i] for i in satellites) ==1)
    @constraint(model, ending, sum(x1[i,1] for i in satellites) ==1)

    @constraint(model, flow[i in satellites], sum(x1[j,i] for j in A1)
                                        == sum(x1[i,j] for j in A1))
    # @constraint(model, flow2[i in satellites], sum(x1[j,i] for j in A1) == 1)
    @constraint(model, subtourElm[i in satellites, j in satellites], u[i] + arc_cost[i,j] * x1[i,j] <= u[j] + MM *(1-x1[i,j]))

   
    @objective(model, Min, sum(arc_cost[i,j] * x1[i,j] for i in A1 for j in A1)
                                - M * sum(π[s] * sum(x1[s,j] for j in A1) for s in satellites) )
    set_optimizer_attribute(model, "CPX_PARAM_TILIM", 120)
    optimize!(model)

    println("Subprobem 1e: Reduced cost= ", objective_value(model))
    for i in 1:length(A1)
        for j in 1:length(A1)
            if round(value(x1[i,j]))==1
                println("   x1[$i,$j]=",round(value(x1[i,j]),digits=2))
            end
        end
    end
    new_route = nothing
    # threshold = isapprox(-1e-8, 0.0; atol=1e-10) # Specify the tolerance
    if objective_value(model) < threshold
        # Reduced cost negative, new column generated and added to master problem
        # println("test  : $num_iter")
        new_route = transformRoute(x1)
    end
    return new_route, objective_value(model)
end

struct Label 
    rc::Float64
    cap::Float64
    cost::Float64
    s::Vector{Int}
end
function solve_1e_labelling()
    
end

function solve_pricing_2e(π1,π2)

    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, y1[A2, A2]>=0, Int)
    @variable(model, u[customers]>=0,Int)
    @objective(model, Min, sum((arc_cost[i,j] - π2[i]) * y1[i,j] for i in A2 for j in A2)+
        sum(π1[s]*y1[s,j] for s in satellites for j in customers))

    @constraint(model, flowSatelliteOut, sum(y1[i,j] for i in satellites, j in customers) ==1 )
    @constraint(model, flowSatelliteIn, sum(y1[i,j] for i in customers, j in satellites) ==1 )
    @constraint(model, flowPoints[i in customers], sum(y1[i,j] for j in A2) == sum(y1[j,i] for j in A2))
    @constraint(model, capacityVehicle, sum(demands[i] * y1[i,j] for i in customers for j in A2) <= v_capacity)
    @constraint(model, subtourElm[i in customers, j in customers], u[i] + arc_cost[i,j] * y1[i,j] <= u[j] + MM *(1-y1[i,j]))

    optimize!(model)

    println("Subprobem 2e: Reduced cost= ", objective_value(model))
    new_route = nothing
    if objective_value(model) < threshold
        # Reduced cost negative, new column generated and added to master problem
        new_route = transformRoute(y1) 
    end
    # for i in 1:length(π1)
    #     print("   $i: ", round(π1[i], digits=2))
    # end
    # println("")
    # for i in 1:length(π2)
    #     print("   $i: ", round(π2[i], digits=2))
    # end
    println("")
    for i in A2
        for j in A2
            if round(value(y1[i,j]))==1
                println("   y1[$i, $j]=1")
            end
        end
    end
    return new_route,objective_value(model)
end


global threshold = -1e-8

function xyxy()
    num_iter = 1
    rc1 = []
    rc2 = []
    status1 = false
    status2 = false

    while !(status1==true && status2==true) && num_iter < 65
        println("\n======================Iteration $num_iter======================")
        a,b_1, b_2 = updateParameters()
        π1, π2 = runMasterProblem(a,b_1,b_2)
        new_route = nothing
        if !status1
            new_route,rc_v = solve_pricing_1e(π1)
            if !(isempty(rc1)) && rc_v == rc1[end]
                @info "Reduced cost converged, terminating column generation for 1st echelon"
                status1 = true
            end
            push!(rc1,rc_v)
            if new_route == nothing
                @info "No new 1e routes generated"
                status1 = true
            else
                @info "New 1e route $new_route "
                push!(routes_1, generateRoute(new_route))
            end
        else
            @info "1st echelon terminates"
        end

        if !status2
            new_route = nothing
            new_route,rc_v = solve_pricing_2e(π1, π2)
            # if !(isempty(rc2)) && rc_v == rc2[end]
            #     @info "Reduced cost converged, terminating column generation for 2nd echelon"
            #     status2 = true
            # end
            push!(rc2,rc_v)
            if new_route == nothing
                @info "No new 2e routes generated"
                status2 = true
            else
                @info "New 2e route $new_route "
                push!(routes_2, generateRoute(new_route))
            end
        end
        num_iter += 1

    end
    return rc1, rc2
end

function xxxy()
    num_iter = 1
    rc1 = []
    rc2 = []
    status1 = false
    status2 = false
    while !(status1) && num_iter < 6
        println("\n======================Iteration $num_iter======================")
        a,b_1, b_2 = updateParameters()
        π1, π2 = runMasterProblem(a,b_1,b_2)
        new_route = nothing
        new_route,rc_v = solve_pricing_1e(π1)
        if !(isempty(rc1)) && rc_v == rc1[end]
            @info "Reduced cost converged, terminating column generation for 1st echelon"
            status1 = true
        end
        push!(rc1,rc_v)
        if new_route == nothing
            @info "No new 1e routes generated"
            status1 = true
        else
            @info "New 1e route $new_route "
            push!(routes_1, generateRoute(new_route))
        end
        num_iter += 1
    end

    while !(status2) && num_iter < 65
        println("\n======================Iteration $num_iter======================")
        a,b_1, b_2 = updateParameters()
        π1, π2 = runMasterProblem(a,b_1,b_2)

        new_route = nothing
        new_route,rc_v = solve_pricing_2e(π1, π2)
        # if !(isempty(rc2)) && rc_v == rc2[end]
        #     @info "Reduced cost converged, terminating column generation for 2nd echelon"
        #     status2 = true
        # end
        push!(rc2,rc_v)
        if new_route == nothing
            @info "No new 2e routes generated"
            status2 = true
        else
            @info "New 2e route $new_route "
            push!(routes_2, generateRoute(new_route))
        end
        num_iter += 1
    end
    return rc1, rc2
end

function yyyx()
    num_iter = 1
    rc1 = []
    rc2 = []
    status1 = false
    status2 = false

    while !(status2) && num_iter < 65
        println("\n======================Iteration $num_iter======================")
        a,b_1, b_2 = updateParameters()
        π1, π2 = runMasterProblem(a,b_1,b_2)

        new_route = nothing
        new_route,rc_v = solve_pricing_2e(π1, π2)
        # if !(isempty(rc2)) && rc_v == rc2[end]
        #     @info "Reduced cost converged, terminating column generation for 2nd echelon"
        #     status2 = true
        # end
        
        if new_route == nothing
            @info "No new 2e routes generated"
            status2 = true
        elseif !(rc2 ==[]) && rc_v ==  rc2[end]
            @info "Reduced cost of second echelon subproblem converged"
            status2 = true
        else
            @info "New 2e route $new_route "
            push!(routes_2, generateRoute(new_route))
        end
        push!(rc2,rc_v)
        num_iter += 1
    end
    
    while !(status1) && num_iter < 65
        println("\n======================Iteration $num_iter======================")
        a,b_1, b_2 = updateParameters()
        π1,_ = runMasterProblem(a,b_1,b_2)
        new_route = nothing
        new_route,rc_v = solve_pricing_1e(π1)
        # if !(isempty(rc1)) && rc_v == rc1[end]
        #     @info "Reduced cost converged, terminating column generation for 1st echelon"
        #     status1 = true
        # end
        push!(rc1,rc_v)
        if new_route == nothing
            @info "No new 1e routes generated"
            status1 = true
        else
            @info "New 1e route $new_route "
            push!(routes_1, generateRoute(new_route))
        end
        num_iter += 1
    end

    a,b_1, b_2 = updateParameters()
    π1, π2 = runMasterProblem(a,b_1,b_2)

    new_route = nothing
    new_route,rc_v = solve_pricing_2e(π1, π2)
    
    return rc1, rc2, num_iter

end

v_capacity = 50
x_coor_customers, y_coor_customers, x_coor_depot, y_coor_depot, demands = getInfo()
# println(demands)

nb_s = 10
nb_m = 4

points = 1:length(coor)
customers = 2+nb_s:length(coor)
satellites = 2:1+nb_s
A2 = 2:length(coor)
A1 = 1:1+length(satellites)

demands = vcat(zeros(nb_s+1),demands)
Random.seed!(42)
PI = vcat(ones(nb_m),zeros(nb_s-nb_m))
shuffle!(PI)
x_coor_parkings = []
y_coor_parkings = []
min_x,min_y = minimum(x_coor_customers),minimum(y_coor_customers)
max_x,max_y = maximum(x_coor_customers),maximum(y_coor_customers)

for i in 1:nb_s
    push!(x_coor_parkings, rand(min_x: max_x))
    push!(y_coor_parkings, rand(min_y: max_y))
end

coor_x = vcat(x_coor_depot,x_coor_parkings,x_coor_customers)
coor_y = vcat(y_coor_depot,y_coor_parkings,y_coor_customers)
coor = [[x, y] for (x, y) in zip(coor_x, coor_y)]
# nb_veh = 3
arc_cost = calculate_arc_cost(coor)

M = length(customers) # capacity of a satellite
MM = 10000
gr()
scatter(x_coor_depot, y_coor_depot,marker=:square,legned=false)
scatter!(x_coor_customers, y_coor_customers,marker=:utriangle,legned=false)

# Separate points into two groups: occupied (PI==1) and free (PI==0)
x_yes = x_coor_parkings[PI .== 1]
y_yes = y_coor_parkings[PI .== 1]

x_no = x_coor_parkings[PI .== 0]
y_no = y_coor_parkings[PI .== 0]

# Plot points where PI[i] == 1 (occupied) in blue
scatter!(x_yes, y_yes, marker=:square, markercolor=:purple, legend=false)

# Plot points where PI[i] == 0 (free) in white with a black border
scatter!(x_no, y_no, marker=:square, markercolor=:white, legend=false)

## Initial solution generation
rs_1 = []
rs_2 = [] 
p_ocp = findall(x -> x == 1, PI).+ 1
for i in p_ocp
    push!(rs_1,vcat(1,i,1))
end
for i in customers 
    push!(rs_2, vcat(rand(p_ocp),i,rand(p_ocp)))
end

routes_1 = Vector{Route}()
routes_2 = Vector{Route}()
for r in rs_1
    push!(routes_1, generateRoute(r))
end
push!(rs_1,vcat(1,p_ocp,1))
for r in rs_2
    push!(routes_2, generateRoute(r))
end

function solve_2e_loop(num_iter, rc2, routes_2)
    status = false

    while !(status) 
        println("\n======================Iteration $num_iter======================")
        a,b_1, b_2 = updateParameters()
        π1,π2 = runMasterProblem(a,b_1,b_2)
        new_route = nothing
        new_route,rc_v = solve_pricing_2e(π1,π2)
        if !(isempty(rc2)) && rc_v == rc2[end]
            @info "Reduced cost converged, terminating column generation for 1st echelon"
            status = true
        end
        push!(rc2,rc_v)
        if new_route == nothing
            @info "No new 1e routes generated"
            status = true
        else
            @info "New 1e route $new_route "
            push!(routes_2, generateRoute(new_route))
        end
        num_iter += 1
    end
    return num_iter, rc2, routes_2
end
function solve_1e_loop(num_iter, rc1, routes_1)
    status = false

    while !(status) 
        println("\n======================Iteration $num_iter======================")
        a,b_1, b_2 = updateParameters()
        π1,_ = runMasterProblem(a,b_1,b_2)
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

rc_1 = []
rc_2 = []
num_iter = 1
num_iter, rc_2, routes_2 = solve_2e_loop(num_iter, rc_2, routes_2)
num_iter, rc_1, routes_1 = solve_1e_loop(num_iter, rc_1, routes_1)
num_iter, rc_2, routes_2 = solve_2e_loop(num_iter, rc_2, routes_2)
num_iter, rc_1, routes_1 = solve_1e_loop(num_iter, rc_1, routes_1)
num_iter, rc_2, routes_2 = solve_2e_loop(num_iter, rc_2, routes_2)
num_iter, rc_1, routes_1 = solve_1e_loop(num_iter, rc_1, routes_1)
num_iter, rc_2, routes_2 = solve_2e_loop(num_iter, rc_2, routes_2)
num_iter, rc_1, routes_1 = solve_1e_loop(num_iter, rc_1, routes_1)
num_iter, rc_2, routes_2 = solve_2e_loop(num_iter, rc_2, routes_2)
num_iter, rc_1, routes_1 = solve_1e_loop(num_iter, rc_1, routes_1)
num_iter, rc_2, routes_2 = solve_2e_loop(num_iter, rc_2, routes_2)
num_iter, rc_1, routes_1 = solve_1e_loop(num_iter, rc_1, routes_1)

println("Route 1e")
for r in routes_1
    println("   ",r)
end
println("Route 2e")
for r in routes_2
    println("   ",r)
end
println("Reduced cost 1e")
for v in rc_1
    println("   ",v)
end
println("Reduced cost 2e")
for v in rc_2
    println("   ",v)
end