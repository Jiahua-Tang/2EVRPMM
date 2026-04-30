using JuMP, CPLEX, Dates
include("Utiles.jl")


function solveCompactModelDisplayResult()
    t_start = time()
    model, x, y, t, w, z, f, tau = buildModel()
    build_time = time() - t_start
    displayResult(model, x, y, t, w, z, f, execution_time_limit, tau, t_start, build_time)
end
   
function buildModel() 

    eta1 = 1
    eta2 = 1
    M = 1000000

    TT1 = arc_cost
    TT2 = arc_cost

    model = Model(CPLEX.Optimizer)


    # Decision variable
    @variable(model, x[A1,A1], Bin) #Arc(x,y) traversed by FEV
    @variable(model, y[A1,A1], Bin) #Arc(x,y) traversed by MM
    @variable(model, tau[customers]>=0) #Cumulatedd distance
    @variable(model, t[A2]>=0) #Arrival time
    @variable(model, w[satellites]>=0, Int) #Amount of freight transported from the depot to parking node
    @variable(model, z[A2,A2], Bin) #Arc(x,y) traversed by SEV
    @variable(model, f[A2,A2]>=0,Int) #Load of SEV
    @variable(model, u[satellites]>=0,Int)
    
    for i in A1
        @constraint(model, x[i, i] == 0)
        @constraint(model, y[i, i] == 0)
    end
    for i in A2
        @constraint(model, z[i,i] == 0)
        for j in A2
            if i in satellites && j in satellites
                @constraint(model, z[i,j]==0)
            end
        end
    end

    #======================================================================#
    @objective(model, Min,
        sum(arc_cost[i, j] * x[i, j] for i in A1, j in A1 if i != j) +
        sum(arc_cost[i, j] * z[i, j] for i in A2, j in A2 if i != j) )
    #======================================================================#
    #1 #2
    #Flow conservation at parking for FEV
    @constraint(model, [i in satellites], sum(x[j,i] for j in A1 if i != j) == sum(x[i,j] for j in A1 if i != j))
    @constraint(model, [i in satellites], sum(x[i,j] for j in A1 if i != j) <= 1)
    #3
    #Flow conservation of MM
    @constraint(model, [i in satellites], sum(y[i,j] for j in A1 if i != j) + sum(y[j,i] for j in A1 if i != j)<=1)
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
    @constraint(model, sum(w[p] for p in satellites)<= capacity_1e_vehicle)
    #8
    #Can't tow a MM from parking without MM
    @constraint(model, [i in satellites], sum(y[i,j] for j in A1)<= parking_availability[i])
    #9
    #Can't tow a MM to a parking occupied
    @constraint(model, [j in satellites], sum(y[i,j] for i in A1)<=1-parking_availability[j])
    #10
    #If MM leaves a site, the freight to the site should be zero, otw could be positive
    @constraint(model, [p in satellites], w[p] <= capacity_microhub * (1-sum(y[p,j] for j in A1)))
    #11
    #Link 1st and 2nd
    @constraint(model, [p in satellites], w[p] == sum(f[p,j] for j in A2 if p !=j))
    #12
    #Capacity limit for MM and connection of FEV
    @constraint(model, [p in satellites], w[p] <= capacity_microhub * sum(x[i,p] for i in A1))
    #13
    #Can't distribute from a site without MM
    @constraint(model, [p in satellites], w[p]<= capacity_microhub *(sum(y[i,p] for i in A1)+ parking_availability[p]))

    #14
    #Flow consercvation at parking and customer for SEV
    @constraint(model, [i in A2], sum(z[i,j] for j in A2) == sum(z[j,i] for j in A2))
    #15
    #Flow consercvation at parking node for SEV
    @constraint(model, [p in satellites], sum(z[p,j] for j in A2) <= nb_vehicle_per_satellite)
    #16
    #Each SEV departs from parking node at most once
    #  @constraint(model, [p in P], sum(z[p,j] for j in A2) <= 1)

    #17
    #Flow conservation at customer node for SEV
    @constraint(model, [i in customers], sum(z[i,j] for j in A2) == 1)
    #18
    #Customer demand met
    @constraint(model, [i in customers], sum(f[j,i] for j in A2)-sum(f[i,j] for j in A2) == demands[i])
    #19
    #Connection and capacity limit for SEV
    @constraint(model, [i in A2, j in A2], f[i,j] <= capacity_2e_vehicle * sum(z[i,j]))
    #  #20 #21
    #  #Total working time cannot exceed the length of planning horizon
    #  @constraint(model, sum(arc_cost[i,j]*x[i,j] for i in A1 for j in A1) + eta1*sum(PI[p-1]*x[i,p] for p in P for i in A1)<= zeta)
    #  @constraint(model, [i in C, j in P], t[i]+TT2[i,j]+eta2 <= zeta + M*(1 - z[i,j]))
    eta1 = 0
    eta2 = 0
    #22
    #Time constraint for FEV and MTZ
    @constraint(model, [i in satellites, j in satellites], t[i] + eta1*(1-x[i,j]) + arc_cost[i,j]*x[i,j] <= t[j] + M*(1 - x[i,j]))
    #23
    #Time constraint for SEV and MTZ
    @constraint(model, [i in customers, j in customers], t[i]+eta2*(1-z[i,j])+arc_cost[i,j]*z[i,j] <= t[j]+M * (1 - z[i,j]))
    #24
    @constraint(model, [i in customers], t[i] >= time_window[i][1])
    @constraint(model, [i in customers], t[i] <= time_window[i][2])
    #25 26
    #Arrival time initialization
    # @constraint(model, [i in satellites], arc_cost[1,i] * x[1,i] <= t[i])
    @constraint(model, [i in satellites, j in customers], arc_cost[i,j] * z[i, j] <= t[j])
    # @constraint(model, [p in satellites, j in customers], t[p] + arc_cost[p,j] * z[p,j] <= t[j])

    #27
    # #Max time and duration & MTZ
    @constraint(model, [i in satellites, j in customers], tau[j] + M * (1-z[i,j]) >= arc_cost[i,j])
    @constraint(model, [i in customers, j in customers], tau[i] + arc_cost[i,j] <= tau[j] + M * (1-z[i,j]) )
    @constraint(model, [i in customers, j in satellites], tau[i] + arc_cost[i,j] <= maximum_duration_2e_vehicle + M * (1-z[i,j]) )
    @constraint(model, [i in customers], tau[i] <= maximum_duration_2e_vehicle)
    @constraint(model, [i in satellites,j in satellites], u[i] + 1 <= u[j] + length(A1) * (1-x[i,j]))

    # @constraint(model, sum(distances[i,j]*x[i,j] for i in A1 for j in A1)<=maxDuration1e)

    # optimize!(model)
    # println("Objective value: ", objective_value(model))

    return model, x, y, t, w, z, f, tau
end

function displayResult(model, x, y, t, w, z, f, execution_time_limit, tau, t_start=time(), build_time=0.0)
    # set_silent(model)
    set_optimizer_attribute(model, "CPX_PARAM_TILIM", execution_time_limit)
    set_optimizer_attribute(model, "CPX_PARAM_CLOCKTYPE", 1)
    set_optimizer_attribute(model, "CPX_PARAM_THREADS", 1)
    set_optimizer_attribute(model, "CPX_PARAM_SCRIND", 1)
    total_time = @elapsed optimize!(model)
    total_time_with_build = time() - t_start  # build + optimize, wall-clock
    resultStatus = ""
    currentTime = Dates.format(now(), "dd-mm-yyyy-HH-MM")

    println()
    println("Model build time             : $(round(build_time, digits=3)) seconds")
    println("CPLEX solver time            : $(MOI.get(model, MOI.SolveTimeSec())) seconds")
    println("Total execution time (build+solve): $(round(total_time_with_build, digits=3)) seconds")
    println("Gap: ",MOI.get(model, MOI.RelativeGap()))
    if primal_status(model) == MOI.FEASIBLE_POINT
        println("Total distance traveled: ", objective_value(model))
    end

    println("File name: ", instance_name)
    println("Capacity of FEV: ", capacity_1e_vehicle)
    println("Capacity of Microhub: ", capacity_microhub)
    println("Capacity of SEV: ", capacity_2e_vehicle)
    println("Number of customers: ", nb_customer)
    println("Number of parkings: ", nb_parking)
    println("Number of microhubs: ", nb_microhub)
    println("Number of robots/MM: ", nb_vehicle_per_satellite)
    # println("Parking generation rule: ", parkingGenerationRule)
    # if parkingGenerationRule == "specified"
    #     print("Specified parking location:  ")
    #     for parking in specifiedParkings 
    #         print("$parking  ")
    #     end
    #     println("")
    # end

    # Check solver status and print results
    jobid = get(ENV, "SLURM_JOB_ID", "nojob")
    taskid = get(ENV, "SLURM_PROCID", "0")
    outfile = "./c_$(nb_customer)_result_$(jobid)_$(taskid).csv"

    status_text = ""
    obj_val = try
        objective_value(model)
    catch
        missing
    end

    gap_val = try
        MOI.get(model, MOI.RelativeGap())
    catch
        missing
    end

    solve_time = try
        MOI.get(model, MOI.SolveTimeSec())
    catch
        missing
    end

    if termination_status(model) == MOI.OPTIMAL
        println("Optimal solution found!")
        status_text = "/"
        resultStatus = "-O-" * currentTime * "-"

    elseif primal_status(model) == MOI.FEASIBLE_POINT
        println("Feasible solution found within the time limit!")
        status_text = string(gap_val)
        resultStatus = "-F-" * currentTime * "-"

    else
        println("No feasible solution found.")
        status_text = "no feasible solution"
        resultStatus = "-N-" * currentTime * "-"
    end

    row_data = [
        currentTime,
        "c",
        "\"$instance_name\"",
        length(customers),
        length(satellites),
        sum(parking_availability),
        nb_vehicle_per_satellite,
        total_time_with_build,   # total wall-clock: model build + CPLEX solve
        build_time,              # JuMP model construction time only
        solve_time,               # CPLEX solver internal time only
        obj_val,
        status_text
    ]

    open(outfile, "a") do file
        println(file, join(row_data, ","))
    end

    if !(termination_status(model) == MOI.OPTIMAL || primal_status(model) == MOI.FEASIBLE_POINT)
        return
    end

    #============================================================
    Print solution routes
    ============================================================#
    println("\n", repeat("-", 70))
    println("Solution Routes")
    println(repeat("-", 70))

    # 1st-echelon FEV route (depot = 1)
    println("\n[1st-echelon FEV route]")
    fev_route = [1]
    current = 1
    fev_cost = 0.0
    for step in 1:length(A1)+1
        next_node = nothing
        for j in A1
            if j != current && round(value(x[current, j])) > 0.5
                next_node = j
                break
            end
        end
        if next_node === nothing
            break
        end
        fev_cost += arc_cost[current, next_node]
        push!(fev_route, next_node)
        if next_node == 1
            break
        end
        current = next_node
    end
    println("  Route: ", join(fev_route, " -> "))
    println("  Cost : ", round(fev_cost, digits=2))

    # Freight w[p] delivered to each parking
    println("\n[Freight w[p] delivered to each parking]")
    any_w = false
    for p in satellites
        wp = value(w[p])
        if wp > 1e-6
            println("  w[$p] = ", round(wp, digits=2))
            any_w = true
        end
    end
    if !any_w
        println("  (no freight delivered)")
    end

    # MM towed arcs
    println("\n[MM towed arcs (y[i,j] == 1)]")
    mm_count = 0
    for i in A1, j in A1
        if i != j && round(value(y[i, j])) > 0.5
            println("  $i -> $j")
            mm_count += 1
        end
    end
    if mm_count == 0
        println("  (no MM towed)")
    end

    # 2nd-echelon SEV routes per satellite
    println("\n[2nd-echelon SEV routes]")
    total_sev_cost = 0.0
    for p in satellites
        departures = Int[]
        for j in A2
            if j != p && round(value(z[p, j])) > 0.5
                push!(departures, j)
            end
        end
        if isempty(departures)
            continue
        end
        println("  Satellite $p (#routes = $(length(departures))):")
        for start_cust in departures
            sev_route = [p, start_cust]
            current = start_cust
            sev_load = value(f[p, start_cust])
            sev_cost = arc_cost[p, start_cust]
            for step in 1:length(A2)+1
                next_node = nothing
                for j in A2
                    if j != current && round(value(z[current, j])) > 0.5
                        next_node = j
                        break
                    end
                end
                if next_node === nothing
                    break
                end
                sev_cost += arc_cost[current, next_node]
                push!(sev_route, next_node)
                if next_node in satellites
                    break
                end
                current = next_node
            end
            total_sev_cost += sev_cost
            println("    ", join(sev_route, " -> "),
                    "  | load = ", round(sev_load, digits=2),
                    "  | cost = ", round(sev_cost, digits=2))
        end
    end
    println("\n  Total FEV cost = ", round(fev_cost, digits=2),
            " | Total SEV cost = ", round(total_sev_cost, digits=2),
            " | Sum = ", round(fev_cost + total_sev_cost, digits=2))
    println(repeat("-", 70))

    # New data to append
    # Time / Filename / Cap V1 / Cap MM / Cap V2 / #Parking / #MM / #Robot / Parking generation rule / Limit time / Total Distance / Execution time 
    # row_data = [currentTime, fileName, Q0, Q1, Q2, np, sum(PI), length(V2), case, resultStatus * string(minutes), objective_value(model), total_time]
    # open("../../Result/output.csv", "a") do file
    #     println(file, join(row_data, ",")) 
    # end

    # if primal_status(model) == MOI.FEASIBLE_POINT
    #     light_green = RGBA(0.5, 1.0, 0.5, 1.0)
    #     plt = displayMap()
    #     num_y = maximum([p[2] for p in coor])
    #     num_y =printText(plt, num_y,"File name: "* instance_name)
    #     if termination_status(model) == MOI.OPTIMAL
    #         num_y = printText(plt, num_y, "Optimal solution found!")
    #     else
    #         num_y = printText(plt, num_y, "Feasible solution found within the time limit: $execution_time_limit")
    #     end
    #     num_y = printText(plt, num_y,"Objective: "*string(objective_value(model)))
    #     num_y = printText(plt, num_y,"Execution time: "*string(total_time))
    #     num_y = printText(plt, num_y,"Number of customers: "*string(nb_customer))
    #     num_y = printText(plt, num_y,"Capacity of FEV: "*string(capacity_1e_vehicle))
    #     num_y = printText(plt, num_y,"Capacity of Microhub: "*string(capacity_microhub))
    #     num_y = printText(plt, num_y,"Capacity of SEV: "*string(capacity_2e_vehicle))
    #     num_y = printText(plt, num_y,"Number of parkings: "*string(nb_parking))
    #     num_y = printText(plt, num_y,"Number of microhubs: "*string(sum(parking_availability)))
    #     num_y = printText(plt, num_y,"Number of robots/MM: "*string(nb_vehicle_per_satellite))
    #     # num_y = printText(plt, num_y,"Parking generation rule: "*string(parkingGenerationRule))
    #     # num_y =printText(num_y,"Max duration of SEV: "*string(maxDuration))
    #     num_y = printText(plt, num_y,"")
    #     title!(instance_name)
 
    #     node_labels = [string("N.", i) for i in points]
    #     demand_labels = [string("D= ",demands[i]) for i in customers]
    #     # Time window label
    #     # x
    #     # y
    #     # z
    #     # tau
    #     # f
    #     # w
    #     println()
    #     for i in satellites
    #         for j in customers
    #             if round(value(z[i, j])) == 1
    #                 dis, iti = totalDuration(z, j)
    #                 dis = dis + arc_cost[i, j]
                    
    #                 # Format itinerary as "3 -> 24 -> 25 -> 3"
    #                 formatted_iti = join([i; iti], ", ")
                    
    #                 # Print the formatted output
    #                 num_y = printText(plt, num_y,"$formatted_iti : "*string(round(dis, digits=2)))
    #                 println("$formatted_iti is : ", round(dis, digits=2))
    #             end
    #         end
    #     end

    #     for i in A1
    #         for j in A1
    #             if round(value(x[i,j])!=0)
    #                 println("x[$i $j]=",round(value(x[i,j])))
    #             end
    #         end
    #     end
        
    #     x_coor = [p[1] for p in coor]
    #     y_coor = [p[2] for p in coor]
    #     time_labels = [string("t[$i]= ", round(value(t[i]))) for i in A2] # t
    #     println(time_labels)
    #     distance_labels = [string("d= ", round(value(tau[i]), digits=2)) for i in customers] # tau
    #     for i in points
    #         # Add node number lable
    #         annotate!(plt, x_coor[i], y_coor[i]+0.3, text(node_labels[i], :center, 4))
            
    #         if i in A1
    #             # Add FEV arcs between the locations if they are traversed
    #             for j in A1         
    #                 if round(value(y[i, j])) == 1
    #                     plot!(plt, [x_coor[i], x_coor[j]], [y_coor[i], y_coor[j]],line=:arrow,color = light_green, linealpha=4, lw=4)
    #                 end
    #                 if round(value(x[i, j])) == 1
    #                     plot!(plt, [x_coor[i], x_coor[j]], [y_coor[i], y_coor[j]],line=:arrow,color = :black, lw = 1.5)
    #                 end
    #             end
    #             # Add SEV arcs between the locations if they are traverse
    #             if i in satellites
    #                 colorR = RGBA(rand(),rand(),rand(),1)
    #                 backTracking(z, colorR, i)
    #             end
    #         end
            
    #         if i in A2
    #             # Add arriving time lable
    #             # annotate!(x_coor[i]+1.5, y_coor[i]-0.3, text(time_labels[i-1], :center, 4))
    #             if i in customers
    #                 # Add customer demand lable
    #                 # annotate!(x_coor[i]-0.3, y_coor[i]-0.3, text(demand_labels[i-1-np], :center, 6)) 
    #                 # Add distance lable
    #                 # annotate!(plt, x_coor[i]+1.5, y_coor[i]-0.3, text(distance_labels[i-np-1], :center, 4))
    #             end 
    #         end
    #     end     
    # end

    # display(plt)

    # result_path_svg = root * "Result/Fig/" * fileName * "-" * parkingGenerationRule * resultStatus * "result.svg"
    # result_path_png = root * "Result/Fig/" * fileName * "-" * parkingGenerationRule * resultStatus * "result.png"

    # result_path_svg = root * "ResultCPLEX/svg/" * instance_name * resultStatus * "result.svg"
    # result_path_png = root * "ResultCPLEX/png/" * instance_name * resultStatus * "result.png"
    # println(result_path_svg)
    
    # savefig(result_path_svg)
    # savefig(result_path_png)

end
