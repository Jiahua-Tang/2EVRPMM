using JuMP, CPLEX, Dates
include("Utiles.jl")

function solveCompactModel()
    model, _, _, _, _, _, _ = buildModel()
    set_silent(model)
    optimize!(model)
    primal_status(model) == MOI.FEASIBLE_POINT && println("Objective value: ", objective_value(model))
end

function solveCompactModelDisplayResult()
    model, x, y, t, w, z, f, tau = buildModel()
    displayResult(model, x, y, t, w, z, f, execution_time_limit, tau)
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
    @variable(model,tau[customers]>=0) #Cumulatedd distance
    @variable(model, t[A2]>=0, Int) #Arrival time
    @variable(model, w[satellites]>=0, Int) #Amount of freight transported from the depot to parking node
    @variable(model, z[A2,A2], Bin) #Arc(x,y) traversed by SEV
    @variable(model, f[A2,A2]>=0,Int) #Load of SEV
    
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
        sum(arc_cost[i, j] * z[i, j] for i in A2, j in A2 if i != j))
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
    #  @constraint(model, sum(TT1[i,j]*x[i,j] for i in A1 for j in A1) + eta1*sum(PI[p-1]*x[i,p] for p in P for i in A1)<= zeta)
    #  @constraint(model, [i in C, j in P], t[i]+TT2[i,j]+eta2 <= zeta + M*(1 - z[i,j]))
    #22
    #Time constraint for FEV and MTZ
    @constraint(model, [i in satellites, j in satellites], t[i] + eta1*(1-x[i,j]) + TT1[i,j]*x[i,j] <= t[j] + M*(1 - x[i,j]))
    #23
    #Time constraint for SEV and MTZ
    @constraint(model, [i in customers, j in customers], t[i]+eta2*(1-z[i,j])+TT2[i,j]*z[i,j] <= t[j]+M * (1 - z[i,j]))
    #  #24
    #  @constraint(model, [i in C], t[i] >= time_windows[i-1-np][1])
    #  @constraint(model, [i in C], t[i] <= time_windows[i-1-np][2])
    #25 26
    #Arrival time initialization
    @constraint(model, [i in satellites], TT1[1,i] * x[1,i] <= t[i])
    @constraint(model, [p in satellites, j in customers], t[p] + TT2[p,j] * z[p,j] <= t[j])

    #27
    # #Max duration & MTZ
    @constraint(model, [i in satellites, j in customers], tau[j] + M * (1-z[i,j]) >= arc_cost[i,j]  )
    @constraint(model, [i in customers, j in customers], tau[i] + arc_cost[i,j] <= tau[j] + M * (1-z[i,j]) )
    @constraint(model, [i in customers, j in satellites], tau[i] + arc_cost[i,j] <= maximum_duration_2e_vehicle + M * (1-z[i,j]) )
    @constraint(model, [i in customers], tau[i] <= maximum_duration_2e_vehicle)

    # @constraint(model, sum(distances[i,j]*x[i,j] for i in A1 for j in A1)<=maxDuration1e)

    # optimize!(model)
    # println("Objective value: ", objective_value(model))

    return model, x, y, t, w, z, f, tau
end

function displayResult(model, x, y, t, w, z, f, minutes,tau)
    set_silent(model)
    set_optimizer_attribute(model, "CPX_PARAM_TILIM", execution_time_limit)
    total_time = @elapsed optimize!(model)
    resultStatus = ""
    currentTime = Dates.format(now(), "dd-mm-yyyy-HH-MM")
    
    println()
    println("Total execution time: $total_time seconds")
    if primal_status(model) == MOI.FEASIBLE_POINT
        println("Total distance traveled: ", objective_value(model))
    end

    println("File name: ", fileName)
    println("Capacity of FEV: ", capacity_1e_vehicle)
    println("Capacity of Microhub: ", capacity_microhub)
    println("Capacity of SEV: ", capacity_2e_vehicle)
    println("Number of customers: ", nb_customer)
    println("Number of parkings: ", nb_parking)
    println("Number of microhubs: ", nb_microhub)
    println("Number of robots/MM: ", nb_vehicle_per_satellite)
    println("Parking generation rule: ", parkingGenerationRule)
    if parkingGenerationRule == "specified"
        print("Specified parking location:  ")
        for parking in specifiedParkings 
            print("$parking  ")
        end
        println("")
    end

    # Check solver status and print results
    if termination_status(model) == MOI.OPTIMAL
        println("Optimal solution found!")
        resultStatus = "-O-" * currentTime * "-"
    elseif primal_status(model) == MOI.FEASIBLE_POINT
        println("Feasible solution found within the time limit!")
        resultStatus = "-F-" * currentTime * "-"
    else
        println("No feasible solution found.")
        # # New data to append
        # # Time / Filename / Cap V1 / Cap MM / Cap V2 / #Parking / #MM / #Robot / Parking generation rule / Limit time / Result 
        # row_data = [currentTime, fileName, Q0, Q1, Q2, np, sum(PI), length(V2), case, minutes, "No feasible solution found"]
        # open("../../Result/output.csv", "a") do file
        #     println(file, join(row_data, ",")) 
        # end
        return
    end

    # New data to append
    # Time / Filename / Cap V1 / Cap MM / Cap V2 / #Parking / #MM / #Robot / Parking generation rule / Limit time / Total Distance / Execution time 
    # row_data = [currentTime, fileName, Q0, Q1, Q2, np, sum(PI), length(V2), case, resultStatus * string(minutes), objective_value(model), total_time]
    # open("../../Result/output.csv", "a") do file
    #     println(file, join(row_data, ",")) 
    # end

    if primal_status(model) == MOI.FEASIBLE_POINT
        light_green = RGBA(0.5, 1.0, 0.5, 1.0)
        plt = displayMap()
        num_y = maximum([p[2] for p in coor])
        num_y =printText(plt, num_y,"File name: "*fileName)
        if termination_status(model) == MOI.OPTIMAL
            num_y = printText(plt, num_y, "Optimal solution found!")
        else
            num_y = printText(plt, num_y, "Feasible solution found within the time limit: $runningTime")
        end
        num_y = printText(plt, num_y,"Objective: "*string(objective_value(model)))
        num_y = printText(plt, num_y,"Execution time: "*string(total_time))
        num_y = printText(plt, num_y,"Number of customers: "*string(nb_customer))
        num_y = printText(plt, num_y,"Capacity of FEV: "*string(capacity_1e_vehicle))
        num_y = printText(plt, num_y,"Capacity of Microhub: "*string(capacity_microhub))
        num_y = printText(plt, num_y,"Capacity of SEV: "*string(capacity_2e_vehicle))
        num_y = printText(plt, num_y,"Number of parkings: "*string(nb_parking))
        num_y = printText(plt, num_y,"Number of microhubs: "*string(sum(parking_availability)))
        num_y = printText(plt, num_y,"Number of robots/MM: "*string(nb_vehicle_per_satellite))
        num_y = printText(plt, num_y,"Parking generation rule: "*string(parkingGenerationRule))
        # num_y =printText(num_y,"Max duration of SEV: "*string(maxDuration))
        num_y = printText(plt, num_y,"")
        title!(fileName)
 
        node_labels = [string("N.", i) for i in points]
        demand_labels = [string("D= ",demands[i]) for i in customers]
        # Time window label
        # x
        # y
        # z
        # tau
        # f
        # w
        println()
        for i in satellites
            for j in customers
                if round(value(z[i, j])) == 1
                    dis, iti = totalDuration(z, j)
                    dis = dis + arc_cost[i, j]
                    
                    # Format itinerary as "3 -> 24 -> 25 -> 3"
                    formatted_iti = join([i; iti], " -> ")
                    
                    # Print the formatted output
                    num_y = printText(plt, num_y,"$formatted_iti : "*string(round(dis, digits=2)))
                    println("$formatted_iti is : ", round(dis, digits=2))
                end
            end
        end
        
        x_coor = [p[1] for p in coor]
        y_coor = [p[2] for p in coor]
        time_labels = [string("t= ", round(value(t[i]))) for i in A2] # t
        distance_labels = [string("d= ", round(value(tau[i]), digits=2)) for i in customers] # tau
        for i in points
            # Add node number lable
            annotate!(plt, x_coor[i], y_coor[i]+0.3, text(node_labels[i], :center, 4))
            
            if i in A1
                # Add FEV arcs between the locations if they are traversed
                for j in A1         
                    if round(value(y[i, j])) == 1
                        plot!(plt, [x_coor[i], x_coor[j]], [y_coor[i], y_coor[j]],line=:arrow,color = light_green, linealpha=4, lw=4)
                    end
                    if round(value(x[i, j])) == 1
                        plot!(plt, [x_coor[i], x_coor[j]], [y_coor[i], y_coor[j]],line=:arrow,color = :black, lw = 1.5)
                    end
                end
                # Add SEV arcs between the locations if they are traverse
                if i in satellites
                    colorR = RGBA(rand(),rand(),rand(),1)
                    backTracking(z, colorR, i)
                end
            end
            
            if i in A2
                # Add arriving time lable
                # annotate!(x_coor[i]+1.5, y_coor[i]-0.3, text(time_labels[i-1], :center, 4))
                if i in customers
                    # Add customer demand lable
                    # annotate!(x_coor[i]-0.3, y_coor[i]-0.3, text(demand_labels[i-1-np], :center, 6)) 
                    # Add distance lable
                    # annotate!(plt, x_coor[i]+1.5, y_coor[i]-0.3, text(distance_labels[i-np-1], :center, 4))
                end 
            end
        end     
    end

    display(plt)

    result_path_svg = root * "Result/Fig/" * fileName * "-" * parkingGenerationRule * resultStatus * "result.svg"
    result_path_png = root * "Result/Fig/" * fileName * "-" * parkingGenerationRule * resultStatus * "result.png"
    println(result_path_svg)
    
    savefig(result_path_svg)
    savefig(result_path_png)

end
