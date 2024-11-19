include("Globals.jl")
include("Utiles.jl")
include("Model.jl")
using Dates
using JuMP, CPLEX
using Plots.PlotMeasures 

function resolve(model, x, y, t, w, z, f, minutes,tau)
    set_optimizer_attribute(model, "CPX_PARAM_TILIM", 60 * minutes)
    total_time = @elapsed optimize!(model)
    resultStatus = ""
    currentTime = Dates.format(now(), "dd-mm-yyyy HH:MM")
    
    println()
    println("Total execution time: $total_time seconds")
    if primal_status(model) == MOI.FEASIBLE_POINT
        println("Total distance traveled: ", objective_value(model))
    end
    println("File name: ", fileName)
    println("Number of customers: ", nc)
    println("Capacity of FEV: ", Q0)
    println("Capacity of Microhub: ", Q1)
    println("Capacity of SEV: ", Q2)
    println("Number of parkings: ", np)
    println("Number of microhubs: ", sum(PI))
    println("Number of robots/MM: ", length(V2))
    println("Parking generation rule: ", case)

    # Check solver status and print results
    if termination_status(model) == MOI.OPTIMAL
        println("Optimal solution found!")
        resultStatus = "-O"
    elseif primal_status(model) == MOI.FEASIBLE_POINT
        println("Feasible solution found within the time limit!")
        resultStatus = "-F"
    else
        println("No feasible solution found.")
        # New data to append
        # Time / Filename / Cap V1 / Cap MM / Cap V2 / #Parking / #MM / #Robot / Parking generation rule / Limit time / Result 
        row_data = [currentTime, fileName, Q0, Q1, Q2, np, sum(PI), length(V2), case, minutes, "No feasible solution found"]
        open("../../Result/output.csv", "a") do file
            println(file, join(row_data, ",")) 
        end
        return
    end

    # New data to append
    # Time / Filename / Cap V1 / Cap MM / Cap V2 / #Parking / #MM / #Robot / Parking generation rule / Limit time / Total Distance / Execution time 
    row_data = [currentTime, fileName, Q0, Q1, Q2, np, sum(PI), length(V2), case, resultStatus * string(minutes), objective_value(model), total_time]
    open("../../Result/output.csv", "a") do file
        println(file, join(row_data, ",")) 
    end

    if primal_status(model) == MOI.FEASIBLE_POINT
        light_green = RGBA(0.5, 1.0, 0.5, 1.0)
        displayMap()
        num_y = maximum(y_coor)
        num_y =printText(num_y,"File name: "*fileName)
        if termination_status(model) == MOI.OPTIMAL
            num_y = printText(num_y,"Optimal solution found!")
        else
            num_y = printText(num_y,"Feasible solution found within the time limit: $runningTime")
        end
        num_y =printText(num_y,"Objective: "*objective_value(model))
        num_y =printText(num_y,"Execution time: "*string(total_time))
        num_y =printText(num_y,"Number of customers: "*string(nc))
        num_y =printText(num_y,"Capacity of FEV: "*string(Q0))
        num_y =printText(num_y,"Capacity of Microhub: "*string(Q1))
        num_y =printText(num_y,"Capacity of SEV: "*string(Q2))
        num_y =printText(num_y,"Number of parkings: "*string(np))
        num_y =printText(num_y,"Number of microhubs: "*string(sum(PI)))
        num_y =printText(num_y,"Number of robots/MM: "*string(length(V2)))
        num_y =printText(num_y,"Parking generation rule: "*string(case))
        # num_y =printText(num_y,"Max duration of SEV: "*string(maxDuration))
        
        title!(fileName)
        P = 2 : np+1 #Set of parking place
        C = np+2 : np+nc+1 #Set of customers
        A1 = 1 : 1+np #Set of FE arcs
        A2 = 2 : 1+np+nc #Set of SE arcs
        N = 1:np+nc+1 #Set of nodes
        
        node_labels = [string("N.", i) for i in N]
        demand_labels = [string("D= ",demands[i-np-1]) for i in C]
        # Time window label
        # x
        # y
        # z
        # tau
        # f
        # w
        println()
        for i in P
            for j in C
                if round(value(z[i, j])) == 1
                    dis, iti = totalDuration(z, j)
                    dis = dis + distances[i, j]
                    
                    # Format itinerary as "3 -> 24 -> 25 -> 3"
                    formatted_iti = join([i; iti], " -> ")
                    
                    # Print the formatted output
                    num_y = printText(num_y,"total distance of $formatted_iti is : "*string(round(dis, digits=2)))
                    println("total distance of $formatted_iti is : ", round(dis, digits=2))
                end
            end
        end
        
        
        time_labels = [string("t= ", round(value(t[i]))) for i in A2] # t
        distance_labels = [string("d= ", round(value(tau[i]), digits=2)) for i in C] # tau
        for i in N
            # Add node number lable
            annotate!(x_coor[i], y_coor[i]+0.3, text(node_labels[i], :center, 4))
            
            if i in A1
                # Add FEV arcs between the locations if they are traversed
                for j in A1         
                    if round(value(y[i, j])) == 1
                        plot!([x_coor[i], x_coor[j]], [y_coor[i], y_coor[j]],line=:arrow,color = light_green, linealpha=4, lw=4)
                    end
                    if round(value(x[i, j])) == 1
                        plot!([x_coor[i], x_coor[j]], [y_coor[i], y_coor[j]],line=:arrow,color = :black,linestyle=:dash)
                    end
                end
                # Add SEV arcs between the locations if they are traverse
                if i in P
                    colorR = RGBA(rand(),rand(),rand(),1)
                    backTracking(z, colorR, i)
                end
            end
            
            if i in A2
                # Add arriving time lable
                # annotate!(x_coor[i]+1.5, y_coor[i]-0.3, text(time_labels[i-1], :center, 4))
                if i in C
                    # Add customer demand lable
                    annotate!(x_coor[i]+1.5, y_coor[i]-0.3, text(distance_labels[i-np-1], :center, 4))
                    # annotate!(x_coor[i]-0.3, y_coor[i]-0.3, text(demand_labels[i-1-np], :center, 6)) 
                end 
            end
        end     
    end

    result_path_svg = savePath * resultStatus * string(minutes) * "min-result.svg"
    result_path_png = savePath * resultStatus * string(minutes) * "min-result.png"
    
    savefig(result_path_svg)
    savefig(result_path_png)

end
