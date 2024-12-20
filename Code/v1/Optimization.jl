include("Globals.jl")
include("Utiles.jl")
include("Model.jl")
using Dates
using JuMP, CPLEX

function resolve(model, x, y, t, w, z, f, minutes)
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
        # f
        # w

        # for i in P
        #     println("Show max duration of point $i: ", totalDuration(z, i, 0))
        # end


        # for i in P
        #     for j in C
        #         if round(value(z[i,j]))==1
        #             dis = totalDuration(z,j) + distances[i,j]
        #             println("total distance of point $i to $j is : ", dis)
        #             println()
        #         end
        #     end
        # end
        
        
        time_labels = [string("t= ", round(value(t[i]))) for i in A2] # t
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
                annotate!(x_coor[i]+1.5, y_coor[i]-0.3, text(time_labels[i-1], :center, 4))
                if i in C
                    # Add customer demand lable
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
