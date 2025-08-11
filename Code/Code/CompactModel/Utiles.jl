using Plots.PlotMeasures

function displayMap()

    default(size=(1200, 800))
    # gr()
    ENV["GKSwstype"] = "100" 

    x_coor = [p[1] for p in coor]
    y_coor = [p[2] for p in coor]
    
    # Add the depot point in a different color
    plt = scatter!(x_coor[1:1], y_coor[1:1], 
            markersize = 8, markershape=:square, markercolor = :yellow) 

    # Create a parking scatter plot
    scatter!(plt, x_coor[A2], y_coor[A2], 
            legend = false)      

    # Add the initial parking place in a different color
    for p in A1
        if parking_availability[p]==1
            scatter!(plt, [x_coor[p]], [y_coor[p]], 
                markersize = 8, markercolor = :lightblue)
        else
            scatter!(plt, [x_coor[p]], [y_coor[p]], 
            markersize = 6, markercolor = :white)
        end
    end

    # Create a customers scatter plot
    scatter!(plt, x_coor[customers], y_coor[customers],
                title = "Coordinate Plot",
                legend = false, markersize = 6, markercolor = :pink, 
                marker=:utriangle, markerstrokecolor = :transparent, 
                markerstrokewidth=0, label = "Customers",right_margin=100mm)
    
    return plt
end


function printText(plt, num_y,text::String)
    annotate!(plt, maximum([p[1] for p in coor][customers]) +5,
                                num_y,
                                Plots.text(
                                text,
                                :left,
                                color=:black,
                                6))
    return num_y - 1.5
end


function totalDuration(z, x, itinerary=[])
    # Add the current point to the itinerary
    push!(itinerary, x)
    
    # Check if the vehicle arrives at a parking (assumed to be points in `2:1+np`)
    if x in satellites
        return (0, itinerary)  # Return 0 distance and the itinerary
    end
    
    for k in A2
        if round(value(z[x, k])) == 1
            # Recur to the next point and add the distance
            distance, sub_itinerary = totalDuration(z, k, itinerary)
            return (arc_cost[x, k] + distance, sub_itinerary)
        end
    end
    
    # If no next point is found, return 0 distance and the current itinerary
    return (0, itinerary)
end


function backTracking(z, colorR, x)
    x_coor = [p[1] for p in coor]
    y_coor = [p[2] for p in coor]
    for k in A2
        if round(value(z[x,k])) == 1
            plot!([x_coor[x], x_coor[k]], [y_coor[x], y_coor[k]], line=:arrow, color = colorR)

            cox = (x_coor[x] + x_coor[k]) / 2
            coy = (y_coor[x] + y_coor[k]) / 2  # Use y_coor here
            # annotate!(cox, coy, text(round(distances[x, k], digits=2), :center, 6)) 

            if k in A1   return  end
            backTracking(z, colorR, k)           
            colorR = RGBA(rand(),rand(),rand(),1)
        end
    end
end