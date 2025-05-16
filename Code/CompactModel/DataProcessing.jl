include("Globals.jl")
include("Utiles.jl")
using Plots
using FileIO

# Target of this module is to read the data file, and final output are:
#           distance matrix
#           demand
#           save data figure


function dataProcessing(case::String, filePath::String)
    # Read coordinates of customers and depot, Read customers demand
    result = readFileElison(filePath)

    x_coor_customers, y_coor_customers = result[1], result[2]
    x_coor_depot, y_coor_depot = result[3], result[4]

    global PI 
    if case == "random"
        x_coor_parkings, y_coor_parkings, PI = randomGenerateParking(x_coor_customers, y_coor_customers)
    else
        x_coor_parkings, y_coor_parkings, PI = fixedGenerateParking(x_coor_customers, y_coor_customers)
    end
    global nc = length(x_coor_customers)
    # global np = length(x_coor_parkings)

    global demands = vcat(zeros(Int, 1+np), result[5])   
    global Q0 = sum(demands)
    global Q2 = result[6]
    global Q1 = 4 * Q2
    # global V2 = 1 : 5
    global x_coor = vcat(x_coor_depot, x_coor_parkings, x_coor_customers)
    global y_coor = vcat(y_coor_depot, y_coor_parkings, y_coor_customers)

    for (idx, value) in enumerate(PI) 
        println("PI[$idx] = $value")
    end

    for (idx, _) in enumerate(x_coor)
        if idx in 1:np+1
            println("$idx  [$(round(x_coor[idx],digits=2))  $(round(y_coor[idx],digits=2))]   $(PI[idx] == 1 ? "occupied" : "empty")")
        else
            println("$idx  [$(round(x_coor[idx],digits=2))  $(round(y_coor[idx],digits=2))]   $(demands[idx])")
        end
    end

    # Disatance matrix
    global distances = calculate_distance_matrix(x_coor, y_coor, nc)

end

function displayData()
    
    displayMap()

    node_labels = [string("N.", i) for i in 1:1+nc+np]
    demand_labels = [string("D= ",demands[i-1]) for i in 2:1+nc]
#        tw_labels = [string(time_windows[i-1-np]) for i in C]       

    for i in 1:1+nc+np
        annotate!(x_coor[i], y_coor[i]+0.3, text(node_labels[i], :center, 6))
        if i in 2:1+nc
            annotate!(x_coor[i]-0.3, y_coor[i]-0.3, text(demand_labels[i-1], :center, 6)) 
            # annotate!(x_coor[i]-0.3, y_coor[i]-2, text(tw_labels[i-1-np], :center, 4)) 
        end 
    end 

    # Extract the file name (without extension) and create the desired fileName
    baseName = splitext(basename(filePath))[1]  # This extracts the base name without extension, e.g., C101-10
    global fileName = baseName * "-Q" * string(Q2) *"-" *case # e.g., C101-10-Q100

    # Create the save directory, avoiding duplication of the fileName inside the path
    save_dir = joinpath("../../Result", baseName,fileName)  # Only use baseName for the directory structure
    println(save_dir)
    # Paths for saving files
    global savePath = joinpath(save_dir, fileName)
    data_path_svg = savePath * "-data.svg"
    data_path_png = savePath * "-data.png"


    # Check if the directory exists, if not, create it (use mkpath instead of mkdir)
    if !isdir(save_dir)
        mkpath(save_dir)  # mkpath will create all necessary directories along the path
    end

    # Print paths to verify correctness
    println("SVG path: ", data_path_svg)
    println("PNG path: ", data_path_png)
    savefig(data_path_png)
    savefig(data_path_svg)    

end