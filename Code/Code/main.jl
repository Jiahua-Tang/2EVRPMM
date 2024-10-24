include("Globals.jl")
include("Utiles.jl")
include("DataProcessing.jl")
include("Optimization.jl")

using .Globals
using .Utiles


println("Number of arguments: ", length(ARGS))

filePath = "../Data/Demo/C101-10.txt" # use in command
# filePath = "Data/Demo/C101-20.txt" # use in VSCode
case = "f"
Q1 = 600
Q2 = 100
runningTime = 10

if length(ARGS) >= 1
    if length(ARGS[1])>1 filePath = "../Data/Demo/" * ARGS[1]     end
    if length(ARGS) >= 2
        if readArgument(ARGS[2]) != 0 Q1 = readArgument(ARGS[2])  end
        if length(ARGS) >= 3
            if readArgument(ARGS[3]) != 0 Q2 = readArgument(ARGS[3])  end
            if length(ARGS) >= 4
                if readArgument(ARGS[4]) != 0 runningTime = readArgument(ARGS[4])  end
                if length(ARGS) >= 5
                    if ARGS[5] == "r"||"f" case = ARGS[5]  end
                end
            end
        end
    end
else
    println("No arguments provided")
end

println("\n", "File path: ", filePath)
println("Capacity of FEV: ",Q1)
println("Capacity of SEV: ",Q2)
println("Execution time limit: ",runningTime)
println("Case: ",case,"\n")


# model = runModel(filePath, Q1,Q2,runningTime,case) 