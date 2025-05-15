include("Globals.jl")
include("Utiles.jl")
include("DataProcessing.jl")
include("Model.jl")
include("Optimization.jl")

# Read file
println("Number of arguments: ", length(ARGS))
filePath = "../Data/E/E-n33-k4.txt"
case = "specified"
global runningTime = 10 # minute

## Input parameter
## FileName / #SEV / Specified parking / Running time
if length(ARGS) >= 1
    if length(ARGS[1])>1 filePath = "../../Data/Demo/" * ARGS[1]     end
    if length(ARGS) >= 2
        if readArgument(ARGS[2]) != 0 global runningTime = readArgument(ARGS[2])  end
    end
else
    println("No arguments provided")
end


println("\n", "File path: ", filePath)
println("Execution time limit: ",runningTime)

# Process data
dataProcessing(case, filePath)

plt = displayMap()
# display(plt)
# displayData()

# Build model
model, x, y, t, w, z, f, tau = buildModel()

# Run model, note result
resolve(model, x, y, t, w, z, f, 15, tau)

nothing