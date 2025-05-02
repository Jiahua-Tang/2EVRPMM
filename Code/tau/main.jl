include("Globals.jl")
include("Utiles.jl")
include("DataProcessing.jl")
include("Model.jl")
include("Optimization.jl")

# Read file
println("Number of arguments: ", length(ARGS))
filePath = "/Users/lenovo1/Library/CloudStorage/OneDrive-EcoledeManagementdeNormandie/2EVRPMM/Data/Demo/C101-70.txt"
case = "r"
runningTime = 20 # minute

if length(ARGS) >= 1
    if length(ARGS[1])>1 filePath = "../../Data/" * ARGS[1]     end
    if length(ARGS) >= 2
        if readArgument(ARGS[2]) != 0 runningTime = readArgument(ARGS[2])  end
    end
else
    println("No arguments provided")
end


println("\n", "File path: ", filePath)
println("Execution time limit: ",runningTime)

# Process data
dataProcessing(case, "")
displayData()

# Build model
model, x, y, t, w, z, f, tau = buildModel()

# Run model, note result
resolve(model, x, y, t, w, z, f, runningTime, tau)

println("\nrsync -avz tangj@129.175.109.2:/gpfs/users/tangj/2EVRPMM/Result  ~/Downloads/")