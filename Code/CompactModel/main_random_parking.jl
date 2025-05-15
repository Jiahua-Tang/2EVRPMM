include("Globals.jl")
include("Utiles.jl")
include("DataProcessing.jl")
include("Model.jl")
include("Optimization.jl")

# Read file
println("Number of arguments: ", length(ARGS))
filePath = "../Data/E/E-n33-k4.txt"
case = "random"
global runningTime = 10*60 # minute
global root = "/gpfs/workdir/tangj/2EVRPMM/Benchmark/"

## Input parameter
## FileName / #SEV / Running time
if length(ARGS) >= 1
    global fileName = ARGS[1]
    filePath = "/gpfs/workdir/tangj/2EVRPMM/Benchmark/E/" * fileName
    global V2 = 1 : Int(ARGS[2])
    global runningTime = Int(ARGS[3])
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