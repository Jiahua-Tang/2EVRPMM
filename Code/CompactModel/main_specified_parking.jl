include("Globals.jl")
include("Utiles.jl")
include("DataProcessing.jl")
include("Model.jl")
include("Optimization.jl")

# Read file
println("Number of arguments: ", length(ARGS))
global case = "specified"
global runningTime = 10*60 # minute
# global root = "/gpfs/workdir/tangj/2EVRPMM/Benchmark/"
global root = "./../../Benchmark/"

## Input parameter
## FileName / #SEV / Running time / Specified parking
if length(ARGS) >= 1
    global fileName = ARGS[1]
    filePath = root * "Data/" * fileName
    global V2 = 1 : parse(Int, ARGS[2])
    global runningTime = parse(Int, ARGS[3])
    global specifiedParkings = Vector{Int}()
    for i in 4:length(ARGS) 
        push!(specifiedParkings, parse(Int, ARGS[i]))
    end
    global np = length(specifiedParkings) * 2
else
    println("No arguments provided")
end


println("\n", "File path: ", filePath)
println("Execution time limit: ",runningTime)

# display(specifiedParkings)

# Process data
dataProcessing(case, filePath)

# plt = displayMap()
# # display(plt)
# # displayData()

# Build model
model, x, y, t, w, z, f, tau = buildModel()

# Run model, note result
resolve(model, x, y, t, w, z, f, 15, tau)

nothing