using Pkg
# Make sure required packages are available
try; using Coluna; catch; Pkg.add("Coluna"); using Coluna; end
try; using BlockDecomposition; catch; Pkg.add("BlockDecomposition"); using BlockDecomposition; end

include("2evrpColuna.jl")

function test_coluna_implementation()
    """
    Test the Coluna implementation with the E-n33-k4 instance
    """
    
    println("="^60)
    println("Testing 2E-VRP-MM with Coluna")
    println("="^60)
    
    # Read the instance data
    global root = "$(pwd())/TEST/"
    readData("E-n33-k4.txt", String[])
    
    println("Instance loaded:")
    println("  - $(length(customers)) customers")
    println("  - $(length(satellites)) satellites") 
    println("  - Capacity 2e vehicle: $capacity_2e_vehicle")
    println("  - Capacity microhub: $capacity_microhub")
    
    # Test Coluna implementation
    println("\nSolving with Coluna...")
    
    try
        result = solve_2evrp_with_coluna()
        
        if !isnothing(result)
            println("\n" * "="^60)
            println("SUCCESS: Coluna found solution with objective: $(round(result, digits=2))")
            println("="^60)
        else
            println("\n" * "="^60)
            println("WARNING: Coluna did not find a solution")
            println("="^60)
        end
        
        return result
        
    catch e
        println("\n" * "="^60)
        println("ERROR: Exception occurred during optimization:")
        println("  $e")
        if isa(e, MethodError) || isa(e, UndefVarError)
            println("\nStack trace:")
            for (i, frame) in enumerate(stacktrace(catch_backtrace())[1:min(10, end)])
                println("  $i: $frame")
            end
        end
        println("="^60)
        return nothing
    end
end

function compare_implementations()
    """
    Compare Coluna vs your original custom implementation (if available)
    """
    
    println("\n" * "="^60)
    println("Comparing Implementations")
    println("="^60)
    
    # Test Coluna
    coluna_result = test_coluna_implementation()
    
    # TODO: Test original implementation
    # This would require setting up the original branch-and-price call
    # For now, just indicate where the comparison would go
    
    println("\nComparison results:")
    if !isnothing(coluna_result)
        println("  Coluna:    $(round(coluna_result, digits=2))")
        println("  Original:  [Not implemented in this test]")
    else
        println("  Coluna:    Failed")
        println("  Original:  [Not tested]")
    end
end

# Run the test
if abspath(PROGRAM_FILE) == @__FILE__
    compare_implementations()
end
