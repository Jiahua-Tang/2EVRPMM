using Plots, Random, DataStructures, Combinatorics, Printf, 
    HiGHS, SparseArrays, Test, DataFrames, CPLEX, JuMP, Dates
using Logging, LoggingExtras

include("Utiles.jl")
include("BranchAndPrice/Utiles.jl")
include("BranchAndPrice/branchAndPrice.jl")
include("CompactModel/compactModel.jl")
include("LrpLowerBound/solveLRP.jl")


global root = "$(pwd())/TEST/"
# global root = "/gpfs/workdir/tangj/2EVRPMM/Code/Code/"

instance_size = 15
random_seed = 41
time_stamp = "_"*Dates.format(now(), "ddmmyyHHMM")
file_name = "Outout/output.txt"
file_name = "Output/output_c"*string(instance_size)*"_s"*string(random_seed)*time_stamp*".txt"

open(file_name, "w") do io
    redirect_stdout(io) do

        generateData(instance_size, random_seed)
        println("Minimum 2e routes required: $minimum_2e_vehicle_required")
        # readData("E-n33-k4.txt", ARGS)

        #=========================================================#

        # solveCompactModelDisplayResult() 

        #=========================================================#

        # solveMasterProblem()

        #=========================================================#

        #region B&P: Prep
        global num_iter_global = 1
        global upperBound = Inf
        global optimalSolution = nothing

        global optimal_found_iteration = 0
        global execution_time_total = 0
        global execution_time_branchandprice = 0
        global execution_time_root_node = 0
        global execution_time_child_node = 0
        global execution_time_branching = 0
        global execution_time_column_generation = 0
        global execution_time_set_bound = 0
        global execution_time_test = 0
        global execution_time_pricing = 0
        global execution_time_output = 0
        global execution_time_subproblem = 0
        global execution_time_build_model = 0
        global execution_time_add_columns = 0
        global execution_time_rmp = 0
        global execution_time_filtering = 0
        global deepest_level = 0
        global optimal_found_in = 0
        #endregion

        execution_time_total = @elapsed begin

            generate2eInitialRoutes()
            # lb_lrp_per_route = calculateLRPLowerBoundCG()
            lb_lrp_per_route = calculateLRPLowerBoundByParking()
            displayLRPLowerBound(deepcopy(lb_lrp_per_route))

            #region B&P: Start
            while !isempty(lb_lrp_per_route) # && num_iter_global < 2
                min_value, min_route = findmin(lb_lrp_per_route)
                if min_value > upperBound
                    println("lower bound of subproblem exceed UB, stop algo")
                    break   
                end

                println("")
                solve_branch_and_price_2e_subproblem(min_route)
                delete!(lb_lrp_per_route, min_route)


                @info "current upper bound is $(round(upperBound,digits=2))"
                global num_iter_global
                num_iter_global += 1
            end
        end


        #===============================================================================================#
        # println("\n================================================================")
        # if !isnothing(optimalSolution)
            println("\nTotal Execution time = $(round(execution_time_total, digits=2))")
            println("\ntime spent in soving root node = $(round(execution_time_root_node, digits = 2)), takes percentage of $(round(execution_time_root_node/execution_time_total, digits =2)*100)%")
            println("time spent in branching decision = $(round(execution_time_branching, digits = 2)), takes percentage of $(round(execution_time_branching/execution_time_total, digits =2)*100)%")
            println("time spent in solving child node = $(round(execution_time_child_node, digits = 2)), takes percentage of $(round(execution_time_child_node/execution_time_total, digits =2)*100)%")

            println("\ntime spent in filtering = $(round(execution_time_filtering, digits=2)), takes percentage of $(round(execution_time_filtering/execution_time_total, digits =2)*100)%")
            println("time spent in solving column generation = $(round(execution_time_column_generation, digits=2)), takes percentage of $(round(execution_time_column_generation/execution_time_total,digits=2)*100)%")
            println("   - time spent in building model = $(round(execution_time_build_model, digits=2)), takes percentage of $(round(execution_time_build_model/execution_time_column_generation,digits=2)*100)%")
            println("       - time spent in setting bound = $(round(execution_time_set_bound, digits=2)), takes percentage of $(round(execution_time_set_bound/execution_time_column_generation,digits=2)*100)%")
            println("       - time spent in adding columns = $(round(execution_time_add_columns, digits=2)), takes percentage of $(round(execution_time_add_columns/execution_time_column_generation,digits=2)*100)%")
            println("   - time spent in solving pricing = $(round(execution_time_pricing, digits=2)), takes percentage of $(round(execution_time_pricing/execution_time_column_generation,digits=2)*100)%")
            println("   - time spent in solving RMP = $(round(execution_time_rmp, digits = 2)), takes percentage of $(round(execution_time_rmp/execution_time_column_generation, digits =2)*100)%")
       #     println("   - time spent in output = $(round(execution_time_output, digits = 2)), takes percentage of $(round(execution_time_output/execution_time_column_generation, digits =2)*100)%")

        #     println("\ndeepest node dived to level $deepest_level\n")
            println("Current optimal solution $(round(upperBound, digits=2)) found in interation $optimal_found_iteration in level $optimal_found_in:")
            for route in optimalSolution 
                println(route.sequence, "  ", round(route.cost, digits=2))
            end   
        # end
        #endregion

    end
end
run(`open -a "Visual Studio Code" $file_name`)