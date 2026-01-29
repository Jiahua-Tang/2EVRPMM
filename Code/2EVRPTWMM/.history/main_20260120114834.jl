using Plots, Random, DataStructures, Combinatorics, Printf, 
    HiGHS, SparseArrays, Test, DataFrames, CPLEX, JuMP, Dates, Base.Threads, CPUTime
using Logging, LoggingExtras

include("Utiles.jl")
include("BranchAndPrice/Utiles.jl")
include("BranchAndPrice/branchAndPrice.jl")
include("CompactModel/compactModel.jl")
include("LrpLowerBound/solveLRP.jl")


global root = "$(pwd())/TEST/"
# global root = "/gpfs/workdir/tangj/2EVRPMM/Code/Code/"


# instance_size = 70
global random_seed = 42
# # time_stamp = "_"*Dates.format(now(), "ddmmyyHHMM")
# file_name = "Output/S$(random_seed)/v2.2"*"_s"*string(random_seed)*time_stamp*".txt"
file_name = "Output/demo.txt"
mkpath(dirname(file_name))

open(file_name, "w") do io
    redirect_stdout(io) do

        # redirect_stderr(io) do

#            # generateData(instance_size, random_seed)
            global fileName = "R103"
            # read_Solomon_Dataset_TW("../../Data/Demo/100/" * fileName * ".txt", 1200)
            retrieve_solomon_random_data("../../Data/Demo/100/" * fileName * ".txt", 1200, 25)
            println("\n================================================================")

#             #=========================================================#

            # solveCompactModelDisplayResult()

#             #=========================================================#


            global execution_time_total = @time @CPUtime begin
                lrp_subproblems = preparation_branch_and_price()

                println("\n================================================================")
                #region : create model and initial columns
                execution_time = @elapsed begin
                    global model = Model(CPLEX.Optimizer)
                    set_silent(model)
                    set_optimizer_attribute(model, "CPX_PARAM_THREADS", 4)
                    # set_optimizer_attribute(model, "CPXPARAM_MIP_Display", 0)

                    global y_vars = Dict{Int, VariableRef}()

                    @objective(model, Min, 0.0)

                    global sync = Vector{ConstraintRef}(undef, length(satellites))
                    for (k,_) in enumerate(satellites)
                        sync[k] = @constraint(model, -nb_vehicle_per_satellite <= 0.0)
                    end

                    global custVisit = Vector{ConstraintRef}(undef, length(customers))
                    for (k,_) in enumerate(customers) 
                        custVisit[k] = @constraint(model, 1.0 <= 0.0)
                    end

                    global number2evfixe = Vector{ConstraintRef}(undef, length(satellites))
                    for (k,_) in enumerate(satellites)
                        number2evfixe[k] = @constraint(model, 0.0 == 0.0)
                    end

                    global maxVolumnMM = Vector{ConstraintRef}(undef, length(satellites))
                    for (k,_) in enumerate(satellites) 
                        maxVolumnMM[k] = @constraint(model, -capacity_microhub <= 0.0)
                    end

                    global lower_bound_2e_routes = minimum_2e_vehicle_required
                    global upper_bound_2e_routes = nb_parking * nb_vehicle_per_satellite

                    global globalLowerBound = @constraint(model, 0 <= -minimum_2e_vehicle_required) 
                    global globalUpperBound = @constraint(model, 0 <= upper_bound_2e_routes)
                end
                global execution_time_build_model += execution_time

                for (route,_) in enumerate(routes_2e)
                    add_2eroute!(route)
                end
                #endregion

                root_nodes = PriorityQueue()
                for (subproblem, lb) in lrp_subproblems
                    println(subproblem.sequence, "   ", round(lb, digits=2))
                end
                execution_time_cg_subproblem = @elapsed begin
                    for (subproblem, lb) in lrp_subproblems
                        if lb < upperBound
                            # println(subproblem.sequence,"   ",round(lb,digits=2),"   ",round(upperBound,digits=2))
                            dequeue!(lrp_subproblems)
                            execution_time_subproblem_root_node = @elapsed begin
                                root_result = solve_root_node(subproblem)
                            end
                            println("execution time solving subproblem : $(round(execution_time_subproblem_root_node, digits=2)) seconds")
                            if !isnothing(root_result)
                                enqueue!(root_nodes, Pair(subproblem, root_result), root_result[1].cgLowerBound)
                            end
                        else
                            println("\nSubproblem lower bound exceeds global optimal solution, finish precompiling\n")
                            break
                        end
                    end
                end
                # println("total execution time of column generation solving subproblems : ", round(execution_time_cg_subproblem,digits=2)," seconds")
                
                
                println("\n================================================================")
                println("\nCurrent optimal value $upperBound\nLeft 2e subproblems :")
                execution_time_bap = @elapsed begin
                    for (k, v) in root_nodes
                        if v < upperBound
                            println(k[1].sequence, " : ",v, "\n")
                            solve_branch_and_price_2e_subproblem(k[1], k[2])
                        else
                            println("\nSubproblem lower bound exceeds global optimal solution")
                            break
                        end
                    end
                end
                # println("total execution time solving branch and price : ", round(execution_time_bap,digits=2)," seconds")
            end


            println("\n================================================================")
            # println("\nTotal Execution time = $(round(execution_time_total, digits=2))")

            if !isnothing(optimalSolution)
                # println("\nTotal Execution time = $(round(execution_time_total, digits=2)) seconds")
        #         println("\ntime spent in soving root node = $(round(execution_time_root_node, digits = 2)), takes percentage of $(round(execution_time_root_node/execution_time_total, digits =2)*100)%")
        #         println("time spent in branching decision = $(round(execution_time_branching, digits = 2)), takes percentage of $(round(execution_time_branching/execution_time_total, digits =2)*100)%")
        #         println("time spent in solving child node = $(round(execution_time_child_node, digits = 2)), takes percentage of $(round(execution_time_child_node/execution_time_total, digits =2)*100)%")

        #         println("\ntime spent in filtering = $(round(execution_time_filtering, digits=2)), takes percentage of $(round(execution_time_filtering/execution_time_total, digits =2)*100)%")
        #         println("time spent in solving column generation = $(round(execution_time_column_generation, digits=2)), takes percentage of $(round(execution_time_column_generation/execution_time_total,digits=2)*100)%")
        #         println("   - time spent in building model = $(round(execution_time_build_model, digits=2)), takes percentage of $(round(execution_time_build_model/execution_time_column_generation,digits=2)*100)%")
        #         println("       - time spent in setting bound = $(round(execution_time_set_bound, digits=2)), takes percentage of $(round(execution_time_set_bound/execution_time_column_generation,digits=2)*100)%")
        #         println("       - time spent in adding columns = $(round(execution_time_add_columns, digits=2)), takes percentage of $(round(execution_time_add_columns/execution_time_column_generation,digits=2)*100)%")
        #         println("   - time spent in solving pricing = $(round(execution_time_pricing, digits=2)), takes percentage of $(round(execution_time_pricing/execution_time_column_generation,digits=2)*100)%")
        #         println("   - time spent in solving RMP = $(round(execution_time_rmp, digits = 2)), takes percentage of $(round(execution_time_rmp/execution_time_column_generation, digits =2)*100)%")
        # #     println("   - time spent in output = $(round(execution_time_output, digits = 2)), takes percentage of $(round(execution_time_output/execution_time_column_generation, digits =2)*100)%")

        #     #     println("\ndeepest node dived to level $deepest_level\n")
                println("Current optimal solution $(round(upperBound, digits=2))")# found in interation $optimal_found_iteration in level $optimal_found_in:")
                for route in optimalSolution 
                    println(route.sequence, "  ", round(route.cost, digits=2))
                end   
            end
            #endregion
    # end
    end
end

run(`open -a "Visual Studio Code" $file_name`)