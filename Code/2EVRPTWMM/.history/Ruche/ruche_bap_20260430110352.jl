using Plots, Random, DataStructures, Combinatorics, Printf, 
    HiGHS, SparseArrays, Test, DataFrames, CPLEX, JuMP, Dates, Base.Threads, CPUTime
using Logging, LoggingExtras

include("../Utiles.jl")
include("../BranchAndPrice/Utiles.jl")
include("../BranchAndPrice/branchAndPrice.jl")
include("../CompactModel/compactModel.jl")
include("../LrpLowerBound/solveLRP.jl")


# global root = "/gpfs/workdir/tangj/2EVRPMM/Code/Code/"

time_stamp = "_"*Dates.format(now(), "ddmmyy_HHMM")
global root = "$(pwd())/../../Data/Instances/"

filename = ARGS[1]

# num_cust = parse(Int, ARGS[1])
# global random_seed = parse(Int, ARGS[2])
# name_diff = parse(Int, ARGS[3])
# file_name = "Output/bp_c$(num_cust)"*"s"*string(random_seed)*time_stamp*"_"*string(name_diff)*".txt"


file_name = "Output/bp_"*filename*"_"*time_stamp*".txt"

const TIME_LIMIT = 3600*3

# file_name = "Output/demo.txt"
mkpath(dirname(file_name))

open(file_name, "w") do io
    redirect_stdout(io) do
#            # generateData(instance_size, random_seed)
            # read_Solomon_Dataset_TW("../../Data/Demo/100/" * fileName * ".txt", 1200)
            # retrieve_solomon_random_data("../../Data/Demo/100/" * fileName * ".txt", 3600, num_cust)

            read_nico_dataset("../../Data/Instances/Data/"*filename*".txt")
            
            println("\n================================================================")

            global execution_time_total = @time @CPUtime begin
                start_time = time()
                execution_time = @elapsed lrp_subproblems = preparation_branch_and_price()
                println("execution time of preparation for branch-and-price : $(round(execution_time, digits=2)) seconds")
                println("\n $(repeat("=", 70))")

                #region : create model and initial columns
                execution_time = @elapsed begin
                    global model = Model(CPLEX.Optimizer)
                    set_silent(model)
                    set_optimizer_attribute(model, "CPXPARAM_Threads", 1)
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

                for route in routes_2e
                    add_2eroute!(route)
                end
                #endregion

                execution_time_cg_subproblem = 0.0
                execution_time_bap = 0.0

                global num_iter = 1
                global num_iter_global = 1
                batch_size = 3
                stop_processing = false

                root_nodes = PriorityQueue()

                while length(root_nodes) < batch_size && !isempty(lrp_subproblems)
                    top = peek(lrp_subproblems)
                    subproblem = top.first
                    lb = top.second
                    if lb >= upperBound
                        println("\nSubproblem $num_iter lower bound exceeds global optimal solution, finish precompiling\n")
                        stop_processing = true
                        break
                    end
                    dequeue!(lrp_subproblems)
                    execution_time_root_node = @elapsed root_result = solve_root_node(subproblem)
                    execution_time_cg_subproblem += execution_time_root_node
                    println("execution time solving subproblem $num_iter : $(round(execution_time_root_node, digits=2)) seconds")
                    if !isnothing(root_result)
                        enqueue!(root_nodes, Pair(subproblem, root_result), root_result[1].cgLowerBound)
                    end
                    num_iter += 1
                end

                if isempty(root_nodes)
                    continue
                end

                println("\n--- Branch-and-price batch (size = $(length(root_nodes))) ---")
                # ---------- 2) Run branch-and-price on each root node in the batch ----------
                bap_batch_time = @elapsed begin
                    while !isempty(root_nodes)
                        if time_exceeded()
                            println("\n Time limit reached during branch-and-price.")
                            stop_processing = true
                            break
                        end
                        top = peek(root_nodes)
                        k = top.first
                        v = top.second
                        dequeue!(root_nodes)
                        if v < upperBound
                            println("\n[BAP $num_iter_global] ", k[1].sequence, " : ", v)
                            solve_branch_and_price_2e_subproblem(k[1], k[2])
                        else
                            println("\nSubproblem lower bound exceeds global optimal solution")
                            stop_processing = true
                            break
                        end
                        global num_iter_global += 1
                    end
                end
                execution_time_bap += bap_batch_time
            end

                # for (subproblem, lb) in lrp_subproblems
                #     println(subproblem.sequence, "   ", round(lb, digits=2))
                # end

                # global num_iter = 1
                # for (subproblem, lb) in lrp_subproblems
                #     if lb < upperBound  && num_iter <= 3
                #         # println(subproblem.sequence,"   ",round(lb,digits=2),"   ",round(upperBound,digits=2))
                #         execution_time_root_node = @elapsed root_result = solve_root_node(dequeue!(lrp_subproblems))
                #         println("execution time solving subproblem : $(round(execution_time_root_node, digits=2)) seconds")
                #         if !isnothing(root_result)
                #             enqueue!(root_nodes, Pair(subproblem, root_result), root_result[1].cgLowerBound)
                #         end
                #     else
                #         println("\nSubproblem lower bound exceeds global optimal solution, finish precompiling")
                #         break
                #     end
                #     num_iter += 1
                # end

                # println("\nCurrent optimal value $upperBound\nLeft 2e subproblems :")
                # for (k, v) in root_nodes
                #     if time_exceeded()
                #         println("\n Time limit reached during branch-and-price.")
                #         break
                #     end
                #     if v < upperBound
                #         println(k[1].sequence, " : ",v, "\n")
                #         solve_branch_and_price_2e_subproblem(k[1], k[2])
                #     else
                #         println("\nSubproblem lower bound exceeds global optimal solution")
                #         break
                #     end
                # end


            println("\n================================================================")
            # println("\nTotal Execution time = $(round(execution_time_total, digits=2))")


            if !isnothing(optimalSolution)
                currentTime = Dates.format(now(), "dd-mm-yyyy-HH-MM-SS-s")

                jobid = get(ENV, "SLURM_JOB_ID", "nojob")
                outfile = "bp_$(filename)_result_$jobid.csv"

                row_data = [
                    currentTime,
                    "bp",
                    "\"$filename\"",
                    length(customers),
                    length(satellites),
                    sum(parking_availability),
                    nb_vehicle_per_satellite,
                    time() - start_time,
                    upperBound,
                    "/"
                ]

                open(outfile, "a") do file
                    println(file, join(row_data, ","))
                end
            else
                currentTime = Dates.format(now(), "dd-mm-yyyy-HH-MM-SS-s")
                jobid = get(ENV, "SLURM_JOB_ID", "nojob")
                outfile = "bp_$(filename)_result_$jobid.csv"

                row_data = [
                    currentTime,
                    "bp",
                    "\"$filename\"",
                    length(customers),
                    length(satellites),
                    sum(parking_availability),
                    nb_vehicle_per_satellite,
                    time() - start_time,
                    "/"
                ]

                open(outfile, "a") do file
                    println(file, join(row_data, ","))
                end
            end
                # println("\nTotal Execution time = $(round(execution_time_total, digits=2))")
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
        #         println("Current optimal solution $(round(upperBound, digits=2)) found in interation $optimal_found_iteration in level $optimal_found_in:")
        #         for route in optimalSolution 
        #             println(route.sequence, "  ", round(route.cost, digits=2))
        #         end   
            # end
            #endregion
    end
    end
end

# run(`open -a "Visual Studio Code" $file_name`)