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

# Branching mode, optional 2nd argument:
#   "full"       (default) full branching cascade + best-bound/gradient node selection
#   "random_dfs" random customer pair from the most fractional route + depth-first search
#   "baseline"   fixed microhubs (no relocation), arc-based 1e TSP; 2e routes,
#                branching and node selection as in "full"
run_mode = length(ARGS) >= 2 ? ARGS[2] : "full"
run_mode in ("full", "random_dfs", "baseline") || error("unknown mode '$run_mode' (use full, random_dfs or baseline)")
global random_pair_branching = run_mode == "random_dfs"
global dfs_node_selection = run_mode == "random_dfs"
global fixed_microhub_baseline = run_mode == "baseline"
branching_mode = random_pair_branching ? "random_pair" : "full"
search_mode = dfs_node_selection ? "dfs" : "best_bound"

# num_cust = parse(Int, ARGS[1])
# global random_seed = parse(Int, ARGS[2])
# name_diff = parse(Int, ARGS[3])
# file_name = "Output/bp_c$(num_cust)"*"s"*string(random_seed)*time_stamp*"_"*string(name_diff)*".txt"


file_name = "Output/bp_"*filename*"_"*run_mode*"_"*time_stamp*".txt"
global status_debug = false

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
                global start_time = time()
                # Same clock and limit for the checks inside the B&P tree and CG.
                global bap_start_time = start_time
                global bap_time_limit = TIME_LIMIT
                time_exceeded() = time_limit_exceeded()
                global jobid  = get(ENV, "SLURM_JOB_ID", "nojob")
                global outfile = "bp_$(filename)_$(run_mode)_result_$jobid.csv"
                execution_time_prep = @elapsed lrp_subproblems = preparation_branch_and_price()

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

                    global satelliteRouteLB = Dict{Int, ConstraintRef}()
                    global satelliteRouteUB = Dict{Int, ConstraintRef}()
                    for s in satellites
                        satelliteRouteLB[s] = @constraint(model, 0 <= 0)
                        satelliteRouteUB[s] = @constraint(model, 0 <= nb_vehicle_per_satellite)
                    end
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
                time_limit_reached = false

                # Loop over all LRP subproblems, batch by batch: pull up to
                # `batch_size` subproblems, solve their root nodes, then run
                # branch-and-price on each before pulling the next batch.
                # Subproblems are sorted by LRP lower bound, so the whole run
                # stops once the next one's bound reaches the incumbent.
                while !isempty(lrp_subproblems) && !stop_processing
                    if time_exceeded()
                        println("\n Time limit reached before next batch.")
                        time_limit_reached = true
                        break
                    end

                    root_nodes = PriorityQueue()

                    while length(root_nodes) < batch_size && !isempty(lrp_subproblems)
                        if time_exceeded()
                            println("\n Time limit reached while solving root nodes.")
                            stop_processing = true
                            time_limit_reached = true
                            break
                        end
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

                    if !isempty(root_nodes)
                        println("\n--- Branch-and-price batch (size = $(length(root_nodes))) ---")
                        # ---------- 2) Run branch-and-price on each root node in the batch ----------
                        bap_batch_time = @elapsed begin
                            while !isempty(root_nodes)
                                if time_exceeded()
                                    println("\n Time limit reached during branch-and-price.")
                                    stop_processing = true
                                    time_limit_reached = true
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
                                    # Skip only this subproblem: later subproblems
                                    # can still have a lower root bound.
                                    println("\nSubproblem lower bound exceeds global optimal solution, skip it")
                                end
                                global num_iter_global += 1
                            end
                        end
                        execution_time_bap += bap_batch_time
                    end
                end # while: next batch of subproblems
                # The limit may have been hit inside a tree or CG loop of the
                # last subproblem, which the checks above never see.
                time_limit_reached = time_limit_reached || time_limit_hit
            end

            total_time          = time() - start_time
            build_model_time    = execution_time_build_model               # master + all subproblem JuMP construction
            time_excluding_build = total_time - build_model_time            # algorithm/solver work without model building

            println("total execution time of preparation for sorting subproblems   : $(round(execution_time_prep, digits=2)) seconds")
            println("total execution time of column generation solving subproblems : ", round(execution_time_cg_subproblem, digits=2), " seconds")
            println("total execution time solving branch and price                 : ", round(execution_time_bap, digits=2), " seconds")
            println("total number of branch-and-price nodes explored               : ", total_bap_nodes)
            println("total model construction time (all JuMP builds)               : $(round(build_model_time, digits=2)) seconds")
            println("total execution time (build + algo)                           : $(round(total_time, digits=2)) seconds")
            println("total execution time excluding model building                 : $(round(time_excluding_build, digits=2)) seconds")

            println("\n================================================================")


            currentTime = Dates.format(now(), "dd-mm-yyyy-HH-MM-SS-s")

            obj_val   = !isnothing(optimalSolution) ? upperBound : "/"
            status_tx = "/"

            row_data = [
                currentTime,
                fixed_microhub_baseline ? "bl_bp" : "bp",
                "\"$filename\"",
                length(customers),
                length(satellites),
                sum(parking_availability),
                nb_vehicle_per_satellite,
                total_time,             # full wall-clock time: build + algo (compare with compact's total_time_with_build)
                build_model_time,       # JuMP model construction time only (compare with compact's build_time)
                time_excluding_build,   # algorithm + CPLEX solver time, no JuMP build
                execution_time_prep,    # LRP preparation (subproblem sorting)
                execution_time_cg_subproblem,  # column generation at root nodes
                execution_time_bap,     # branch-and-price
                obj_val,
                status_tx,
                total_bap_nodes,        # total number of B&P nodes explored
                branching_mode,         # "full" or "random_pair"
                search_mode             # "best_bound" or "dfs"
            ]
            if length(customers) >= 75 && !time_limit_reached
                push!(row_data, "Opt.")
            end

            open(outfile, "w") do file
                println(file, join(row_data, ","))
            end
    end
end

# run(`open -a "Visual Studio Code" $file_name`)