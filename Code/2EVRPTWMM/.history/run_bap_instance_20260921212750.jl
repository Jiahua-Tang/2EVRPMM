using Plots, Random, DataStructures, Combinatorics, Printf,
    HiGHS, SparseArrays, Test, DataFrames, CPLEX, JuMP, Dates, Base.Threads, CPUTime
using Logging, LoggingExtras

include("Utiles.jl")
include("BranchAndPrice/Utiles.jl")
include("BranchAndPrice/branchAndPrice.jl")
include("CompactModel/compactModel.jl")
include("LrpLowerBound/solveLRP.jl")

# ============================================================
# Single-instance branch-and-price runner, parameterized by
# instance name via ARGS[1]. Used by run_all_15customer.sh to
# batch-run every 15-customer instance (cx-x,x,15) locally and
# collect the total number of B&P nodes explored per instance
# into a shared summary CSV.
#
# Usage: julia run_bap_instance.jl <instance_name>
#   e.g. julia run_bap_instance.jl ce1-3,5,15
# ============================================================

global root = "$(pwd())/../../Data/Instances/"

const TIME_LIMIT = 3600*3
filename = ARGS[1]
file_name = "Output/bap_$(filename).txt"
mkpath(dirname(file_name))
global status_debug = true

summary_csv = "Output/bap_15customer_nodes_summary.csv"

open(file_name, "w") do io
    redirect_stdout(io) do

        read_nico_dataset("../../Data/Instances/Data/"*filename*".txt")
        global optimalSolution = nothing

        for (idx, parking) in enumerate(parking_availability)
            println("parking availability[$idx] = ", parking)
        end
        println("\n $(repeat("=", 70))")

        #region : branch-and-price
        global start_time = time()
        time_exceeded() = (time() - start_time) > TIME_LIMIT
        global outfile = "bp_$(filename)_result.csv"
        global random_pair_branching = true  # BASELINE: random customer pair from most fractional route
        global dfs_node_selection = true     # BASELINE: depth-first search tree exploration

        global execution_time_total = @time @CPUtime begin
            execution_time_prep = @elapsed lrp_subproblems = preparation_branch_and_price()

            #region : create model and initial columns
            execution_time = @elapsed begin
                global model = Model(CPLEX.Optimizer)
                set_silent(model)
                set_optimizer_attribute(model, "CPX_PARAM_THREADS", 1)
                set_optimizer_attribute(model, "CPX_PARAM_SCRIND", 1)

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
            batch_size = 1
            stop_processing = false

            while !isempty(lrp_subproblems) && !stop_processing
                if time_exceeded()
                    println("\n Time limit reached before next batch.")
                    break
                end

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
            println("total execution time of preparation for sorting subproblems   : $(round(execution_time_prep, digits=2)) seconds")
            println("total execution time of column generation solving subproblems : ", round(execution_time_cg_subproblem, digits=2), " seconds")
            println("total execution time solving branch and price                 : ", round(execution_time_bap, digits=2), " seconds")
            println("total number of branch-and-price nodes explored               : ", total_bap_nodes)
            @info "total number of branch-and-price nodes explored : ", total_bap_nodes
        end
        #endregion

        global total_time = time() - start_time
        println("total execution time (build + algo)                           : $(round(total_time, digits=2)) seconds")

        println("\n================================================================")

        if !isnothing(optimalSolution)
            println("\nOptimal solution found with cost = ", round(upperBound, digits=2), " and sequence: ")
            for route in optimalSolution
                println(route.sequence, "  ", "load =$(route.load)  cost =", round(route.cost, digits=2))
            end
        else
            println("\nNo optimal solution recorded (time limit reached or infeasible).")
        end

        # ---- Append this instance's summary row to the shared CSV ----
        currentTime = Dates.format(now(), "dd-mm-yyyy-HH-MM")
        obj_report = isnothing(optimalSolution) ? "no solution" : round(upperBound, digits=2)
        # NOTE: instance names contain commas themselves (e.g. "ce1-2,3,15"),
        # so this row uses ";" as the delimiter -- a plain "," would make the
        # filename field swallow extra commas and shift every column after it.
        row_data = [currentTime, filename, length(customers), length(satellites),
                    total_bap_nodes, obj_report, round(total_time, digits=2)]
        open(summary_csv, "a") do f
            println(f, join(row_data, ";"))
        end
    end
end
