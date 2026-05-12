using Plots, Random, DataStructures, Combinatorics, Printf, 
    HiGHS, SparseArrays, Test, DataFrames, CPLEX, JuMP, Dates, Base.Threads, CPUTime
using Logging, LoggingExtras

include("Utiles.jl")
include("BranchAndPrice/Utiles.jl")
include("BranchAndPrice/branchAndPrice.jl")
include("CompactModel/compactModel.jl")
include("LrpLowerBound/solveLRP.jl")
include("baseline.jl")

global root = "$(pwd())/../../Data/Instances/"
# global root = "/gpfs/workdir/tangj/2EVRPMM/Code/Code/"


# instance_size = 70
# global random_seed = 42

const TIME_LIMIT = 3600*3
# # time_stamp = "_"*Dates.format(now(), "ddmmyyHHMM")
# file_name = "Output/S$(random_seed)/v2.2"*"_s"*string(random_seed)*time_stamp*".txt"
file_name = "Output/demo.txt"
mkpath(dirname(file_name))
filename = "ce4-2,3,30"

open(file_name, "w") do io
    redirect_stdout(io) do

        # redirect_stderr(io) do

#            # generateData(instance_size, random_seed)
            # global fileName = "R103"
            # read_Solomon_Dataset_TW("../../Data/Demo/100/" * fileName * ".txt", 1200)
            # retrieve_solomon_random_data("../../Data/Demo/100/" * fileName * ".txt", 1200, 25)
            read_nico_dataset("../../Data/Instances/Data/"*filename*".txt")
            global optimalSolution = nothing

            for (idx, parking) in enumerate(parking_availability)   
                println("parking availability[$idx] = ", parking)
            end
            println("\n $(repeat("=", 70))")
    
            #==========================================================================
            BASELINE
            solvebaseline()
            ==========================================================================#


            #==========================================================================
            COMPACT MODEL
            ==========================================================================#
            global execution_time_limit = 3600*3
            # execution_time_cplex = @time @CPUtime solveCompactModelDisplayResult()
    
            #==========================================================================
            BRANCH-AND-PRICE
            ==========================================================================#

            #region : branch-and-price
            start_time = time()
            time_exceeded() = (time() - start_time) > TIME_LIMIT

            global execution_time_total = @time @CPUtime begin
                execution_time_prep = @elapsed lrp_subproblems = preparation_branch_and_price()

                # println("\n $(repeat("=", 70))")
                #region : create model and initial columns
                execution_time = @elapsed begin
                    global model = Model(CPLEX.Optimizer)
                    set_silent(model)
                    set_optimizer_attribute(model, "CPX_PARAM_THREADS", 1)
                    set_optimizer_attribute(model, "CPX_PARAM_SCRIND", 1)
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

                # ============================================================
                # Batch-by-3 root-node + branch-and-price
                # Loop: pull up to 3 LRP subproblems, solve their root nodes,
                # then run branch-and-price on each before pulling the next 3.
                # ============================================================
                open("NodeMatrix/NodeMatrix_$(filename).csv", "w") do file
                    row_data = ["id", "parent", "cgLowerBound", "FS", "rdtCG", "rdtFS","Child","Child"]
                    println(file, join(row_data, ","))
                end

                execution_time_cg_subproblem = 0.0
                execution_time_bap = 0.0

                global num_iter = 1
                global num_iter_global = 1
                batch_size = 3
                stop_processing = false

                while !isempty(lrp_subproblems) && !stop_processing
                    if time_exceeded()
                        println("\n Time limit reached before next batch.")
                        break
                    end

                    # ---------- 1) Build a batch of up to `batch_size` root nodes ----------
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
                        while !isempty(root_nodes) &&  num_iter_global == 1
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

            end
            #endregion

            # ----- Time breakdown (build vs algorithm) -----
            total_time           = time() - start_time
            build_model_time     = execution_time_build_model               # master + all subproblem JuMP construction
            time_excluding_build = total_time - build_model_time            # algorithm/solver work without model building

            println("total model construction time (all JuMP builds)               : $(round(build_model_time, digits=2)) seconds")
            println("total execution time (build + algo)                           : $(round(total_time, digits=2)) seconds")
            println("total execution time excluding model building                 : $(round(time_excluding_build, digits=2)) seconds")

            # for route in routes_2e
            #     println(route.sequence, "  ", round(route.cost, digits=2))

            # end

            println("\n================================================================")

            if !isnothing(optimalSolution)
                # @info "output"
                currentTime = Dates.format(now(), "dd-mm-yyyy-HH-MM")
                row_data = [currentTime, "bp", "\"$filename\"", length(satellites), sum(parking_availability), nb_vehicle_per_satellite, upperBound, time() - start_time, "/"]
                # open("result.csv", "a") do file
                #     println(file, join(row_data, ",")) 
                # end
                println("\nOptimal solution found with cost = ", round(upperBound, digits=2), " and sequence: ")
                for (idx, route) in enumerate(optimalSolution)
                    println(route.sequence, "  ", "load =$(route.load)  cost =", round(route.cost, digits=2))
                    # if idx == 1
                    #     # 1e (FEV) route: print arrival only at satellites (skip the depot).
                    #     println("        arrival (satellites only): ",
                    #             [(n, round(route.arrival_time[n], digits=2)) for n in route.sequence if n in satellites])
                    # else
                    #     # 2e (SEV) route: print arrival at every node in the sequence.
                    #     println("        arrival: ",
                    #             [(n, round(route.arrival_time[n], digits=2)) for n in route.sequence])
                    # end
                end
            end
            #endregion
    end
end

run(`open -a "Visual Studio Code" $file_name`)