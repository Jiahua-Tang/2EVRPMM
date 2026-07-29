using Plots, Random, DataStructures, Combinatorics, Printf,
    HiGHS, SparseArrays, Test, DataFrames, CPLEX, JuMP, Dates, Base.Threads, CPUTime
using Logging, LoggingExtras

include("Utiles.jl")

global root = "$(pwd())/../../Data/Instances/"

time_stamp = Dates.format(now(), "ddmmyy_HHMM")
filename = ARGS[1]
file_name = "Output/bl_"*filename*"_"*time_stamp*".txt"
mkpath(dirname(file_name))
global status_debug = true

read_nico_dataset("../../Data/Instances/Data/"*filename*".txt")

function solveBaseline()
    t_start = time()
    eta1 = 1
    eta2 = 1
    M = 1000000

    TT1 = arc_cost
    TT2 = arc_cost

    model = Model(CPLEX.Optimizer)

    active_satellites = A1[parking_availability .== 1]
    active_A1 = [1; collect(active_satellites)]
    active_A2 = union(active_satellites, customers)
    @variable(model, x[active_A1, active_A1], Bin)
    @variable(model, z[active_A2, active_A2, active_satellites], Bin)
    @variable(model, t[active_A2]>=0)
    @variable(model, w[active_satellites]>=0)
    @variable(model, f[active_A2,active_A2,active_satellites]>=0)
    @variable(model, u[active_A1]>=0,Int)

    for i in active_A1
        @constraint(model, x[i,i] == 0)
    end
    for i in active_A2
        for s in active_satellites
            @constraint(model, z[i,i,s] == 0)
        end
    end
    for i in active_satellites
        for j in active_A2
            for s in active_satellites
                if i != s
                    @constraint(model, z[i,j,s]==0)
                end
            end
        end
    end
    for i in active_satellites
        for j in active_A2
            for s in active_satellites
                if i != s
                    @constraint(model, z[j,i,s]==0)
                end
            end
        end
    end

    @objective(model, Min,
               sum(arc_cost[i,j] * x[i,j] for i in active_A1, j in active_A1 if i!=j) +
               sum(arc_cost[i,j] * z[i,j,s] for i in active_A2, j in active_A2, s in active_satellites if i != j))

    @constraint(model, [i in active_satellites], sum(x[j,i] for j in active_A1 if i!= j) == sum(x[i,j] for j in active_A1 if i!= j))
    @constraint(model, [i in active_satellites], sum(x[i,j] for j in active_A1 if i != j) <= 1)

    @constraint(model, sum(x[1,j] for j in active_A1 if j !=1)==1)
    @constraint(model, sum(x[j,1] for j in active_A1 if j !=1)==1)

    @constraint(model, [i in active_satellites, j in active_satellites, i!=j], u[i] - u[j] + length(active_A1)*x[i,j] <= length(active_A1)-1)

    @constraint(model, [s in active_satellites], w[s] <= capacity_1e_vehicle * sum(x[i,s] for i in active_A1))

    @constraint(model, [s in active_satellites], w[s] == sum(f[s, j, s] for j in active_A2))

    @constraint(model, [i in active_A2, s in active_satellites], sum(z[j,i,s] for j in active_A2 if i!= j) == 
                                                                 sum(z[i,j,s] for j in active_A2 if i!= j))
    @constraint(model, [i in active_satellites, j in active_satellites, s in active_satellites], z[i,j,s] == 0)
    
    @constraint(model, [s in active_satellites], sum(z[s,j,s] for j in active_A2)<=nb_vehicle_per_satellite)

    @constraint(model, [j in customers], sum(z[i,j,s] for i in active_A2, s in active_satellites) == 1)

    @constraint(model, [i in customers], sum(f[j,i,s] for j in active_A2, s in active_satellites) -
                                         sum(f[i,j,s] for j in active_A2, s in active_satellites)
                                         == demands[i])

    @constraint(model, [i in active_A2, j in active_A2, s in active_satellites], f[i,j,s]<=capacity_2e_vehicle*z[i,j,s])

    @constraint(model, [s in active_satellites], w[s] <= sum(x[j,s] for j in active_A1)*capacity_1e_vehicle)

    # Time window constraints (2e echelon)
    # Arrival time initialization from satellite to first customer
    @constraint(model, [s in active_satellites, j in customers],
        arc_cost[s,j] * z[s,j,s] <= t[j])
    # Arrival time propagation between customers
    @constraint(model, [i in customers, j in customers, s in active_satellites],
        t[i] + arc_cost[i,j] <= t[j] + M * (1 - z[i,j,s]))
    # Time window bounds at customers
    @constraint(model, [i in customers], t[i] >= time_window[i][1])
    @constraint(model, [i in customers], t[i] <= time_window[i][2])

    build_time = time() - t_start
    set_optimizer_attribute(model, "CPX_PARAM_TILIM", 3600*3)
    set_optimizer_attribute(model, "CPX_PARAM_CLOCKTYPE", 1)
    set_optimizer_attribute(model, "CPX_PARAM_THREADS", 1)
    set_optimizer_attribute(model, "CPX_PARAM_SCRIND", 1)
    optimize!(model)
    total_time_with_build = time() - t_start

    currentTime = Dates.format(now(), "dd-mm-yyyy-HH-MM")

    println()
    println("Model build time                  : $(round(build_time, digits=3)) seconds")
    println("CPLEX solver time                 : $(MOI.get(model, MOI.SolveTimeSec())) seconds")
    println("Total execution time (build+solve): $(round(total_time_with_build, digits=3)) seconds")
    gap_print = try MOI.get(model, MOI.RelativeGap()) catch; "N/A" end
    println("Gap: ", gap_print)

    println("\n $(repeat("=", 70))")
    if primal_status(model) == MOI.FEASIBLE_POINT
        println("Baseline objective value: ", round(objective_value(model), digits=2))
    end
    println("Termination status: ", termination_status(model))
    println("File name: ", instance_name)
    println("Number of customers: ", nb_customer)
    println("Number of parkings: ", nb_parking)
    println("Number of microhubs: ", nb_microhub)
    println("Number of robots/MM: ", nb_vehicle_per_satellite)
    println(repeat("=", 70))

    # CSV output
    jobid = get(ENV, "SLURM_JOB_ID", "nojob")
    taskid = get(ENV, "SLURM_PROCID", "0")
    outfile = "./bl_$(nb_customer)_result_$(jobid)_$(taskid).csv"

    obj_val = try objective_value(model) catch; missing end
    gap_val = try MOI.get(model, MOI.RelativeGap()) catch; missing end
    solve_time = try MOI.get(model, MOI.SolveTimeSec()) catch; missing end

    status_text = if termination_status(model) == MOI.OPTIMAL
        "/"
    elseif primal_status(model) == MOI.FEASIBLE_POINT
        string(gap_val)
    else
        "no feasible solution"
    end

    row_data = [
        currentTime,
        "bl",
        "\"$instance_name\"",
        length(customers),
        length(satellites),
        sum(parking_availability),
        nb_vehicle_per_satellite,
        total_time_with_build,
        build_time,
        solve_time,
        obj_val,
        status_text
    ]

    open(outfile, "a") do file
        println(file, join(row_data, ","))
    end

    println("\n--- 1e routes (x) ---")
    for i in active_A1
        for j in active_A1
           if value(x[i,j]) > 0.5
                println("x[$i -> $j] = 1")
           end
        end
    end

    println("\n--- 2e routes (z) ---")
    for i in active_A2
        for j in active_A2
            for s in active_satellites
                if value(z[i,j,s]) > 0.5
                    println("z[$i -> $j | sat=$s] = ", round(value(z[i,j,s]), digits=2))
                end
            end
        end
    end

end

open(file_name, "w") do io
    redirect_stdout(io) do

        for (idx, parking) in enumerate(parking_availability)
            println("parking availability[$idx] = ", parking)
        end
        println("\n $(repeat("=", 70))")

        solveBaseline()

    end
end

# run(`open -a "Visual Studio Code" $file_name`)