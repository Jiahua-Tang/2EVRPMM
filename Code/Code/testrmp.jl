using JuMP
using CPLEX
using Printf
using Dates

# ---------------- Timing helpers ----------------
mutable struct TTimer
    t_build::Float64
    t_solve::Float64
end
TTimer() = TTimer(0.0, 0.0)

tic() = Dates.now()
toc_ms(t0::DateTime) = (Dates.now() - t0).value / 1_000  # milliseconds

# ---------------- Pricing: unbounded knap DP (1-based indexing) ----------------
# maximize sum(v[i] * a[i]) s.t. sum(w[i]*a[i]) ≤ L, a[i] ≥ 0 integer
# 返回 (best_value, a::Vector{Int})
function pricing_unbounded_knap(v::Vector{Float64}, w::Vector{Int}, L::Int)
    n = length(w)
    dp  = fill(-1e18, L + 1)  # dp[c+1] = best value at capacity c
    pre = fill(0,     L + 1) # pre[c+1] = last item index used
    dp[1] = 0.0               # capacity 0

    for c in 0:L
        valc = dp[c + 1]
        if valc ≤ -1e17
            continue
        end
        @inbounds for i in 1:n
            wi = w[i]
            nc = c + wi
            if nc ≤ L
                val = valc + v[i]
                if val > dp[nc + 1] + 1e-12
                    dp[nc + 1] = val
                    pre[nc + 1] = i
                end
            end
        end
    end

    # best end capacity
    best_val = -1e18
    best_c   = 0
    for c in 0:L
        if dp[c + 1] > best_val
            best_val = dp[c + 1]; best_c = c
        end
    end

    # reconstruct pattern
    a = zeros(Int, n)
    c = best_c
    while c > 0 && pre[c + 1] != 0
        i = pre[c + 1]
        a[i] += 1
        c -= w[i]
    end
    return best_val, a
end

# ---------------- Add a new column (pattern) ----------------
function add_pattern!(model::Model, cover_con::Vector{ConstraintRef}, a::Vector{Int}, cost::Float64)
    # instrumentation: print the column being added
    @printf("  + add pattern a = %s  (cost=%.3f)\n", string(a), cost)
    x = @variable(model, lower_bound = 0.0)                 # new column var
    JuMP.set_objective_coefficient(model, x, cost)          # column cost
    @inbounds for i in 1:length(a)
        ai = a[i]
        if ai != 0
            JuMP.set_normalized_coefficient(cover_con[i], x, ai)
        end
    end
    return x
end

# ---------------- Column Generation main ----------------
function cutting_stock_cg(; L::Int=10,
                          w::Vector{Int} = [6,4,3],
                          d::Vector{Int} = [7,10,9],
                          max_iter::Int=10,
                          rc_tol::Float64=-1e-9,
                          use_artificial_init::Bool=true,
                          BigM::Float64=100.0)
    @assert length(w) == length(d)
    n = length(w)

    model = Model(CPLEX.Optimizer)
    set_silent(model)
    set_optimizer_attribute(model, "CPXPARAM_Threads", 1)
    set_optimizer_attribute(model, "CPXPARAM_MIP_Display", 0)

    # Minimize total rolls
    @objective(model, Min, 0.0)

    # Coverage constraints shell: sum a_ij x_j ≥ d[i]
    cover = Vector{ConstraintRef}(undef, n)
    for i in 1:n
        cover[i] = @constraint(model, 0.0 >= d[i])
    end

    timer = TTimer()

    # -------- Initial columns --------
    t0 = tic()
    if use_artificial_init
        # One artificial column per constraint: covers exactly 1 unit of item i, cost BigM.
        # This guarantees feasibility but is very expensive, so pricing will add cheap real patterns.
        for i in 1:n
            a = zeros(Int, n)
            a[i] = 1
            add_pattern!(model, cover, a, BigM)   # expensive fake column
        end
    else
        # Single-item patterns as before (may converge in 1 iter on some data)
        for i in 1:n
            qty = fld(L, w[i])
            if qty > 0
                a = zeros(Int, n); a[i] = qty
                add_pattern!(model, cover, a, 1.0)
            end
        end
    end
    timer.t_build += toc_ms(t0)

    objs = Float64[]
    for it in 1:max_iter
        # Solve RMP
        t1 = tic()
        optimize!(model)
        timer.t_solve += toc_ms(t1)

        obj = objective_value(model)
        push!(objs, obj)

        # Duals of ≥ constraints in a minimization model are ≤ 0 → use v = -dual ≥ 0
        π = dual.(cover)
        v = [-πi for πi in π]

        # Pricing
        t2 = tic()
        best_val, a = pricing_unbounded_knap(v, w, L)
        timer.t_build += toc_ms(t2)

        rc = 1.0 - best_val   # cost(1) - dual value
        @printf("Iter %2d | RMP obj = %.6f | pricing best = %.6f | rc = %.6e\n",
                it, obj, best_val, rc)

        if rc >= rc_tol
            println("No negative reduced cost column. Converged.")
            break
        end

        # Add new (real) pattern with unit cost
        t3 = tic()
        add_pattern!(model, cover, a, 1.0)
        timer.t_build += toc_ms(t3)
    end

    println("------ Timing (ms) ------")
    @printf("Build/Pricing total: %.1f ms\n", timer.t_build)
    @printf("Solve total:         %.1f ms\n", timer.t_solve)
    return model, cover, objs
end

# -------- Run with artificial columns to FORCE multiple additions ----------
L = 10
w = [6, 4, 3]
d = [7, 10, 9]
model, cover, objs = cutting_stock_cg(L=L, w=w, d=d, use_artificial_init=true, BigM=100.0, max_iter=6)
println("Final LP lower bound = ", last(objs))