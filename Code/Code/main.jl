using Plots, Random, DataStructures, Combinatorics, Printf, 
    HiGHS, SparseArrays, Test, DataFrames, CPLEX, JuMP

include("Utiles.jl")
include("BranchAndPrice/Utiles.jl")
include("BranchAndPrice/branchAndPrice.jl")
include("CompactModel/compactModel.jl")
include("LrpLowerBound/solveLRP.jl")


global root = "$(pwd())/TEST/"
# global root = "/gpfs/workdir/tangj/2EVRPMM/Code/Code/"

# generateData()
readData("E-n33-k4.txt", ARGS)

#=========================================================#

# solveCompactModelDisplayResult()

#=========================================================#

# solvemasterProblem()

#=========================================================#

# . [1, 2, 1]  Set([2])   lower bound= 484.1
# 2. [1, 2, 5, 1]  Set([5, 2])   lower bound= 539.22
# 3. [1, 5, 1]  Set([5])   lower bound= 540.03
# 4. [1, 2, 7, 1]  Set([7])   lower bound= 560.08
# 5. [1, 2, 5, 7, 1]  Set([7, 2])   lower bound= 583.0
# 6. [1, 2, 5, 6, 1]  Set([6, 2])   lower bound= 598.65
# 7. [1, 3, 1]  Set([3])   lower bound= 598.68
# 8. [1, 5, 6, 1]  Set([6])   lower bound= 599.47
# 9. [1, 2, 5, 3, 6, 1]  Set([5, 6, 2])   lower bound= 612.06
# 10. [1, 2, 5, 3, 1]  Set([5, 2, 3])   lower bound= 612.68
# 11. [1, 5, 3, 1]  Set([5, 3])   lower bound= 613.5
# 12. [1, 5, 3, 6, 1]  Set([5, 6])   lower bound= 614.53
# 13. [1, 2, 3, 1]  Set([2, 3])   lower bound= 615.84
# 14. [1, 2, 5, 6, 3, 1]  Set([6, 2, 3])   lower bound= 621.96
# 15. [1, 5, 6, 3, 1]  Set([6, 3])   lower bound= 622.99
# 16. [1, 5, 2, 7, 1]  Set([5, 7])   lower bound= 628.7
# 17. [1, 2, 5, 4, 3, 6, 1]  Set([4, 6, 2])   lower bound= 632.42
# 18. [1, 2, 5, 4, 1]  Set([4, 2])   lower bound= 632.54
# 19. [1, 5, 4, 3, 6, 1]  Set([4, 6])   lower bound= 633.45
# 20. [1, 5, 4, 1]  Set([4])   lower bound= 633.57
# 21. [1, 2, 5, 6, 3, 7, 1]  Set([6, 7, 2])   lower bound= 645.53
# 22. [1, 3, 4, 5, 2, 1]  Set([5, 4, 2])   lower bound= 646.85
# 23. [1, 5, 6, 3, 7, 1]  Set([6, 7])   lower bound= 649.71
# 24. [1, 2, 5, 4, 3, 1]  Set([4, 2, 3])   lower bound= 649.81
# 25. [1, 2, 5, 3, 7, 1]  Set([5, 7, 2])   lower bound= 650.57
# 26. [1, 5, 4, 3, 1]  Set([4, 3])   lower bound= 650.63
# 27. [1, 3, 4, 5, 1]  Set([5, 4])   lower bound= 651.03
# 28. [1, 2, 6, 3, 5, 1]  Set([5, 6, 3])   lower bound= 651.9
# 29. [1, 3, 5, 7, 1]  Set([7, 3])   lower bound= 660.64
# 30. [1, 2, 6, 3, 4, 5, 1]  Set([5, 4, 6])   lower bound= 674.05
# 31. [1, 2, 3, 5, 7, 1]  Set([7, 2, 3])   lower bound= 674.44
# 32. [1, 2, 5, 4, 3, 7, 1]  Set([4, 7, 2])   lower bound= 677.23
# 33. [1, 5, 4, 3, 7, 1]  Set([4, 7])   lower bound= 681.41
# 34. [1, 2, 4, 3, 5, 1]  Set([5, 4, 3])   lower bound= 691.12
# 35. [1, 2, 6, 3, 5, 7, 1]  Set([6, 7, 3])   lower bound= 692.02
# 36. [1, 3, 6, 5, 2, 7, 1]  Set([5, 6, 7])   lower bound= 704.53
# 37. [1, 3, 5, 2, 7, 1]  Set([5, 7, 3])   lower bound= 704.98
# 38. [1, 2, 6, 3, 4, 5, 7, 1]  Set([4, 6, 7])   lower bound= 718.77
# 39. [1, 2, 4, 3, 5, 7, 1]  Set([4, 7, 3])   lower bound= 735.84
# 40. [1, 3, 4, 5, 2, 7, 1]  Set([5, 4, 7])   lower bound= 736.4
# 41. [1, 2, 4, 5, 6, 3, 1]  Set([4, 6, 3])   lower bound= 737.23

# routes_1e_complete = generateNonDominate1eRoutes(minimum_parkings_required)
# lb_lrp_per_route = calculateLRPLowerBound(routes_1e_complete)

lb_lrp_per_route = calculateLRPLowerBoundByParking()


# displayLRPLowerBound(deepcopy(lb_lrp_per_route), routes_1e_complete)

# global num_iter_global = 1
# global upperBound = Inf
# global optimalSolution = nothing

# global optimal_found_iteration = 0
# global execution_time_subproblem = 0
# global deepest_level = 0
# global optimal_found_in = 0

# execution_time = @elapsed begin
#     while !isempty(lb_lrp_per_route) # && num_iter_global < 11
#         min_value, min_idx = findmin(lb_lrp_per_route)
#         if min_value > upperBound   break   end

#         route_1e = Vector{Route}()
#         push!(route_1e, routes_1e_complete[min_idx])
#         branchAndPriceWithScore(route_1e)
#         delete!(lb_lrp_per_route, min_idx)
#         @info "current upper bound is $(round(upperBound,digits=2))"
#         global num_iter_global
#         num_iter_global += 1
#     end
# end

# if !isnothing(optimalSolution)
#     @info "Current optimal solution found in interation $optimal_found_iteration:"
#     for route in optimalSolution 
#         println(route.sequence)
#     end   
# end
# println("Execution time = $(round(execution_time, digits=2)), time spent in solving subproblem = $(round(execution_time_subproblem, digits=2)), takes percentage of $(round(execution_time_subproblem/execution_time,digits=2)*100)%, deepest node dived to level $deepest_level")

#===============================================================================================#

# execution_time = @elapsed begin
#         route_1e = Vector{Route}()
#         push!(route_1e, generate1eRoute([1,3,4,5,2,1]))
#         branchAndPriceWithScore(route_1e)
#         @info "current upper bound is $(round(upperBound,digits=2)) "
#         global num_iter
#         num_iter += 1
#     end
# if !isnothing(optimalSolution)
#     println("\nExecution time = $(round(execution_time, digits=2)), time spent in solving subproblem = $(round(execution_time_subproblem, digits=2)), takes percentage of $(round(execution_time_subproblem/execution_time,digits=2)*100)%, deepest node dived to level $deepest_level")
#     println("Current optimal solution found in interation $optimal_found_iteration in level $optimal_found_in:")
#     for route in optimalSolution 
#         println(route.sequence)
#     end   
# end

# solveRMP(generate1eRoute([1,3,6,2,5,1]))