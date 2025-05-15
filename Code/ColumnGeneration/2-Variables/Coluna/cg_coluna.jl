using JuMP, CPLEX, BlockDecomposition, Coluna, MathOptInterface;
using Coluna.ColGen
using JuMP, CPLEX, Plots, Random, DataStructures, Combinatorics
import DataFrames
import HiGHS
import Plots
import SparseArrays
import Test  #src
include("Utiles.jl")

const BD = BlockDecomposition
const MOI = MathOptInterface

data = initializeData(5,3,10)
const coor = data[1]
const nb_parking = data[2]
const nb_microhub = data[3]
const parking_availability = data[4]
const demands = data[5]
const arc_cost = data[6]

## Define parameters
const nb_vehicle_per_satellite = 10
const capacity_2e_vehicle = 30
const capacity_microhub = 35
const maximum_duration_2e_vehicle = 50
const minimum_parkings_required = Int(ceil(sum(demands)/capacity_microhub))

## Define set
const points = 1:length(demands)
const customers = 2 + nb_parking : length(coor)
const satellites = 2:1 + nb_parking
const A2 = 2:length(coor)
const A1 = 1:1+length(satellites)

function solve_esp()
    sp_model = Model(CPLEX.Optimizer)

    @variable(sp_model, x[i in A2], Bin)
    # @constraint(sp_model, )

    # @objective()

    # optimize!(sp_model)


end

function pricing_callback(cbdata)
    cur_route = BD.callback_spid(cbdata, model)

    println("Pricing callback for route $(cur_route): $(initial_2e_routes[cur_route].sequence)")

    reduced_cost = [BD.callback_reduced_cost(cbdata, y[cur_route])]
    display(reduced_cost)

end

# routes_1e, routes_2e = generateAllRoutes()

route_1e = generate1eRoute([1,2,3,4,1])
all_routes_2e = generateAllFeasible2eRoute()

initial_2e_routes = generate2eInitialRoutes()

coluna = optimizer_with_attributes(
    Coluna.Optimizer,
    "params" => Coluna.Params(
        solver = Coluna.Algorithm.TreeSearchAlgorithm()
    ),
    "default_optimizer" => CPLEX.Optimizer
)

@axis(Routes_axis, 1:length(customers))

model = BlockModel(coluna)

@variable(model, y[r in 1:length(initial_2e_routes)], Bin)

@constraint(model, parkingSync[p in satellites], route_1e.b1[p]*nb_vehicle_per_satellite >= sum(y[r]*route.b2out[p] for (r, route) in enumerate(initial_2e_routes)))

@constraint(model, cov[i in customers], sum(route.a[i-1-length(satellites)] * y[r] for (r, route) in enumerate(initial_2e_routes) )==1)

@objective(model, Min, sum(y[r] * route.cost for (r, route) in enumerate(initial_2e_routes)))

@dantzig_wolfe_decomposition(model, decomposition, Routes_axis)

subproblems = BD.getsubproblems(decomposition);
BD.specify!.(subproblems, lower_multiplicity = 0, solver = pricing_callback);

optimize!(model)