fileName = ""
savePath = ""

x_coor = []
y_coor = []
demands = []
distances = Array{Float64}(undef, 0, 0)
np = 0
nc = 0
PI = []

Q0 = 0
Q1 = 200
Q2 = 50

eta1 = 1
eta2 = 1

V1 = 1:1
V2 = 1:10

maxDuration = 300
maxDuration1e = 1000