"""This program solves for the UAV optimization formulated by Mixed Integer Linear Programming (MILP) Formulation.
This is a Vehicle Routing Problem formulated by MILP formulation and solved using Gurobi solver. This is an exact solution
solved by Branch-and-Bound algorithm.
"""

import gurobipy
import pandas as pd
import numpy as np
from gurobipy import *

Q = 30000  # vehicle fuel capacity
no_of_vehicles = 3  # free to modify
vehicle_speed = 33

df = pd.read_excel('3veh_2clusters_3clustdist_datapts/3veh_2clusters_1bd_2.xlsx', engine='openpyxl')
df = df.drop(df.columns[0], axis=1)
df.insert(0, 'Node', range(1, len(df)+1))
df = df.rename(columns={"x": "X", "y": "Y"})
df2 = pd.read_excel('Additional data for gurobi.xlsx')
df2 = df2.drop(df2.columns[0], axis=1)
df = pd.concat([df, df2], axis=1)
Y, X = list(df["Y"]), list(df["X"])
coordinates = np.column_stack((X, Y))
et, lt, st = list(df["St"]), list(df["Et"]), list(df["Sert"])
n = len(coordinates)
# depot, customers = coordinates[0:9, :], coordinates[9:, :]
M = 1000000  # big number

# e = Env(empty=True)
# e.setParam("Record", 1)
# e.start()

m = Model("MDVRPCTW")
x, f, t = {}, {}, {}  #intialize the decision variables

'''distance matrix (34*34 array)'''
dist_matrix = np.empty([n, n])
for i in range(len(X)):
    for j in range(len(Y)):
        '''variable_1: X[i,j] =(0,1), i,j = Nodes'''
        x[i, j] = m.addVar(vtype=GRB.BINARY, name="x%d,%d" % (i, j))
        dist_matrix[i, j] = np.sqrt((X[i] - X[j]) ** 2 + (Y[i] - Y[j]) ** 2)
        if i == j:
            dist_matrix[i, j] = M  ## big 'M'
        continue
m.update()

'''variable_2: f[j] = fuel level
   variable_3: t[j] = cumulative time'''
for j in range(n):
    f[j] = m.addVar(lb=0, vtype=GRB.CONTINUOUS, name="y%d" % (j))   # fuel level variable
    t[j] = m.addVar(lb=0, vtype=GRB.INTEGER, name="z%d" % (j))   # cumulative time variable
m.update()

  # vehicles leaving each node except start and end depot
for i in range(n):
    if i == 0 or i == 8:
        continue
    elif i in [1, 2, 3, 4, 5, 6, 7]:
        continue
    else:
        m.addConstr(quicksum(x[(i, j)] for j in range(n)) == 1)
m.update()

  # vehicles arriving to each node except start and end depot
for j in range(n):
    if j == 0 or j == 8:
        continue
    elif j in [1, 2, 3, 4, 5, 6, 7]:
        continue
    m.addConstr(quicksum(x[(i, j)] for i in range(n)) == 1)
m.update()

for i in range(1, 8):
    m.addConstr(quicksum(x[(i, j)] for j in range(n)) <= 1)
m.update()

for j in range(1, 8):
    m.addConstr(quicksum(x[(i, j)] for i in range(n)) <= 1)
m.update()

 # vehicles leaving start depot
m.addConstr(quicksum(x[(0, j)] for j in range(n)) == no_of_vehicles)
m.update()

  # vehicles arriving to end depot
m.addConstr(quicksum(x[(i, 8)] for i in range(n)) == no_of_vehicles)
m.update()


'''constraint_5: capacity of vehicle, if the vehicle visits any node between 0 through 8 (station nodes), it reloads to Q, 
else vehicle continues to unload'''

for i in range(n):
    for j in range(0, n):
        if j <= 8:
            m.addConstr(f[j] == Q)
        else:
            m.addConstr(f[j] <= Q)
            m.addConstr(f[j] >= 0)
            m.addConstr(f[j] <= f[i] - (dist_matrix[i, j] * (x[i, j])) + M*(1-x[(i, j)]))
m.update()

'''constraint_6: time-windows and also eliminating sub-tours'''
for i in range(n):
    for j in range(n):
        m.addConstr(t[j] >= (et[j]))  # service should start after the earliest service start time
        m.addConstr(t[j] <= (lt[j]))  # service can't be started after the latest service start time
        m.addConstr(t[j] >= t[i] + (st[i] + ((dist_matrix[i, j]/vehicle_speed)) * x[(i, j)]) - M*(1-x[(i, j)]))
m.update()

'''constraint 7: two consecutive visits should not be station nodes'''
for i in range(9):
    for j in range(9):
        if i == j:
            continue
        m.addConstr((x[(i, j)] == 0))
m.update()
'''constraint 8: refueling constraints'''
# x1 = 0
for i in range(9, n):
    for j in range(9):
        m.addConstr((x[(i, j)] == 1) >> (f[i] >= dist_matrix[(i, j)]))
m.update()

'''constraint 9: Refueling constraints'''
for i in range(9):
    for j in range(9, n):
        m.addConstr((x[(i, j)] == 1) >> (f[i] == Q))
m.update()

for j in range(n):
    m.addConstr((x[(8, j)]) == 0)
m.update()

for i in range(9, n):
    for j in range(8):
        m.addConstr((x[(i, j)] == 1) >> (quicksum(x[(j, i)] for i in range(9, n)) == 1))

m.write("myLP.lp")

'''objective function'''

m.setObjective(quicksum(quicksum(x[(i, j)]*dist_matrix[(i, j)] for j in range(n))
                        for i in range(n)), GRB.MINIMIZE)
m.update()
# copy = m.copy()

m.Params.MIPGap = 0.1
m.Params.Method = 1
m.Params.IntFeasTol = 0.000000001
m.Params.NumericFocus = 3
m.Params.TimeLimit = 240  # seconds
m.optimize()
# m.printQuality()

sol_y, sol_x, sol_z = {}, {}, {}
if m.status == GRB.OPTIMAL:
    sol_y, sol_x, sol_z = m.getAttr('x', f), m.getAttr('x', x), m.getAttr('x', t)
    X, Y, Z = np.empty([n, n]), np.empty([n]), np.empty([n])
    for i in range(n):
        Y[i] = sol_y[i]
        Z[i] = sol_z[i]
        for j in range(n):
            X[i, j] = int(sol_x[i, j])
    print('\nObjective is:', m.objVal)
    print('\nDecision variable X (binary decision of travelling from one node to another):\n', X.astype('int32'))
    print('\nDecision variable z:(service start time of every customers in minutes)\n', Z.astype('int32')[1:])
    print('\nDecision variable y (cumulative fuel at every customer node):\n', Y.astype('int32')[1:])

    selected = gurobipy.tuplelist((i, j) for i, j in sol_x.keys() if sol_x[i, j] > 0.5)
    for i, tup in enumerate(selected.select(0, '*')):
        print("\nRoute for UAV {}:\n 0 Fuel(30000)".format(i+1), end='')
        neighbor = tup[1]
        UAV_dist = dist_matrix[0][neighbor]
        UAV_fuel = Y[neighbor]
        while neighbor:
            if neighbor == 8:
                break
            print(" -> {} Fuel({})".format(neighbor, UAV_fuel), end='')
            next_neighbor = selected.select(neighbor, '*')[0][1]
            UAV_dist += dist_matrix[neighbor][next_neighbor]
            if next_neighbor in range(1, 8):
                UAV_fuel = Q
            else:
                UAV_fuel = Y[next_neighbor]
            neighbor = next_neighbor
        print(" -> 8 Fuel({})".format(UAV_fuel))
        print("Route distance: {}".format(UAV_dist))
        print("Route fuel: {}".format(UAV_fuel))
    print("\nTotal distance for all routes: {}".format(m.objVal))

'''retrieve the solution'''


