"""Main points and formulation
This is the inner-loop UAV optimization. This program is carried on to optimize a fleet of aerial vehicles, where the number of vehicles is 3.
This program performs the optimization using the Vehicle Routing Problem formulation with time-window, recharging and optional node constraints.
The UGV velocity is not a constant one, but it varies within the range of certain values (Here it is 1.5m/s-4.5m/s)
The fuel consumption is linearly proportional to the distance of the UAV traveled. That is, it is the negative of the
distance traveled.
Monte Carlo simulation is performed to analyze our problem formulation.
Objective of our problem is to minimize the total distance traveled.
Recharging time and service time at any point (UGV refuel stop / UAV missions) is 300 seconds (5 minutes).
"""

from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math
import numpy as np
import matplotlib.pyplot as plt
import UGV_route_2cluster_3clustdist_bounds_one
import pandas as pd

distance_list = []
UGV_route, mission_locations = UGV_route_2cluster_3clustdist_bounds_one.main(random_seed=2 * 5)
x = []
y = []
for k in range(len(UGV_route)):
    x.append(UGV_route[k][0])
    y.append(UGV_route[k][1])
interpolated_UGV_stops = [(13200, 13200)]
for i in range(0, len(UGV_route)-1):
    interp_x = [x[i], x[i+1]]
    interp_y = [y[i], y[i+1]]
    for j in range(1, 5):
        interpolate_x = interp_x[0] + (((interp_x[1] - interp_x[0])*j)/4)
        interpolate_y = interp_y[0] + (((interp_y[1] - interp_y[0])*j)/4)
        interpolated_UGV_stops.append((interpolate_x, interpolate_y))


def create_data_model():   # creating a dictionary containing input UAV and UGV parameters
    data = {}
    fuel_capacity = 30000   # fuel_capacity in -ft
    # locs, mission_locations = Monte_carlo_K_means_3_clusters.random_locations()
    for _ in range(len(mission_locations)):
        insert_locations = mission_locations.pop(0)
        interpolated_UGV_stops.append(insert_locations)
    _locations = interpolated_UGV_stops
    print(_locations)
    # print(len(_locations))
    data["locations"] = _locations   # 26.4k x 26.4k sq. ft. (5 x 5 miles sq.)
    df = pd.DataFrame(_locations, columns=['x', 'y'])
    df.to_excel('3veh_2clusters_3clustdist_datapts/3veh_2clusters_1bd_2.xlsx', engine='openpyxl')
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal', adjustable='box')
    for i in range(len(data["locations"])):
        if i <= 8:
            ax.plot(data["locations"][i][0], data["locations"][i][1], 'k.', markersize=10)
        else:
            ax.plot(data["locations"][i][0], data["locations"][i][1], 'kx', markersize=10)
    plt.xlim(0, 26400)
    plt.ylim(0, 26400)
    plt.savefig('3veh_2clusters_3clustdist_datapts/3veh_2clusters_3clustdist_1bd_data_pts_plot_2.pdf')
    data["coordinates"] = [(round((l[0]/5280), 1), round((l[1]/5280), 1)) for l in _locations]

    data["locations"] = _locations   # 26.4k x 26.4k sq. ft. (5 x 5 miles sq.)
    data["num_locations"] = len(data["locations"])
    '''The time windows are devised such a way that the UGV can travel within the speed range of 1.5m/s-4.5m/s'''
    '''When UAV not recharged on the UGV, UGV travels at speed of 1.5 m/s. If recharged, UGV travels 4.5 m/s'''
    data["time_windows"] = [(0, 60),  # veh_start_node
                            (660, 760),  # recharge_station_1
                            (1260, 1360),
                            (1860, 1960),
                            (2460, 2560),
                            (3060, 3160),
                            (3660, 3760),
                            (4260, 4360),
                            (4860, 496000),
                            (0, 100000),
                            (7, 100000),
                            (10, 100000),
                            (14, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000)]
    table_for_gurobi = {
        'St': [tw[0] for tw in data["time_windows"]],
        'Et': [tw[1]//100 if i == 8 else tw[1] for i, tw in enumerate(data["time_windows"])]
    }

    indexes_to_zero = [0, 8]   # First and last recharging elements

    # Create 'Sert' column based on conditions
    table_for_gurobi['Sert'] = [0 if index in indexes_to_zero else 300 for index, _ in enumerate(table_for_gurobi['St'])]

    # Convert to DataFrame
    df_gurobi = pd.DataFrame(table_for_gurobi)
    df_gurobi.to_excel('Additional data for gurobi.xlsx', engine='openpyxl')
    data["counter"] = [0, 0, 0, 0, 0, 0, 0, 0, 0,
                       1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    data["num_vehicles"] = 3
    data["fuel_capacity"] = fuel_capacity
    data["vehicle_speed"] = 33  # ft/s
    data["starts"] = [0, 0, 0]
    data["ends"] = [8, 8, 8]
    distance_matrix = np.zeros((data["num_locations"], data["num_locations"]), dtype=int)
    for i in range(data["num_locations"]):
        for j in range(data["num_locations"]):
            if i == j:
                distance_matrix[i][j] = 0
            else:
                distance_matrix[i][j] = euclidean_distance(data["locations"][i], data["locations"][j])
    dist_matrix = distance_matrix.tolist()
    # print(dist_matrix)
    data["distance_matrix"] = dist_matrix
    assert len(data['distance_matrix']) == len(data['locations'])
    assert len(data['distance_matrix']) == len(data['time_windows'])
    assert len(data['starts']) == len(data['ends'])
    assert data['num_vehicles'] == len(data['starts'])
    assert len(data["counter"]) == len(data['time_windows'])
    return data


def euclidean_distance(position_1, position_2):
    return round(math.hypot((position_1[0] - position_2[0]), (position_1[1] - position_2[1])))


def print_solution(data, manager, routing, solution):
    print("Objective: {}".format(solution.ObjectiveValue()))
    total_distance = 0
    total_load = 0
    total_time = 0
    distance_dimension = routing.GetDimensionOrDie("Distance")
    fuel_dimension = routing.GetDimensionOrDie("Fuel")
    time_dimension = routing.GetDimensionOrDie("Time")
    # counter_dimension = routing.GetDimensionOrDie("Counter")
    dropped_nodes = "Dropped nodes:"
    for node in range(routing.Size()):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if solution.Value(routing.NextVar(node)) == node:
            dropped_nodes += " {}".format(manager.IndexToNode(node))
    print(dropped_nodes)
    dum_list = [(data["locations"][0][0], data["locations"][0][1])]
    dum_list2 = [(data["locations"][0][0], data["locations"][0][1])]
    dum_list3 = [(data["locations"][0][0], data["locations"][0][1])]
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = "Route for vehicle {}:\n".format(vehicle_id)
        # distance = 0
        while not routing.IsEnd(index):
            fuel_var = fuel_dimension.CumulVar(index)
            time_var = time_dimension.CumulVar(index)
            slack_var = time_dimension.SlackVar(index)
            dist_var = distance_dimension.CumulVar(index)
            plan_output += "{0} Fuel({1}) Time({2},{3}) Slack({4},{5}) Distance({6}) -> ".format(
                manager.IndexToNode(index),
                solution.Value(fuel_var),
                solution.Min(time_var),
                solution.Max(time_var),
                solution.Min(slack_var),
                solution.Max(slack_var), solution.Value(dist_var))
            index = solution.Value(routing.NextVar(index))
            if vehicle_id == 0:
                if index <= 32:
                    if index == 1 or index == 2 or index == 3 or index == 4 or index == 5 or index == 6 or index == 7:
                        dum_list.append(data["locations"][index])
                    else:
                        dum_list.append(data["locations"][index + 1])
            elif vehicle_id == 1:
                if index <= 32:
                    if index == 1 or index == 2 or index == 3 or index == 4 or index == 5 or index == 6 or index == 7:
                        dum_list2.append(data["locations"][index])
                    else:
                        dum_list2.append(data["locations"][index + 1])
            else:
                if index <= 32:
                    if index == 1 or index == 2 or index == 3 or index == 4 or index == 5 or index == 6 or index == 7:
                        dum_list3.append(data["locations"][index])
                    else:
                        dum_list3.append(data["locations"][index + 1])
        dist_var = distance_dimension.CumulVar(index)
        fuel_var = fuel_dimension.CumulVar(index)
        time_var = time_dimension.CumulVar(index)
        plan_output += "{0} Fuel({1}) Time({2},{3})\n".format(
            manager.IndexToNode(index),
            solution.Value(fuel_var),
            solution.Min(time_var),
            solution.Max(time_var))
        plan_output += "Distance of the route: {} ft\n".format(solution.Value(dist_var))
        plan_output += "Remaining Fuel of the route: {}\n".format(solution.Value(fuel_var))
        plan_output += "Total Time of the route: {} seconds\n".format(solution.Value(time_var))
        print(plan_output)
        total_distance += solution.Value(dist_var)
        total_load += solution.Value(fuel_var)
        total_time += solution.Value(time_var)
    print('Total Distance of all routes: {} ft'.format(total_distance))
    print('Total Fuel remaining of all routes: {}'.format(total_load))
    print('Total Time of all routes: {} seconds'.format(total_time))

    dum_list.append((data["locations"][8][0], data["locations"][8][1]))
    dum_list2.append((data["locations"][8][0], data["locations"][8][1]))
    dum_list3.append((data["locations"][8][0], data["locations"][8][1]))
    x1 = []
    y1 = []
    x2 = []
    y2 = []
    x3 = []
    y3 = []
    for i in dum_list:
        x1.append(i[0])
        y1.append(i[1])
    for j in dum_list2:
        x2.append(j[0])
        y2.append(j[1])
    for k in dum_list3:
        x3.append(k[0])
        y3.append(k[1])
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal', adjustable='box')

    # plotting the output routes
    for i in range(len(data["locations"])):
        if i <= 8:
            ax.plot(data["locations"][i][0], data["locations"][i][1], 'k.', markersize=10)
        else:
            ax.plot(data["locations"][i][0], data["locations"][i][1], 'kx', markersize=10)

    for i in range(len(x1)):
        if i <= len(x1) - 2:
            ax.arrow(x1[i], y1[i], (x1[i+1] - x1[i]), (y1[i+1] - y1[i]), width=100, color='red')

    for j in range(len(x2)):
        if j <= len(x2) - 2:
            ax.arrow(x2[j], y2[j], (x2[j+1] - x2[j]), (y2[j+1] - y2[j]), width=100, color='green')

    for k in range(len(x3)):
        if k <= len(x3) - 2:
            ax.arrow(x3[k], y3[k], (x3[k+1] - x3[k]), (y3[k+1] - y3[k]), width=100, color='blue')
    plt.xlim(0, 26400)
    plt.ylim(0, 26400)
    plt.savefig('3veh_2clusters_3clustdist_datapts/3veh_2clusters_3clustdist_1bd_2.pdf')
    # plt.show()
    return total_distance


def main():
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]),
        data["num_vehicles"],
        data["starts"],
        data["ends"])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Distance
    stations = [0, 1, 2, 3, 4, 5, 6, 7]  # depot + refill stations

    routing.AddVectorDimension(
        data["counter"],
        manager.GetNumberOfNodes(),
        True,
        "Counter")
    counter_dimension = routing.GetDimensionOrDie("Counter")
    nb_visit = 25 // manager.GetNumberOfVehicles()
    print(f'visit_mean: {nb_visit}')

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.End(vehicle_id)
        counter_dimension.SetCumulVarSoftLowerBound(index, nb_visit, 100)
        counter_dimension.SetCumulVarSoftUpperBound(index, nb_visit + 1, 100)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        if from_node in stations and to_node in stations:
            return data["fuel_capacity"]*5
        elif from_node == 0 and to_node == 8:
            return data["fuel_capacity"]*10
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        data["fuel_capacity"]*7,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Fuel constraints
    def fuel_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return -data["distance_matrix"][from_node][to_node]

    fuel_callback_index = routing.RegisterTransitCallback(fuel_callback)
    routing.AddDimension(
        fuel_callback_index,
        data["fuel_capacity"],
        data["fuel_capacity"],
        False,
        'Fuel')

    refuel_penalty = 0
    fuel_dimension = routing.GetDimensionOrDie('Fuel')
    for vehicle_id in range(data["num_vehicles"]):
        fuel_dimension.SlackVar(routing.Start(vehicle_id)).SetValue(0)
        for node in range(len(data["distance_matrix"])):
            if node == 0 or node == 8:
                continue
            if node > 8:
                index = manager.NodeToIndex(node)
                fuel_dimension.SlackVar(index).SetValue(0)
                routing.AddVariableMinimizedByFinalizer(fuel_dimension.CumulVar(node))
            else:
                index = manager.NodeToIndex(node)
                routing.AddDisjunction([index], refuel_penalty)
                fuel_dimension.CumulVar(index).SetValue(0)

    # Time
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        if from_node in [1, 2, 3, 4, 5, 6, 7]:
            return 300 + int(data["distance_matrix"][from_node][to_node] / data["vehicle_speed"])
        else:
            return 300 + int(data["distance_matrix"][from_node][to_node] / data["vehicle_speed"])

    time_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.AddDimension(
        time_callback_index,
        60,
        50000,
        False,
        'Time')

    time_dimension = routing.GetDimensionOrDie('Time')
    for location_idx, time_window in enumerate(data["time_windows"]):
        if location_idx == 0 or location_idx == 8:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        routing.AddToAssignment(time_dimension.SlackVar(index))
    # Add time window constraints for each vehicle start node
    # and "copy" the slack var in the solution object (aka Assignment) to print it
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data["time_windows"][0][0], data["time_windows"][0][1])
        routing.AddToAssignment(time_dimension.SlackVar(index))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # search_parameters.log_search = True
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(10)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        total_distance = print_solution(data, manager, routing, solution)
        distance_list.append(total_distance)
    else:
        print("-------------------------------No solution found-----------------------------")

    print("Solver status:", routing.status())


if __name__ == '__main__':
    main()
