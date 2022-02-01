"""This program performs the UGV optimization using Traveling Salesman Problem formulation.
Number of ground vehicles is 1. This program takes the centroid points obtained from the K-means clustering algorithm
as nodes for the ground vehicle and solves for the optimal route of UGV."""

import math
import K_means_clustering_2cluster_3clustdist_bounds_one
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def main(random_seed):
    location, mission_locs = K_means_clustering_2cluster_3clustdist_bounds_one.clustered_locations(random_seed)
    nodes = []
    for i in range(len(location)):
        nodes.append(location[i][0] + location[i][1])
    max_value = max(nodes)
    node = nodes.index(max_value)

    def create_data_model():
        data = {"locations": location, 'num_vehicles': 1, 'starts': [0], 'ends': [node]}
        return data

    def compute_euclidean_distance_matrix(locations):
        """Creates callback to return distance between points."""
        distances = {}
        for from_counter, from_node in enumerate(locations):
            distances[from_counter] = {}
            for to_counter, to_node in enumerate(locations):
                if from_counter == to_counter:
                    distances[from_counter][to_counter] = 0
                else:
                    # Euclidean distance
                    distances[from_counter][to_counter] = (int(
                        math.hypot((from_node[0] - to_node[0]),
                                   (from_node[1] - to_node[1]))))
        return distances

    def print_solution(data, manager, routing, solution):
        """Prints solution on console."""
        print('Objective: {}'.format(solution.ObjectiveValue()))
        index = routing.Start(0)
        plan_output = 'Route:\n'
        route_distance = 0
        dist_list = [(13200, 13200)]
        while not routing.IsEnd(index):
            plan_output += ' {} ->'.format(manager.IndexToNode(index))
            previous_index = index
            dist_list.append(data["locations"][index])
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
        plan_output += ' {}\n'.format(manager.IndexToNode(index))
        print(plan_output)
        dist_list.append((13200, 13200))
        plan_output += 'Objective: {}m\n'.format(route_distance)
        # for j in range(len(dist_list)):
        #     if j <= len(dist_list) - 2:
        #         distance = round(math.hypot((dist_list[j][0] - dist_list[j+1][0]), (dist_list[j][1] - dist_list[j+1][1])))
        #         print(distance/5280 * 1.61)

    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['locations']),
                                           data['num_vehicles'], data['starts'], data['ends'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    distance_matrix = compute_euclidean_distance_matrix(data['locations'])

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)

    def get_routes(solution, routing, manager):
        """Get vehicle routes from a solution and store them in an array."""
        # Get vehicle routes and store them in a two dimensional array whose
        # i,j entry is the jth location visited by vehicle i along its route.
        routes = []
        for route_nbr in range(routing.vehicles()):
            index = routing.Start(route_nbr)
            route = [manager.IndexToNode(index)]
            while not routing.IsEnd(index):
                index = solution.Value(routing.NextVar(index))
                route.append(manager.IndexToNode(index))
            routes.append(route)
        return routes

    routes = get_routes(solution, routing, manager)
    ugv_route = routes.pop(0)
    route_location = []
    # Display the routes.
    for i, route in enumerate(routes):
        print('Route', i, route)
    for j in range(len(ugv_route)):
        route_location.append(location[ugv_route[j]])
    return route_location, mission_locs


if __name__ == '__main__':
    main()
