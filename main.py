"""Capacited Vehicles Routing Problem (CVRP)."""

from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from flask import Flask, request, Response, jsonify
from flask_cors import CORS

import json

app = Flask(__name__)
CORS(app)
app.config['JSON_SORT_KEYS'] = False


@app.route('/api/optimization', methods=['POST'])
def optimization():
    data_model = create_data_model(request.json)

    solution = solve_problem(data_model)

    return Response(json.dumps(solution), mimetype='application/json')


def create_data_model(problem):
    print('Creating data model...')
    data = {}

    data['distance_matrix'] = problem['durations']
    data['demands'] = list(map(lambda location: location['demand'], problem['metadata']['query']['locations']))

    data['vehicle_capacities'] = problem['metadata']['cars']
    data['num_vehicles'] = len(data['vehicle_capacities'])
    data['depot'] = 0

    data['locations'] = problem['metadata']['query']['locations']

    print('Created: ')
    print(data)
    return data


def get_solution(data, manager, routing, solution):
    json_response = {}
    """Prints solution on console."""
    total_distance = 0
    total_load = 0

    json_response['routes'] = []
    for vehicle_id in range(data['num_vehicles']):

        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0

        locations = []
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data['demands'][node_index]
            plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)

            locations.append(data['locations'][node_index])

            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += ' {0} Load({1})\n'.format(manager.IndexToNode(index),
                                                 route_load)

        locations.append(data['locations'][manager.IndexToNode(index)])
        json_response['routes'].append({'locations': locations})

        print(json_response)
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)

        plan_output += 'Load of the route: {}\n'.format(route_load)
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
    print('Total distance of all routes: {}m'.format(total_distance))
    print('Total load of all routes: {}'.format(total_load))
    json_response['total_distance'] = total_distance
    json_response['total_load'] = total_load
    print(json_response)

    return json_response


def solve_problem(data):
    """Solve the CVRP problem."""
    print('Solving problem')
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(1)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    print(solution)
    # Print solution on console.
    if solution:
        return get_solution(data, manager, routing, solution)


if __name__ == '__main__':
    from waitress import serve

    serve(app, host='0.0.0.0', port=5001)
