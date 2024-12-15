#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Dec  8 11:48:57 2024

@author: andrea
"""

# Import packages
from flask import Flask, request, jsonify
from flask_cors import CORS
import googlemaps
import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# Initialise the Flask and Googlemaps API
WRApp = Flask(__name__)
CORS(WRApp)
API_KEY = "AIzaSyCqoW-Ycm_lNsnuG5bjLaBcKWCbyxIacYw"
gmaps = googlemaps.Client(key=API_KEY)

# Define solve_tsp function to calculate the optimised route (!!! Can use different AI methods to improve)
def solve_tsp(distance_matrix):
    """

    Parameters
    ----------
    Use OR-Tools to solve the TSP Problem given distance matrix

    Returns
    -------
    The order of to_index

    """
    # Set up URL index manager
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 1, 0)
    routing = pywrapcp.RoutingModel(manager)
    
    def distance_callback(from_index, to_index):
        # Transform the URL index to distance matrix node index
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(distance_matrix[from_node][to_node])
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    # Set up Searching Parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    
    # Solver
    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        route = []
        index = routing.Start(0)
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))
        print("Optimal route:", route)
        return route
    else:
        print("No solution found")
        return []

# Define Root URL
@WRApp.route('/')
def home():
    return "Plan Your Route!"

# Define URL: /optimise-route, using 'POST' method to receive the address list in JSON format
@WRApp.route('/optimise-route', methods=['POST'])
def optimise_route():
    if request.method == "OPTIONS":
        # Deal with OPTIONS request, maybe flask-cors will response automatically
        return jsonify({}), 200
    
    data = request.get_json()
    addresses = data.get('addresses', [])
    
    # Limit the number of address to 10
    if not addresses or len(addresses) > 10:
        return jsonify({"error": "Please provide between 1 and 10 addresses."}), 400
    
    # 1. Transform the addresses to geographical latitude and longitude
    coords = []
    for addr in addresses:
        geocode_result = gmaps.geocode(addr)
        if not geocode_result:
            print("Geocode failed:", addr)
            return jsonify({"error": f"Unable to geocode address: {addr}"}), 400
        latlng = geocode_result[0]["geometry"]["location"]
        coords.append((latlng["lat"], latlng["lng"]))
    
    print("coordinates:", coords)
    
    # 2. Get distance matrix in driving mode
    origins = coords
    destinations = coords
    matrix_result = gmaps.distance_matrix(origins, destinations, mode="driving")
    
    if matrix_result["status"] != "OK":
        print("Distance Matrix request failed")
        return jsonify({"error": "Distance Matrix request failed"}), 500
    
    n = len(addresses)
    distance_matrix = np.zeros((n, n))
    for i, row in enumerate(matrix_result["rows"]):
        for j, element in enumerate(row["elements"]):
            if element["status"] == "OK":
                # Use Driving Duration (second) as Cost to implement TSP Optimisation
                distance_matrix[i][j] = element["duration"]["value"]
            else:
                print("Unable to find route for given addresses.:", i, j, element.get('status'))
                return jsonify({"error": "Unable to find route for given addresses."}), 400
    
    print("Calculated distance matrix:", distance_matrix)
    
    # 3. Use OR-Tools to solve TSP
    route_order = solve_tsp(distance_matrix)
    if not route_order:
        print("No solution found for the given addresses")
        return jsonify({"error": "No solution found for the given addresses."}), 500
    
    optimised_addresses = [addresses[i] for i in route_order]
    optimised_coords = [coords[i] for i in route_order]
    
    # Return Result: the Order of Optimised Route
    return jsonify({
        "optimised_addresses": optimised_addresses,
        "route_order": route_order,
        "coordinates": optimised_coords
        })

if __name__ == '__main__':
    # Initiate under local developing mode
    WRApp.run(debug=True)
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        