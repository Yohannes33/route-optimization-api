import json
import math
from haversine import haversine
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def load_json(filepath):
    """Load JSON data from a file."""
    with open(filepath, 'r') as f:
        return json.load(f)

def save_json(filepath, data):
    """Save JSON data to a file."""
    with open(filepath, 'w') as f:
        json.dump(data, f, indent=4)

def calculate_bearing(pointA, pointB):
    """
    Calculates the bearing from pointA to pointB.

    Args:
        pointA (List[float]): [latitude, longitude] of the first point.
        pointB (List[float]): [latitude, longitude] of the second point.

    Returns:
        float: Bearing in degrees from pointA to pointB.
    """
    lat1, lon1 = math.radians(pointA[0]), math.radians(pointA[1])
    lat2, lon2 = math.radians(pointB[0]), math.radians(pointB[1])
    dLon = lon2 - lon1

    x = math.sin(dLon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dLon))

    initial_bearing = math.atan2(x, y)
    initial_bearing_deg = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing_deg + 360) % 360

    return compass_bearing

def calculate_distance_and_bearing_matrix(locations):
    """
    Generates both distance and bearing matrices based on coordinates.

    Args:
        locations (List[List[float]]): List of [latitude, longitude] coordinates.

    Returns:
        Tuple[List[List[float]], List[List[float]]]: Distance matrix and bearing matrix.
    """
    distance_matrix = []
    bearing_matrix = []

    for i, loc1 in enumerate(locations):
        distance_row = []
        bearing_row = []
        for j, loc2 in enumerate(locations):
            if i == j:
                distance = 0
                bearing = 0
            else:
                distance = haversine(loc1, loc2)
                bearing = calculate_bearing(loc1, loc2)
            distance_row.append(distance)
            bearing_row.append(bearing)
        distance_matrix.append(distance_row)
        bearing_matrix.append(bearing_row)
    return distance_matrix, bearing_matrix

def assign_employees_to_shuttles(locations, distance_matrix, bearing_matrix, shuttle_capacities):
    num_locations = len(locations)
    num_shuttles = len(shuttle_capacities)

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(num_locations, num_shuttles, 0)

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Define the custom cost callback incorporating distance and bearing change and implement the penalty
    def combined_cost_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        distance = distance_matrix[from_node][to_node]
        # Retrieve bearing from 'from_node' to 'to_node'
        bearing = bearing_matrix[from_node][to_node]
        bearing_penalty = 10 
        # Simple addition of penalty; can be refined based on bearing changes
        return int(distance * 1000 + bearing_penalty)

    combined_transit_callback_index = routing.RegisterTransitCallback(combined_cost_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(combined_transit_callback_index)

    # Add capacity constraints
    demand = [0] + [1] * (num_locations - 1)  # HQ has no demand

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return demand[from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # No slack
        shuttle_capacities,
        True,  # Start cumul to zero
        "Capacity"
    )

    # Solve the problem - Uses PATH_CHEAPEST_ARC strategy to find initial solution
    #Applies Guided Local Search to improve the solution
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.FromSeconds(30)  # Adjust time limit as needed

    solution = routing.SolveWithParameters(search_parameters)

    # Extract the routes
    if solution:
        routes = []
        for vehicle_id in range(num_shuttles):
            index = routing.Start(vehicle_id)
            route = []
            while not routing.IsEnd(index):
                node = manager.IndexToNode(index)
                route.append(node)
                index = solution.Value(routing.NextVar(index))
            routes.append(route)
        return routes
    else:
        return None

def verify_unique_assignments(routes, num_employees):
    """
    Verifies that each employee is uniquely assigned to exactly one shuttle.

    Args:
        routes (List[List[int]]): The list of routes for each shuttle.
        num_employees (int): The total number of employees.

    Returns:
        bool: True if all employees are uniquely assigned, False otherwise.
    """
    assigned_employees = []
    for shuttle_id, route in enumerate(routes, start=1):
        # Exclude the HQ (node 0) from assignment
        employee_nodes = route[1:]
        assigned_employees.extend(employee_nodes)
        print(f"Shuttle {shuttle_id} assigned employees: {employee_nodes}")

    # Check for duplicates
    duplicates = set([x for x in assigned_employees if assigned_employees.count(x) > 1])
    if duplicates:
        print(f"Duplicate assignments found for employees: {duplicates}")
        return False

    # Check for missing assignments
    expected_employees = set(range(1, num_employees + 1))
    assigned_set = set(assigned_employees)
    missing = expected_employees - assigned_set
    if missing:
        print(f"Missing assignments for employees: {missing}")
        return False

    print("All employees are uniquely assigned to shuttles.")
    return True

def main():
    # Load input data
    data = load_json("data/locations.json")
    capacities = [int(cap) for cap in load_json("data/shuttle_capacities.json")]

    # Prepare locations
    HQ = data["HQ"]
    employees = data["employees"]
    locations = [HQ] + employees

    # Calculate distance and bearing matrices
    distance_matrix, bearing_matrix = calculate_distance_and_bearing_matrix(locations)

    # Assign employees to shuttles
    routes = assign_employees_to_shuttles(locations, distance_matrix, bearing_matrix, capacities)

    if routes:
        print("Routes assigned successfully!")
        save_json("data/output_routes.json", routes)
        for i, route in enumerate(routes):
            assigned = len(route) - 1  # Exclude HQ
            print(f"Shuttle {i + 1}: {[locations[node] for node in route]} (Assigned: {assigned}, Capacity: {capacities[i]})")
            if assigned > capacities[i]:
                print(f"Shuttle {i + 1} is over capacity!")

        # Verify unique assignments
        num_employees = len(employees)
        verification_passed = verify_unique_assignments(routes, num_employees)
        if not verification_passed:
            print(" Verification failed: There are issues with shuttle assignments.")
        else:
            print("All assignments are verified successfully.")

    else:
        print("No solution found.")

    total_demand = len(locations) - 1  # Exclude HQ
    total_capacity = sum(capacities)
    if total_demand > total_capacity:
        print(f"Total demand ({total_demand}) exceeds total capacity ({total_capacity}).")

if __name__ == "__main__":
    main()
