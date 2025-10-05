from ortools.constraint_solver import pywrapcp, routing_enums_pb2

import shapely.geometry
from shapely.ops import split as shapely_split
from shapely.ops import snap

import numpy as np
import osmnx as ox
import networkx as nx
import xml.etree.ElementTree as ET
from math import radians, sin, cos, sqrt, atan2

def parse_kml_coordinates(filepath):
    """
    Parses a KML file and extracts coordinate pairs for each Placemark keyed by the name.
    Returns a dict: {placemark_name: (longitude, latitude)}
    """
    ns = {'kml': 'http://www.opengis.net/kml/2.2'}
    tree = ET.parse(filepath)
    root = tree.getroot()
    placemarks = {}
    for placemark in root.findall('.//kml:Placemark', ns):
        name_elem = placemark.find('kml:name', ns)
        coord_elem = placemark.find('.//kml:coordinates', ns)
        if name_elem is not None and coord_elem is not None:
            name = name_elem.text.strip()
            coords_text = coord_elem.text.strip()
            lon, lat, *_ = map(float, coords_text.split(','))
            placemarks[name] = (lon, lat)
    return placemarks

def bbox(coordinates, padding=0.005):
    """
    Calculate the bounding box for a set of placemarks.
    Returns (min_lon, min_lat, max_lon, max_lat).
    """
    lons = [lon for lon, _ in coordinates]
    lats = [lat for _, lat in coordinates]
    return (min(lons) - padding, min(lats) - padding,
            max(lons) + padding, max(lats) + padding)

def add_node_on_edge(graph, lon, lat):
    # Find nearest edge
    u, v, key = ox.nearest_edges(graph, lon, lat)
    edge_data = graph.get_edge_data(u, v, key)

    # Get geometry of the edge
    if 'geometry' not in edge_data: # TODO why does this happen?
        print(f"Edge {u}-{v} key {key} has no geometry, skipping")
        return
    line = edge_data['geometry']
    # Project placemark onto the edge
    placemark_point = shapely.geometry.Point(lon, lat)
    projected_point = line.interpolate(line.project(placemark_point))
    snapped_point = snap(projected_point, line, tolerance=0.0001)
    # Remove the original edge
    graph.remove_edge(u, v, key)

    # Repeat for the reverse direction
    u2, v2, key2 = ox.nearest_edges(graph, lon, lat)
    edge_data2 = graph.get_edge_data(u2, v2, key2)
    # Get geometry of the edge
    line2 = edge_data2['geometry']
    # Remove the original edge
    graph.remove_edge(u2, v2, key2)

    # Add new node at snapped_point
    new_node = max(graph.nodes) + 1  # or use a unique id
    graph.add_node(new_node, x=snapped_point.x, y=snapped_point.y)

    # Split the edge geometry
    lines = shapely_split(line, snapped_point).geoms
    if len(lines) != 2:
        print("Edge split did not produce two LineStrings") # TODO why?
        return
    # Add new edges
    graph.add_edge(u, new_node, length=lines[0].length, geometry=lines[0])
    graph.add_edge(new_node, v, length=lines[1].length, geometry=lines[1])

    # Split the edge geometry
    lines2 = shapely_split(line2, snapped_point).geoms
    if len(lines) != 2:
        print("Edge split did not produce two LineStrings") # TODO why?
        return
    # Add new edges
    graph.add_edge(u2, new_node, length=lines2[0].length, geometry=lines2[0])
    graph.add_edge(new_node, v2, length=lines2[1].length, geometry=lines2[1])


control_coordinates = parse_kml_coordinates('course.kml')
control_codes = list(control_coordinates.keys())

# Download the street network for the area
graph = ox.graph_from_bbox(bbox(control_coordinates.values()))

# Find the nearest node in the graph for each placemark
control_nodes = {}
for name, (lon, lat) in control_coordinates.items():
    node, distance = ox.nearest_nodes(graph, lon, lat, return_dist=True)
    if distance > 10:
        add_node_on_edge(graph, lon, lat)
        node, distance = ox.nearest_nodes(graph, lon, lat, return_dist=True)
    control_nodes[name] = node

# Build a distance matrix using network distances
size = len(control_codes)
distance_matrix = np.zeros((size, size))
shortest_paths = np.empty((size, size), dtype=object)

for i in range(size):
    for j in range(size):
        if i == j:
            distance_matrix[i, j] = 0
        else:
            node1 = control_nodes[control_codes[i]]
            node2 = control_nodes[control_codes[j]]
            try:
                path = nx.shortest_path(graph, node1, node2, weight='length')
                shortest_paths[i, j] = path
                length = nx.path_weight(graph, path, weight='length')
            except nx.NetworkXNoPath:
                length = 1e6  # Large penalty if no path exists
            distance_matrix[i, j] = length

# Find indices for S1 and F1
start_idx = control_codes.index('S1')
finish_idx = control_codes.index('F1')

max_distance = 9000  # distance in metres

def get_score(name):
    try:
        num = int(name)
        return ((num - 1) // 10 + 1) * 10
    except ValueError:
        return 0  # S1, F1, or other non-numeric names

scores = [get_score(name) for name in control_codes]

def create_data_model():
    data = {}
    data['distance_matrix'] = distance_matrix.astype(int).tolist()
    data['scores'] = scores
    data['num_vehicles'] = 1
    data['depot'] = start_idx
    data['end'] = finish_idx
    data['max_distance'] = max_distance
    return data

data = create_data_model()
manager = pywrapcp.RoutingIndexManager(
    len(data['distance_matrix']),
    data['num_vehicles'],
    [data['depot']],
    [data['end']]
)
routing = pywrapcp.RoutingModel(manager)

# Distance callback (for constraint)
def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return int(data['distance_matrix'][from_node][to_node])

transit_callback_index = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Add distance constraint
dimension_name = 'Distance'
routing.AddDimension(
    transit_callback_index,
    0,  # no slack
    data['max_distance'],  # maximum distance
    True,  # start cumul to zero
    dimension_name)
distance_dimension = routing.GetDimensionOrDie(dimension_name)

# Add a "score" dimension to maximize
def score_callback(from_index):
    node = manager.IndexToNode(from_index)
    return data['scores'][node]

score_callback_index = routing.RegisterUnaryTransitCallback(score_callback)
routing.AddDimension(
    score_callback_index,
    0,  # no slack
    100000,  # upper bound on total score (arbitrary large)
    True,
    "Score"
)
score_dimension = routing.GetDimensionOrDie("Score")

# Allow skipping nodes (except start/end), with penalty = node score
for node in range(len(data['distance_matrix'])):
    if node not in (data['depot'], data['end']):
        routing.AddDisjunction([manager.NodeToIndex(node)], data['scores'][node] * 10000)

# Set search parameters
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
search_parameters.time_limit.seconds = 60

# Solve
solution = routing.SolveWithParameters(search_parameters)

if solution:
    index = routing.Start(0)
    plan = []
    total_score = 0
    total_distance = 0
    while not routing.IsEnd(index):
        node = manager.IndexToNode(index)
        plan.append(node)
        total_score += data['scores'][node]
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        total_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan.append(manager.IndexToNode(index))
    route = [control_codes[i] for i in plan]
    print(f"OR-Tools route: {' -> '.join(route)}")
    print(f"Total score: {total_score}")
    print(f"Total distance: {total_distance:.2f} metres")

    route_paths = [shortest_paths[plan[i], plan[i+1]] for i in range(len(plan)-1)]
    figure, ax = ox.plot_graph_routes(graph, route_paths)
    
else:
    print("No solution found!")