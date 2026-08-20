# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "ortools>=9.14",
#     "osmnx[neighbors,visualization]>=2.0",
#     "networkx>=3.0",
#     "numpy>=2.0",
#     "shapely>=2.0",
# ]
# ///

from ortools.constraint_solver import pywrapcp, routing_enums_pb2

import shapely.geometry
from shapely.ops import substring

import matplotlib.pyplot as plt
import numpy as np
import osmnx as ox
import networkx as nx
import xml.etree.ElementTree as ET

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

def edge_geometry(graph, u, v, data):
    """
    Returns the geometry of an edge, constructing it if necessary.
    OSMnx only stores a geometry for edges that are not a straight line between
    their end nodes, so for the rest we build that line from the node positions.
    """
    if 'geometry' in data:
        return data['geometry']
    return shapely.geometry.LineString([
        (graph.nodes[u]['x'], graph.nodes[u]['y']),
        (graph.nodes[v]['x'], graph.nodes[v]['y']),
    ])

def add_node_on_edge(graph, x, y):
    """
    Inserts a node at the point on the nearest edge closest to (x, y), splitting
    every edge between that edge's end nodes in both directions. Coordinates are
    in the graph's projected CRS.
    Returns the new node, or None if the point projects onto an existing node.
    """
    u, v, key = ox.nearest_edges(graph, x, y)
    line = edge_geometry(graph, u, v, graph.edges[u, v, key])
    offset = line.project(shapely.geometry.Point(x, y))
    if not 0 < offset < line.length:
        return None  # the closest point is an end node, so there is nothing to split
    # Take the split point from the geometry itself so that it lies exactly on the line
    node_x, node_y = substring(line, 0, offset).coords[-1]
    new_node = max(graph.nodes) + 1
    graph.add_node(new_node, x=node_x, y=node_y)

    point = shapely.geometry.Point(node_x, node_y)
    for a, b in ((u, v), (v, u)):
        for k, data in list((graph.get_edge_data(a, b) or {}).items()):
            geometry = edge_geometry(graph, a, b, data)
            along = geometry.project(point)
            if not 0 < along < geometry.length:
                continue
            head = substring(geometry, 0, along)
            tail = substring(geometry, along, geometry.length)
            # Divide the recorded length in proportion to the geometry so that
            # the two new lengths still sum to the length of the original edge
            scale = data['length'] / geometry.length
            attributes = {n: w for n, w in data.items() if n not in ('geometry', 'length')}
            graph.remove_edge(a, b, k)
            graph.add_edge(a, new_node, geometry=head,
                           length=head.length * scale, **attributes)
            graph.add_edge(new_node, b, geometry=tail,
                           length=tail.length * scale, **attributes)
    return new_node

def plot_controls(ax, points, visited):
    """
    Draws the controls on top of a plotted graph, distinguishing the start and
    finish from the rest, and the controls the route visits from those it misses.
    Points are keyed by control code and given in the graph's projected CRS.
    """
    def scatter(names, label, **style):
        if names:
            ax.scatter([points[name].x for name in names],
                       [points[name].y for name in names],
                       label=label, zorder=3, **style)

    controls = [name for name in points if name not in ('S1', 'F1')]
    scatter([name for name in points if name in ('S1', 'F1')],
            'Start/finish', marker='*', s=350, c='#ff00ff')
    scatter([name for name in controls if name in visited],
            'Control on route', marker='o', s=70, c='#ff00ff')
    scatter([name for name in controls if name not in visited],
            'Control not on route', marker='o', s=70,
            facecolors='none', edgecolors='#ff00ff', linewidths=1.5)

    # Group the labels by position so that controls sharing one, such as a start
    # and finish in the same place, do not have their codes drawn on top of one
    # another
    labels = {}
    for name, point in points.items():
        labels.setdefault((round(point.x, 1), round(point.y, 1)), []).append(name)
    for (x, y), names in labels.items():
        ax.annotate('/'.join(names), (x, y), textcoords='offset points',
                    xytext=(7, 7), color='white', fontsize=8, zorder=4,
                    bbox=dict(boxstyle='round,pad=0.15', facecolor='#111111',
                              edgecolor='none', alpha=0.75))

    ax.legend(loc='upper right', facecolor='#111111',
              labelcolor='white', framealpha=0.8)


control_coordinates = parse_kml_coordinates('course.kml')
control_codes = list(control_coordinates.keys())

# Download the street network for the area and project it so that all the
# geometry and nearest-neighbour searches below work in metres
graph = ox.project_graph(ox.graph_from_bbox(bbox(control_coordinates.values())))

# Project the control coordinates into the same CRS as the graph
control_points, _ = ox.projection.project_geometry(
    shapely.geometry.MultiPoint(list(control_coordinates.values())),
    to_crs=graph.graph['crs'])
control_points = dict(zip(control_codes, control_points.geoms))

# Find the nearest node in the graph for each placemark
control_nodes = {}
for name, point in control_points.items():
    node, distance = ox.nearest_nodes(graph, point.x, point.y, return_dist=True)
    if distance > 10:
        add_node_on_edge(graph, point.x, point.y)
        node, distance = ox.nearest_nodes(graph, point.x, point.y, return_dist=True)
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

max_distance = 9_000  # distance in metres

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
    print(f"Route: {' -> '.join(route)}")
    print(f"Score: {total_score}")
    print(f"Distance: {total_distance:.2f} metres")

    route_paths = [shortest_paths[plan[i], plan[i+1]] for i in range(len(plan)-1)]
    # orig_dest_size=0 suppresses the marker OSMnx puts at each end of every leg,
    # leaving plot_controls to mark the controls in one consistent style
    figure, ax = ox.plot_graph_routes(graph, route_paths, orig_dest_size=0,
                                      show=False, close=False)
    plot_controls(ax, control_points, {control_codes[i] for i in plan})
    plt.show()

else:
    print("No solution found!")