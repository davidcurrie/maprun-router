# Optimal Route Planning for OpenStreetMap-based MapRun Score Events

[osm-score.py](osm-score.py) uses the [OSMnx](https://osmnx.readthedocs.io/en/stable/) package to obtain OpenStreetMap data, [NetworkX](https://networkx.org/documentation/stable/index.html) to find the shortest path, and [Google OR-Tools](https://developers.google.com/optimization) to solve "the orienteering problem".

## Usage

As input, it expects a `course.kml` file of the format expected by [MapRun](https://maprunners.weebly.com/course-setting---kml-files.html), i.e. with placemarks S1 and F1 representing the start and finish respectively.
It assumes MapRun's ScoreNxx [scoring scheme](https://maprunners.weebly.com/scoring-schemes.html) (change `get_score` if using a different scheme).
Set `max_distance` to the maximum number of metres you want the route to take.

As output, the programme with list the order of controls to visit, the distance covered, and the score achieved.
It will also show a diagram of the network and selected route.

![Example output](output.png)

## Limitations

The following are known limitations:
* Readability is limited by the use of Copilot to generate the code!
* Some parts of the OSM network seem to contain edges that do not have corresponding lines.
* Routes can only follow the linear network: there's no cutting across open areas!
