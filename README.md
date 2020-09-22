# Mapping Application

This project is a mapping application.  The front end of this application uses
the Google Maps API:

![Map Application][image1]

## Class Design and Basic Graph Search

**Class:** MapGraph

**Modifications made to MapGraph:**

All data relating to the graph structure is stored primarily as a
`Map<GeographicPoint, ArrayList<MapGraphEdge>>` object in the `adjListsMap`
`private` data field in this class.  To explore nodes in the graph during
breadth-first search, it is necessary to find the neighbors of a node in the
graph.  Nodes in the graph represent geographic locations, so nodes are well
represented by `GeographicPoint` objects which are keys in `adjListsMap`.  The
neighbors of a node in the graph can be accessed through the list of
`MapGraphEdge` objects to which a `GeographicPoint` object maps.  Additionally,
the number of vertices and the number of edges in the graph are stored as
`private` `int` data fields in the `MapGraph` class to make it easier to
retrieve the number of vertices and edges in the graph.  It is easy to keep
track of these counts by incrementing the associated data fields whenever a
vertex or edge is added to the graph since the graph is always initialized to
be empty.

**Class name:** MapGraphEdge

**Purpose and description of class:**

This class is designed to store information about an edge in the graph
associated with the `MapGraph` class.  This class is necessary because edges
in the graph have additional information associated with them aside from the
start and end vertices, such as `roadName`, `roadType`, and `length`.  For this
class to work with the `Map<GeographicPoint, ArrayList<MapGraphEdge>>` object
in order to perform breadth-first search, it was important for the
`MapGraphEdge` class to provide access to the end vertex of the edge.  This
access is provided by the `getEnd` method in the `MapGraphEdge` class.

**Overall Design Justification:**

The overall design was motivated by the desire to find an efficient representation
for the graph structure.  An adjacency list representation was chosen for the
graph because each vertex in the graph is connected to only a small number of
other vertices when compared to the overall number of vertices in the graph.
Because nodes in the graph had only location information associated with them,
the `GeographicPoint` class was chosen as a representation for nodes in the
graph.  The representation of edges in the graph was more complicated because
edges had more information associated with them than just the start and end
nodes.  Edges also had `roadName`, `roadType`, and `length` associated with
them.  For this reason, the `MapGraphEdge` class was designed to preserve this
information.

**Testing the Implementation**

Here is an example of a map representing intersections around the University of
California, San Diego campus loaded into the mapping application:

![Load Map][image2]

This is the route returned by breadth-first search between two intersections
from the map above:

![Route][image3]

Here's a [link to a visualization](./Map_App_Visualization.mp4) which shows the
order in which the nodes were explored during breadth-first search.

## Shortest Path

**Testing the Implementation**

Here's a [link to a visualization](./Dijkstra_Visualization.mp4) which shows the
order in which the nodes were explored during Dijkstra's algorithm.

Here's a [link to a visualization](./A_Star_Visualization.mp4) which shows the
order in which the nodes were explored during A* search.

[image1]: Map_App.png
[image2]: Map_App_Load_Map.png
[image3]: Map_App_Route.png
