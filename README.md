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

## Project Extension

**Describe what you did with your extension to someone who will just be using**
**your application and doesn't know anything about programming.**

My project extension finds a route starting from a set of home coordinates,
which are supplied by the user, that visits a set of target coordinates,
sequentially, before returning to the home coordinates.  The application finds a
route which always visits the next closest set of coordinates to the current set
of coordinates.  Also, this route never visits a set of coordinates from the
target set of coordinates more than once, if such a path exists.

**Describe what you did with your extension to someone else on the same**
**project team.  What problems did you encounter and how did you fix them?**

For my project extension, I added the `public` `tsp` method to the
`MapGraph` class.  The `tsp` method solves the Travelling Salesperson Problem
using the greedy algorithm.  The `tsp` method takes as input a
`HashSet<GeographicPoint`, which are a set of nodes to visit, and a
`GeographicPoint` object, which designates the home location.  All
`GeographicPoint` objects which are passed to the `tsp` method must have already
been added to the graph, represented by the `MapGraph` class, as vertices.  The
`tsp` method returns a `List<GeographicPoint>` to represent the route, starting
at the home vertex, which visits each vertex in the target set before returning
to the home vertex.  The route which is returned will not pass through any of
the vertices in the target set more than once.  If no such route exists, the
`tsp` method just returns an empty `List<GeographicPoint>`.

The greatest challenge I encountered while completing my project extension was
choosing the representation for the set of nodes that I still needed to visit.
In my initial approach, I considered using a `PriorityQueue` as I had in the
implementation of A* Search which I wrote the previous week.  I felt that the
advantage of using a `PriorityQueue` was that I could pop the next node to visit
off the front of the queue and the node returned would always be the next
closest node because a `PriorityQueue` returns elements in a sorted order.  One
of the problems I found with that approach was that, every time I popped a node
off the queue and changed the current node, I had to recalculate all the
distances of the nodes that I still needed to visit from the current node, which
had changed.  Because this distance was the value that I was using to determine
the order of my `PriorityQueue`, I was basically having to resort my
`PriorityQueue` every time I changed the current node.  I found that a better
solution was to just keep track of the nodes I still needed to visit using a
`HashSet` and just iterating through the elements in the `HashSet` and keeping
track of the closest next node to the current node as I iterated.

Another challenge I faced was finding the distance from the current node to a
potential next node.  The `aStarSearch` method I authored last week just
returned the shortest path from the start node to the goal node.  This method
did not tell me how long the path was, so I couldn't compare the length of one
path to one potential next destination to a different path to a different next
potential destination.  I could have just iterated through the path returned by
my `aStarSearch` method, but it would take time to iterate through each path
every time I called `aStarSearch`.  Additionally, due to the adjacency list
representation that I was using for my graph, finding the length of even one
edge in my graph would involve looping through all the edges of a given vertex
just to find the length of one edge.  I found a better approach for finding the
length of the path returned by `aStarSearch` when I realized that by the time
`aStarSearch` finishes, I already know the length of the path.  All I had to do
was store that length in a new field in the `MapGraph` class that I called
`pathLength`.  Then, I could just retrieve that path length after each call to
`aStarSearch`.

A final challenge I encountered was keeping track of the nodes in the `toVisit`
`HashSet`, so I didn't return a path that passes through a node in this set more
than once.  I was able to handle this challenge because the `aStarSearch`
method I authored last week already used a `visited` `HashSet` to keep track of
nodes that had already been explored.  All I had to do was modify the
`weightedGraphSearch` method which performs the graph search for `aStarSearch`
and `dijkstra`, so that I could pass in a `HashSet` to keep track of visited
nodes.  Because the `HashSet` which is passed to the `weightedGraphSearch`
method does not have to be empty, I can keep track of visited nodes outside of
the `weightedGraphSearch` method.  The `weightedGraphSearch` method can then
append to the `HashSet` which is passed in to keep track of nodes visited during
the search.  I also had to make sure not to alter the `HashSet` that I pass to
`weightedGraphSearch` to keep track of visited nodes because I didn't want to
add every node visited by `weightedGraphSearch` to the set of nodes that I
didn't want `tsp` to visit more than once, just nodes that were also target
destinations for `tsp` (the nodes in the `toVisit` `HashSet`).  For this reason,
I made a copy of the `HashSet` passed in to `weightedGraphSearch` (line 360 in
MapGraph.java).

[image1]: Map_App.png
[image2]: Map_App_Load_Map.png
[image3]: Map_App_Route.png
