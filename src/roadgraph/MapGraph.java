/**
 * @author Hamadi McIntosh
 * @author UCSD MOOC development team
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.LinkedList;
import java.util.Queue;
import java.util.HashSet;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author Hamadi McIntosh
 * @author UCSD MOOC development team
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	
	private Map<GeographicPoint, MapGraphNode> adjListsMap;
	private int numVertices;
	private int numEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		adjListsMap = new HashMap<GeographicPoint, MapGraphNode>();
		numVertices = 0;
		numEdges = 0;
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return adjListsMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// If location is null, do not change the graph
		if (location == null) {
			return false;
		}
		
		// If location is already in the graph, do not change
		if (adjListsMap.containsKey(location)) {
			return false;
		}
		
		adjListsMap.put(location, new MapGraphNode(location));
		numVertices++;
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		if (!(adjListsMap.containsKey(from) && adjListsMap.containsKey(to))) {
			throw new IllegalArgumentException("Both GeographicPoints have not "
					+ "already been added as nodes to the graph");
		}
		
		if (from == null || to == null || roadName == null || roadType == null) {
			throw new IllegalArgumentException("One or more argument(s) are null");
		}
		
		if (length < 0) {
			throw new IllegalArgumentException("Length is less than zero");
		}
		
		MapGraphEdge newEdge = new MapGraphEdge(adjListsMap.get(from),
												adjListsMap.get(to),
												roadName, roadType, length);
		adjListsMap.get(from).addEdge(newEdge);
		numEdges++;
		
	}
	
	/**
	 * Generate string representation of the graph
	 * @return the String
	 */
	public String toString() {
		String toReturn = "";
		for (GeographicPoint loc : adjListsMap.keySet()) {
			toReturn += "Node:\n\n";
			toReturn += loc + "\n\n";
			toReturn += "Edges:\n\n";
			for (MapGraphEdge edge : adjListsMap.get(loc).getEdges()) {
				toReturn += edge + "\n\n";
			}
		}
		return toReturn;
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		HashMap<GeographicPoint, GeographicPoint> parentMap =
				new HashMap<GeographicPoint, GeographicPoint>();
		
		// System.out.println("Size of parent map: " + parentMap.size());
		// System.out.println();
		
		// System.out.println("Parent Map:");
		// printParentMap(parentMap);
		// System.out.println();
		
		Boolean found = bfsSearch(start, goal, parentMap, nodeSearched);
		
		// System.out.println("Size of parent map: " + parentMap.size());
		// System.out.println();
		
		
		// System.out.println("Parent map:");
		// printParentMap(parentMap);
		// System.out.println();
		
		if (!found) {
			return new LinkedList<GeographicPoint>();
		}

		return constructPath(start, goal, parentMap);
	}
	
	private void printParentMap(HashMap<GeographicPoint, GeographicPoint> parentMap) {
		for (GeographicPoint key : parentMap.keySet()) {
			System.out.println("Node: " + key);
			System.out.println("Parent: " + parentMap.get(key));
			System.out.println();
		}
	}
	
	/**
	 * Generate the path from start to goal using the parentMap
	 * @param start The starting location
	 * @param goal The goal location
	 * @param parentMap A mapping from each node searched in the graph to the node from
	 * 	which it was discovered during the search.
	 * @return The list of intersections that form the shortest (unweighted)
	 * 	path from start to goal (including both start and goal).
	 */
	private List<GeographicPoint> constructPath(GeographicPoint start,
			GeographicPoint goal, HashMap<GeographicPoint, GeographicPoint> parentMap) {
		
		// System.out.println("Construct path");
		// System.out.println();
		
		// System.out.println("Start:");
		// System.out.println(start);
		// System.out.println();
		
		// System.out.println("Goal:");
		// System.out.println(goal);
		// System.out.println();
		
		// System.out.println("Size of parent map: " + parentMap.size());
		// System.out.println();
		
		// System.out.println("Parent map:");
		// printParentMap(parentMap);
		// System.out.println();
		
		LinkedList<GeographicPoint> foundPath = new LinkedList<GeographicPoint>();
		
		GeographicPoint curr = goal;
		
		// Count iterations
		// int i = 0;
		
		// System.out.println("Start loop");
		// System.out.println();
		
		while (curr != start) {
			
			// i++;
			// System.out.println("Loop iteration: " + i);
			// System.out.println();
			
			// System.out.println("Curr:");
			// System.out.println(curr);
			// System.out.println();
			
			// System.out.println("Goal:");
			// System.out.println(start);
			// System.out.println();
			
			foundPath.addFirst(curr);
			curr = parentMap.get(curr);
			
			// System.out.println("New curr:");
			// System.out.println(curr);
			// System.out.println();
			
		}
		
		foundPath.addFirst(start);
		return foundPath;
	}
	
	private void printQueue(Queue<GeographicPoint> toExplore) {
		for (GeographicPoint n : toExplore) {
			System.out.println(n);
			System.out.println();
		}
	}
	
	/**
	 * Perform breadth-first search
	 * @param start The starting location
	 * @param goal The goal location
	 * @param parentMap A mapping from each node searched in the graph to the node from
	 * 	which it was discovered during the search.
	 * @param nodeSearched A hook for visualization.
	 * @return True if a path from start to goal was found, false otherwise.
	 */
	private Boolean bfsSearch(GeographicPoint start, GeographicPoint goal,
			HashMap<GeographicPoint, GeographicPoint> parentMap,
			Consumer<GeographicPoint> nodeSearched) {
		
		Queue<GeographicPoint> toExplore = new LinkedList<GeographicPoint>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		Boolean found = false;
		
		// Count iterations
		// int i = 0;
		
		toExplore.add(start);
		
		visited.add(start);
		
		// System.out.println("Start loop");
		// System.out.println();
		
		while(!toExplore.isEmpty()) {
			
			// Outer loop
			// i++;
			// System.out.println("Loop iteration: " + i);
			// System.out.println();
			
			// System.out.println("Queue:");
			// printQueue(toExplore);
			// System.out.println();
			
			GeographicPoint curr = toExplore.remove();
			
			// System.out.println("Curr:");
			// System.out.println(curr);
			// System.out.println();
			// System.out.println("Queue:");
			// printQueue(toExplore);
			// System.out.println();
			
			nodeSearched.accept(curr);
			
			// System.out.println("Goal:");
			// System.out.println(goal);
			// System.out.println();
			
			// System.out.println("End loop?");
			// System.out.println(curr.equals(goal));
			// System.out.println();
			
			if (curr.equals(goal)) {
				
				// System.out.println("End of loop");
				// System.out.println();
				
				found = true;
				break;
			}
			
			for (MapGraphEdge edge : adjListsMap.get(curr).getEdges()) {
				
				// Inner loop
				// i++;
				// System.out.println("Inner loop iteration: " + i);
				// System.out.println();
				
				MapGraphNode n = edge.getEnd();
				
				// System.out.println("New node:");
				// System.out.println(n);
				// System.out.println();
				
				if (!visited.contains(n.getLoc())) {
					
					visited.add(n.getLoc());
					parentMap.put(n.getLoc(), curr);
					toExplore.add(n.getLoc());
					
					// System.out.println("Queue:");
					// printQueue(toExplore);
					// System.out.println();
					
					// System.out.println("Size of parent map: " + parentMap.size());
					// System.out.println();
					
					// System.out.println("Parent Map:");
					// printParentMap(parentMap);
					// System.out.println();
					
					// System.out.println("Retrieve from parent map:");
					// System.out.println(parentMap.get(n.getLoc()));
					// System.out.println();
					
				}
			}
		}
		
		// System.out.println("End of method");
		// System.out.println();
		
		return found;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		// GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		// GraphLoader.loadRoadMap("data/graders/mod2/map1.txt", firstMap);
		// GraphLoader.loadRoadMap("data/graders/mod2/map2.txt", firstMap);
		// GraphLoader.loadRoadMap("data/graders/mod2/map3.txt", firstMap);
		GraphLoader.loadRoadMap("data/graders/mod2/ucsd.map", firstMap);
		System.out.println("DONE.");
		
		System.out.println();
		
		// System.out.println(firstMap);
		
		
		System.out.println(firstMap.getNumVertices() + " vertices");
		System.out.println(firstMap.getNumEdges() + " edges");
		
		/*
		System.out.println();
		System.out.println("Vertices: ");
		for (GeographicPoint v : firstMap.getVertices()) {
			System.out.println(v);
		}
		*/
		
		
		
		// GeographicPoint firstTestStart = new GeographicPoint(1.0, 1.0);
		// GeographicPoint firstTestEnd = new GeographicPoint(8.0, -1.0);
		// GeographicPoint firstTestStart = new GeographicPoint(0.0, 0.0);
		// GeographicPoint firstTestEnd = new GeographicPoint(6.0, 6.0);
		// GeographicPoint firstTestStart = new GeographicPoint(6.0, 6.0);
		// GeographicPoint firstTestEnd = new GeographicPoint(0.0, 0.0);
		// GeographicPoint firstTestStart = new GeographicPoint(0.0, 0.0);
		// GeographicPoint firstTestEnd = new GeographicPoint(1.0, 2.0);
		GeographicPoint firstTestStart = new GeographicPoint(32.8756538, -117.2435715);
		GeographicPoint firstTestEnd = new GeographicPoint(32.8742087, -117.2381344);
		List<GeographicPoint> firstTestRoute = firstMap.bfs(firstTestStart, firstTestEnd);
		
		
		System.out.println();
		System.out.println("Start: " + firstTestStart);
		System.out.println("Goal: " + firstTestEnd);
		System.out.println();
		System.out.println("Path:");
		for (GeographicPoint curr : firstTestRoute) {
			System.out.println(curr);
		}
		
		
		
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
