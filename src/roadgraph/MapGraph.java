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
import java.util.PriorityQueue;

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
	private double pathLength;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		adjListsMap = new HashMap<GeographicPoint, MapGraphNode>();
		numVertices = 0;
		numEdges = 0;
		pathLength = Double.POSITIVE_INFINITY;
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
		return new HashSet(adjListsMap.keySet());
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numEdges;
	}

	public double getEdgeLength(GeographicPoint start, GeographicPoint goal) {
		MapGraphEdge e = getEdge(start, goal);
		return e.getLength();
	}
	
	public void setEdgeLength(GeographicPoint from, GeographicPoint to, double weight) {
		MapGraphEdge e = getEdge(from, to);
		e.setLength(weight);
	}
	
	private MapGraphEdge getEdge(GeographicPoint start, GeographicPoint goal)
			throws IllegalArgumentException {
			
			if (!(adjListsMap.containsKey(start) && adjListsMap.containsKey(goal))) {
				throw new IllegalArgumentException("Both GeographicPoints have not "
						+ "already been added as nodes to the graph");
			}
			
			if (start == null || goal == null) {
				throw new IllegalArgumentException("One or more argument(s) are null");
			}
			
			MapGraphNode startNode = adjListsMap.get(start);
			MapGraphEdge edge = null;
			for (MapGraphEdge e : startNode.getEdges()) {
				MapGraphNode curr = e.getEnd();
				GeographicPoint currLoc = curr.getLoc();
				if (currLoc.equals(goal)) {
					edge = e;
					break;
				}
			}
			return edge;
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
		
		Boolean found = bfsSearch(start, goal, parentMap, nodeSearched);
		
		if (!found) {
			return new LinkedList<GeographicPoint>();
		}

		return constructPath(start, goal, parentMap);
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
		
		LinkedList<GeographicPoint> foundPath = new LinkedList<GeographicPoint>();
		
		GeographicPoint curr = goal;
		
		while (!curr.equals(start)) {
			
			foundPath.addFirst(curr);
			curr = parentMap.get(curr);
			
		}
		
		foundPath.addFirst(start);
		return foundPath;
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
		
		toExplore.add(start);
		
		visited.add(start);
		
		while(!toExplore.isEmpty()) {
			
			GeographicPoint curr = toExplore.remove();
			
			nodeSearched.accept(curr);
			
			if (curr.equals(goal)) {
				
				found = true;
				break;
			}
			
			for (MapGraphEdge edge : adjListsMap.get(curr).getEdges()) {
				
				MapGraphNode n = edge.getEnd();
				
				if (!visited.contains(n.getLoc())) {
					
					visited.add(n.getLoc());
					parentMap.put(n.getLoc(), curr);
					toExplore.add(n.getLoc());
					
				}
			}
		}
		
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
		HashMap<GeographicPoint, GeographicPoint> parentMap =
				new HashMap<GeographicPoint, GeographicPoint>();
		
		boolean found = weightedGraphSearch(start, goal, parentMap, nodeSearched,
				true);
		
		if (!found) {
			return new LinkedList<GeographicPoint>();
		}

		return constructPath(start, goal, parentMap);
	}
	
	private boolean weightedGraphSearch(
			GeographicPoint start, GeographicPoint goal,
			HashMap<GeographicPoint, GeographicPoint> parentMap,
			Consumer<GeographicPoint> nodeSearched,
			boolean isDijkstra) {
		return weightedGraphSearch(start, goal, parentMap,
				new HashSet<GeographicPoint>(), nodeSearched, isDijkstra);
	}
	
	private boolean weightedGraphSearch(
			GeographicPoint start, GeographicPoint goal,
			HashMap<GeographicPoint, GeographicPoint> parentMap,
			HashSet<GeographicPoint> totVisited,
			Consumer<GeographicPoint> nodeSearched,
			boolean isDijkstra) {
		
		PriorityQueue<MapGraphNode> toExplore = new PriorityQueue<MapGraphNode>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>(totVisited);
		initDistance();
		boolean found = false;
		
		// Count iterations
		int i = 0;
		
		MapGraphNode startNode = adjListsMap.get(start);
		
		double fromSource = 0;
		double toTarget = targetDist(start, goal, isDijkstra);
		double predictDist = fromSource + toTarget;
		
		startNode.setDist(fromSource);
		startNode.setPredicted(predictDist);
		toExplore.add(new MapGraphNode(start, startNode.getEdges(),
										startNode.getDist(),
										startNode.getPredicted()));
		
		while(!toExplore.isEmpty()) {
			
			
			// Outer loop
			i++;
			
			GeographicPoint currLoc = toExplore.poll().getLoc();
			MapGraphNode currNode = adjListsMap.get(currLoc);
			double currDist = currNode.getDist();
						
			nodeSearched.accept(currLoc);
			
			if (!visited.contains(currLoc)) {
				visited.add(currLoc);
				if (currLoc.equals(goal)) {
					
					pathLength = currDist;
					
					found = true;
					break;
				}
				
				for (MapGraphEdge e : currNode.getEdges()) {
					
					double edgeDist = e.getLength();
					fromSource = currDist + edgeDist;
					
					GeographicPoint nLoc = e.getEnd().getLoc();
					MapGraphNode n = adjListsMap.get(nLoc);
					toTarget = targetDist(nLoc, goal, isDijkstra);
					
					predictDist = fromSource + toTarget;
					
					if (!visited.contains(nLoc)) {
						if (predictDist < n.getPredicted()) {
							n.setDist(fromSource);
							n.setPredicted(predictDist);
							parentMap.put(nLoc, currLoc);
							toExplore.add(new MapGraphNode(nLoc, n.getEdges(),
											fromSource, predictDist));
							
						}
					}
				}
				
			}
		}
		
		// Number of nodes visited
		System.out.println(i + " nodes visited");
		System.out.println();
		
		
		return found;
		
	}
	
	private double targetDist(GeographicPoint start, GeographicPoint goal,
			boolean isDijkstra) {
		double toTarget;
		if (isDijkstra) {
			toTarget = 0;
		}
		else {
			toTarget = start.distance(goal);
		}
		return toTarget;
	}
	
	public void initDistance() {
		for (MapGraphNode n : adjListsMap.values()) {
			n.setDist(Double.POSITIVE_INFINITY);
			n.setPredicted(Double.POSITIVE_INFINITY);
		}
		pathLength = Double.POSITIVE_INFINITY;
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
		
		HashMap<GeographicPoint, GeographicPoint> parentMap =
				new HashMap<GeographicPoint, GeographicPoint>();
		
		boolean found = weightedGraphSearch(start, goal, parentMap, nodeSearched,
				false);
		
		if (!found) {
			return new LinkedList<GeographicPoint>();
		}

		return constructPath(start, goal, parentMap);
	}
	
	public List<GeographicPoint> tsp(HashSet<GeographicPoint> targets,
			GeographicPoint home) {
		
		if (targets.contains(home)) {
			targets.remove(home);
		}
		
		List<GeographicPoint> bestPath = new LinkedList<GeographicPoint>();
		bestPath.add(home);
		GeographicPoint curr = home;
		HashSet<GeographicPoint> toVisit = new HashSet<GeographicPoint>(targets);
		HashSet<GeographicPoint> totVisited = new HashSet<GeographicPoint>();
		
		while (!toVisit.isEmpty()) {
			
			List<GeographicPoint> bestSubPath = null;
			GeographicPoint bestStop = null;
			double bestDist = Double.POSITIVE_INFINITY;
			
			for (GeographicPoint nextStop : toVisit) {
				
				List<GeographicPoint> currSubPath = tspPreVisit(curr, nextStop,
						totVisited);
				
				if (currSubPath.size() > 0) {
					
					double currDist = pathLength;
					
					if (currDist < bestDist) {
						
						bestSubPath = currSubPath;
						bestStop = nextStop;
						bestDist = currDist;
						
					}
				}
			}
			if (bestDist == Double.POSITIVE_INFINITY) {
				return new LinkedList<GeographicPoint>();
			}
			else {
				
				List<GeographicPoint> subPathRmFirst =
						new LinkedList<GeographicPoint>(bestSubPath);
				subPathRmFirst.remove(curr);
				bestPath.addAll(subPathRmFirst);
				
				List<GeographicPoint> subPathRmLast =
						new LinkedList<GeographicPoint>(bestSubPath);
				subPathRmLast.remove(bestStop);
				totVisited.addAll(subPathRmLast);
				
				for (GeographicPoint n : bestSubPath) {
					toVisit.remove(n);
				}
				
				curr = bestStop;
				
			}
			
		}
		
		totVisited.remove(home);
		List<GeographicPoint> currSubPath = tspPreVisit(curr, home, totVisited);
		
		if (currSubPath.size() > 0) {
			List<GeographicPoint> subPathRmFirst =
					new LinkedList<GeographicPoint>(currSubPath);
			subPathRmFirst.remove(curr);
			bestPath.addAll(subPathRmFirst);
		}
		else {
			return new LinkedList<GeographicPoint>();
		}
		
		return bestPath;
	}
	
	private List<GeographicPoint> tspPreVisit(GeographicPoint from,
			GeographicPoint to, HashSet<GeographicPoint> visited) {
		Consumer<GeographicPoint> nodeSearched = (x) -> {};
		HashMap<GeographicPoint, GeographicPoint> parentMap =
				new HashMap<GeographicPoint, GeographicPoint>();
		List<GeographicPoint> path = new LinkedList<GeographicPoint>();
		
		boolean found = weightedGraphSearch(from, to, parentMap,
				visited, nodeSearched, false);
		if (found) {
			path = constructPath(from, to, parentMap);
		}
		return path;
	}
	
	public void printRoute(GeographicPoint start, GeographicPoint goal,
			List<GeographicPoint> route) {
		System.out.println("Start: " + start);
		System.out.println("Goal: " + goal);
		System.out.println();
		System.out.println("Path:");
		for (GeographicPoint curr : route) {
			System.out.println(curr);
		}
	}
	
	public static void main(String[] args)
	{
		
		
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		// GraphLoader.loadRoadMap("data/graders/mod2/map1.txt", firstMap);
		// GraphLoader.loadRoadMap("data/graders/mod2/map2.txt", firstMap);
		// GraphLoader.loadRoadMap("data/graders/mod2/map3.txt", firstMap);
		// GraphLoader.loadRoadMap("data/graders/mod2/ucsd.map", firstMap);
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
		
		
		// data/testdata/simpletest.map
		GeographicPoint firstTestStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint firstTestEnd = new GeographicPoint(8.0, -1.0);
		
		// data/graders/mod2/map1.txt
		// GeographicPoint firstTestStart = new GeographicPoint(0.0, 0.0);
		// GeographicPoint firstTestEnd = new GeographicPoint(6.0, 6.0);
		
		// data/graders/mod2/map2.txt
		// GeographicPoint firstTestStart = new GeographicPoint(6.0, 6.0);
		// GeographicPoint firstTestEnd = new GeographicPoint(0.0, 0.0);
		
		// data/graders/mod2/map3.txt
		// GeographicPoint firstTestStart = new GeographicPoint(0.0, 0.0);
		// GeographicPoint firstTestEnd = new GeographicPoint(1.0, 2.0);
		
		// data/graders/mod2/ucsd.map
		// GeographicPoint firstTestStart = new GeographicPoint(32.8756538, -117.2435715);
		// GeographicPoint firstTestEnd = new GeographicPoint(32.8742087, -117.2381344);
		
		List<GeographicPoint> firstTestRoute = firstMap.bfs(firstTestStart, firstTestEnd);
		
		
		System.out.println();
		firstMap.printRoute(firstTestStart, firstTestEnd, firstTestRoute);
		
		
		
		/*
		MapGraph simpleTestMap = new MapGraph();
		// GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		// GraphLoader.loadRoadMap("data/graders/mod3/map1.txt", simpleTestMap);
		// GraphLoader.loadRoadMap("data/graders/mod3/map2.txt", simpleTestMap);
		// GraphLoader.loadRoadMap("data/graders/mod3/map3.txt", simpleTestMap);
		GraphLoader.loadRoadMap("data/graders/mod3/ucsd.map", simpleTestMap);
		
		// data/testdata/simpletest.map
		// GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		// GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		// data/graders/mod3/map1.txt
		// GeographicPoint testStart = new GeographicPoint(0, 0);
		// GeographicPoint testEnd = new GeographicPoint(6, 6);
		
		// data/graders/mod3/map2.txt
		// GeographicPoint testStart = new GeographicPoint(7, 3);
		// GeographicPoint testEnd = new GeographicPoint(4, -1);
		
		// data/graders/mod3/map3.txt
		// GeographicPoint testStart = new GeographicPoint(0, 0);
		// GeographicPoint testEnd = new GeographicPoint(0, 4);
		
		// data/graders/mod3/ucsd.map
		GeographicPoint testStart = new GeographicPoint(32.8709815, -117.2434254);
		GeographicPoint testEnd = new GeographicPoint(32.8742087, -117.2381344);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		System.out.println();
		System.out.println("Dijkstra");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		System.out.println("AStar");
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		System.out.println();
		System.out.println("Dijkstra");
		System.out.println();
		simpleTestMap.printRoute(testStart, testEnd, testroute);
		
		System.out.println();
		System.out.println("AStar");
		System.out.println();
		simpleTestMap.printRoute(testStart, testEnd, testroute2);
		*/
		
		/*
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		*/
		
		/*
		// A very simple test using real data
		GeographicPoint testStart = new GeographicPoint(32.869423, -117.220917);
		GeographicPoint testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		System.out.println();
		System.out.println("Dijkstra");
		List<GeographicPoint> testroute = testMap.dijkstra(testStart,testEnd);
		System.out.println("AStar");
		List<GeographicPoint> testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		/*
		// A slightly more complex test using real data
		GeographicPoint testStart = new GeographicPoint(32.8674388, -117.2190213);
		GeographicPoint testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		System.out.println();
		System.out.println("Dijkstra");
		List<GeographicPoint> testroute = testMap.dijkstra(testStart,testEnd);
		System.out.println("AStar");
		List<GeographicPoint> testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		/*
		// Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");
		System.out.println();

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		System.out.println("Dijkstra");
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		System.out.println("AStar");
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		*/
		
		/*
		// Code to test algorithm for Travelling Salesperson Problem
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		// GraphLoader.loadRoadMap("data/testdata/tspmap1.txt", firstMap);
		// GraphLoader.loadRoadMap("data/testdata/tspmap2.txt", firstMap);
		GraphLoader.loadRoadMap("data/testdata/tspmap3.txt", firstMap);
		System.out.println("DONE.");
		System.out.println();
		
		GeographicPoint tspNode1 = new GeographicPoint(0.0, 1.0);
		GeographicPoint tspNode2 = new GeographicPoint(1.0, 1.0);
		GeographicPoint tspNode3 = new GeographicPoint(1.0, 0.0);
		GeographicPoint tspNode4 = new GeographicPoint(0.0, 0.0);
		// For data/testdata/tspmap2.txt and data/testdata/tspmap3.txt
		GeographicPoint tspNode5 = new GeographicPoint(2.0, 1.0);
		// For data/testdata/tspmap3.txt
		GeographicPoint tspNode6 = new GeographicPoint(2.0, 0.0);
		*/
		
		/*
		// For data/testdata/tspmap1.txt
		firstMap.setEdgeLength(tspNode1, tspNode2, 5.0);
		firstMap.setEdgeLength(tspNode2, tspNode1, 5.0);
		firstMap.setEdgeLength(tspNode2, tspNode3, 7.0);
		firstMap.setEdgeLength(tspNode3, tspNode2, 7.0);
		firstMap.setEdgeLength(tspNode3, tspNode4, 10.0);
		firstMap.setEdgeLength(tspNode4, tspNode3, 10.0);
		firstMap.setEdgeLength(tspNode4, tspNode1, 7.0);
		firstMap.setEdgeLength(tspNode1, tspNode4, 7.0);
		firstMap.setEdgeLength(tspNode1, tspNode3, 6.0);
		firstMap.setEdgeLength(tspNode3, tspNode1, 6.0);
		firstMap.setEdgeLength(tspNode2, tspNode4, 6.0);
		firstMap.setEdgeLength(tspNode4, tspNode2, 6.0);
		*/
		
		/*
		// For data/testdata/tspmap2.txt and data/testdata/tspmap3.txt
		firstMap.setEdgeLength(tspNode1, tspNode2, 5.0);
		firstMap.setEdgeLength(tspNode2, tspNode1, 5.0);
		firstMap.setEdgeLength(tspNode2, tspNode3, 5.0);
		firstMap.setEdgeLength(tspNode3, tspNode2, 5.0);
		firstMap.setEdgeLength(tspNode3, tspNode4, 5.0);
		firstMap.setEdgeLength(tspNode4, tspNode3, 5.0);
		firstMap.setEdgeLength(tspNode4, tspNode1, 6.0);
		firstMap.setEdgeLength(tspNode1, tspNode4, 6.0);
		firstMap.setEdgeLength(tspNode1, tspNode3, 6.0);
		firstMap.setEdgeLength(tspNode3, tspNode1, 6.0);
		firstMap.setEdgeLength(tspNode2, tspNode4, 6.0);
		firstMap.setEdgeLength(tspNode4, tspNode2, 6.0);
		firstMap.setEdgeLength(tspNode2, tspNode5, 4.0);
		firstMap.setEdgeLength(tspNode5, tspNode2, 4.0);
		// For data/testdata/tspmap3.txt
		firstMap.setEdgeLength(tspNode5, tspNode6, 10.0);
		firstMap.setEdgeLength(tspNode6, tspNode5, 10.0);
		firstMap.setEdgeLength(tspNode6, tspNode3, 5.0);
		firstMap.setEdgeLength(tspNode3, tspNode6, 5.0);
		
		HashSet<GeographicPoint> targets = new HashSet<GeographicPoint>();
		targets.add(tspNode2);
		targets.add(tspNode3);
		targets.add(tspNode4);
		targets.add(tspNode5);
		GeographicPoint home = tspNode1;
		List<GeographicPoint> tspPath = firstMap.tsp(targets, home);
		
		System.out.println("Final path:");
		System.out.println();
		firstMap.printRoute(home, home, tspPath);
		System.out.println();
		*/
		
	}
	
}
