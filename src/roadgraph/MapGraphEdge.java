/**
 * 
 */
package roadgraph;

import geography.GeographicPoint;

/**
 * @author Hamadi McIntosh
 * 
 * A class which represents an edge in a graph of geographic locations
 * Nodes in the graph are intersections
 *
 */
public class MapGraphEdge {
	
	private MapGraphNode from;
	private MapGraphNode to;
	private String roadName;
	private String roadType;
	private double length;
	
	/**
	 * Creates a new directed edge in the graph represented by MapGraph between the
	 * specified start and end nodes.  Also, stores the provided properties of the
	 * edge.
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 */
	public MapGraphEdge(MapGraphNode from, MapGraphNode to, String roadName,
			String roadType, double length) {
		this.from = from;
		this.to = to;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}
	
	/**
	 * Retrieve the start point of the edge
	 * @return The starting point of the edge
	 */
	public MapGraphNode getStart() {
		return new MapGraphNode(from.getLoc(), from.getEdges(), from.getDist());
	}
	
	/**
	 * Retrieve the end point of the edge
	 * @return The ending point of the edge
	 */
	public MapGraphNode getEnd() {
		return new MapGraphNode(to.getLoc(), to.getEdges(), to.getDist());
	}
	
	/**
	 * Retrieve the name of the road
	 * @return The name of the road
	 */
	public String getName() {
		return roadName;
	}
	
	/**
	 * Retrieve the type of the road
	 * @return The type of the road
	 */
	public String getType() {
		return roadType;
	}
	
	/**
	 * Retrieve the length of the road
	 * @return The length of the road, in km
	 */
	public double getLength() {
		return length;
	}
	
	/**
	 * Generate string representation of the graph edge
	 * @return the String
	 */
	public String toString() {
		String toReturn = "";
		toReturn += roadName + ", " + roadType + "\n";
		toReturn += "Length: " + length + "\n";
		toReturn += "From: " + from + "\n";
		toReturn += "To: " + to;
		return toReturn;
	}

}
