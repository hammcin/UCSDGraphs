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
	
	private GeographicPoint from;
	private GeographicPoint to;
	private String roadName;
	private String roadType;
	private double length;
	
	public MapGraphEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) {
		this.from = from;
		this.to = to;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}
	
	public GeographicPoint getStart() {
		return from;
	}
	
	public GeographicPoint getEnd() {
		return to;
	}
	
	public String getName() {
		return roadName;
	}
	
	public String getType() {
		return roadType;
	}
	
	public double getLength() {
		return length;
	}
	
	public String toString() {
		String toReturn = "";
		toReturn += roadName + ", " + roadType + "\n";
		toReturn += "Length: " + length + "\n";
		toReturn += "From: " + from + "\n";
		toReturn += "To: " + to;
		return toReturn;
	}

}
