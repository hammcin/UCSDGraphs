/**
 * 
 */
package roadgraph;

import java.util.ArrayList;
import java.util.List;
import geography.GeographicPoint;

/**
 * @author Hamadi McIntosh
 * 
 * A class which represents a node in a graph of geographic locations
 * Nodes in the graph are intersections
 *
 */
public class MapGraphNode implements Comparable<MapGraphNode> {
	
	GeographicPoint location;
	private List<MapGraphEdge> edges;
	private double distance;
	private double predictDist;
	
	public MapGraphNode(GeographicPoint loc) {
		this.location = loc;
		edges = new ArrayList<MapGraphEdge>();
		this.distance = Double.POSITIVE_INFINITY;
		this.predictDist = Double.POSITIVE_INFINITY;
	}
	
	public MapGraphNode(GeographicPoint loc, List<MapGraphEdge> e,
			double d, double p) {
		this.location = loc;
		this.edges = e;
		this.distance = d;
		this.predictDist = p;
	}
	
	public GeographicPoint getLoc() {
		return new GeographicPoint(this.location.getX(), this.location.getY());
	}
	
	public double getDist() {
		return this.distance;
	}
	
	public void setDist(double d) {
		this.distance = d;
	}
	
	public double getPredicted() {
		return this.predictDist;
	}
	
	public void setPredicted(double p) {
		this.predictDist = p;
	}
	
	public void addEdge(MapGraphEdge edge) {
		edges.add(edge);
	}
	
	public List<MapGraphEdge> getEdges() {
		return new ArrayList<MapGraphEdge>(edges);
	}
	
	public int compareTo(MapGraphNode other) throws NullPointerException {
		
		if (other == null) {
			throw new NullPointerException("This object cannot be compared to null");
		}
		
		return Double.compare(this.predictDist, other.getPredicted());
	}
	
	public boolean equals(Object obj) {
		if (!(obj instanceof MapGraphNode)) {
			return false;
		}
		
		MapGraphNode other = (MapGraphNode) obj;
		
		return this.location.equals(other.getLoc());
	}
	
	public int hashCode() {
		return location.hashCode();
	}
	
	public String toString() {
		return "" + location + "\nPredicted Distance: " + predictDist;
	}

}
