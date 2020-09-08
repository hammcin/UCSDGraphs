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
public class MapGraphNode {
	
	GeographicPoint location;
	private List<MapGraphEdge> edges;
	
	public MapGraphNode(GeographicPoint loc) {
		this.location = loc;
		edges = new ArrayList<MapGraphEdge>();
	}
	
	public MapGraphNode(GeographicPoint loc, List<MapGraphEdge> e) {
		this.location = loc;
		this.edges = e;
	}
	
	public GeographicPoint getLoc() {
		return new GeographicPoint(this.location.getX(), this.location.getY());
	}
	
	public void addEdge(MapGraphEdge edge) {
		edges.add(edge);
	}
	
	public List<MapGraphEdge> getEdges() {
		return new ArrayList<MapGraphEdge>(edges);
	}
	
	public String toString() {
		return "" + location;
	}

}
