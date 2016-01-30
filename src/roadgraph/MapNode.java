package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

/**
 * @author Deepti S
 * 
 * A class which represents a node in a map, i.e. geographic locations or intersection of edges.
 */
public class MapNode {
	private GeographicPoint location;
	private List<MapEdge> neighbors;
	
	public MapNode(GeographicPoint geoPoint) {
		super();
		this.location = geoPoint;
		this.neighbors = new ArrayList<MapEdge>();
	}
	public GeographicPoint getLocation() {
		return location;
	}

	public void setLocation(GeographicPoint nodeLocation) {
		this.location = nodeLocation;
	}
	
	public List<MapEdge> getNeighbors() {
		return neighbors;
	}
	

	public void setNeighbors(List<MapEdge> neighbors) {
		this.neighbors = neighbors;
	}	

}
