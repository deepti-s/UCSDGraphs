package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;

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

	@Override
	public int hashCode() {
		int result = location != null ? location.hashCode() : 0;
		result = 31 * result + (neighbors != null ? neighbors.hashCode() : 0);
		return result;
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (!(o instanceof MapNode)) return false;

		MapNode mapNode = (MapNode) o;

		if (location != null ? !location.equals(mapNode.location) : mapNode.location != null) return false;
		if (neighbors != null ? !neighbors.equals(mapNode.neighbors) : mapNode.neighbors != null) return false;

		return true;
	}

	@Override
	public String toString() {
		return "MapNode [location=" + location + ", neighbors=" + neighbors + "]";
	}	
}
