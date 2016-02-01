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
		final int prime = 31;
		int result = 1;
		result = prime * result + ((location == null) ? 0 : location.hashCode());
		result = prime * result + ((neighbors == null) ? 0 : neighbors.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		MapNode other = (MapNode) obj;
		if (location == null) {
			if (other.location != null)
				return false;
		} else if (!location.equals(other.location))
			return false;
		if (neighbors == null) {
			if (other.neighbors != null)
				return false;
		} else if (!neighbors.equals(other.neighbors))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "MapNode [location=" + location + ", neighbors=" + neighbors + "]";
	}	
}
