package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * @author Deepti S
 * 
 * A class which represents a node in a map, i.e. geographic locations or intersection of edges.
 */
public class MapNode implements Comparable<MapNode> {
	private GeographicPoint location;
	private List<MapEdge> edges;
	private double actualDistance;
	private double predictedDistance;

	public MapNode(GeographicPoint geoPoint) {
		super();
		this.location = geoPoint;
		this.edges = new ArrayList<MapEdge>();
		this.actualDistance = 0.0;
		this.predictedDistance = 0.0;
	}

	public GeographicPoint getLocation() {
		return location;
	}

	public void setLocation(GeographicPoint nodeLocation) {
		this.location = nodeLocation;
	}
	
	public List<MapEdge> getEdges() {
		return edges;
	}

	public void setEdges(List<MapEdge> edges) {
		this.edges = edges;
	}

	public double getActualDistance() {
		return actualDistance;
	}

	public void setActualDistance(double actualDistance) {
		this.actualDistance = actualDistance;
	}

	public double getPredictedDistance() {
		return predictedDistance;
	}

	public void setPredictedDistance(double predictedDistance) {
		this.predictedDistance = predictedDistance;
	}

	public void addEdge(MapEdge edge) {
		this.edges.add(edge);
	}

	/** Return the neighbors of this MapNode */
	Set<MapNode> getNeighbors() {
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for (MapEdge edge : edges) {
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (!(o instanceof MapNode)) return false;

		MapNode mapNode = (MapNode) o;

		return !(this.location != null ? !this.location.equals(mapNode.location) : mapNode.location != null);

	}

	@Override
	public int hashCode() {
		return this.location != null ? this.location.hashCode() : 0;
	}

	@Override
	public String toString() {
		final StringBuilder sb = new StringBuilder("MapNode{");
		sb.append("actualDistance=").append(actualDistance);
		sb.append(", location=").append(location);
		sb.append(", edges=").append(edges);
		sb.append(", predictedDistance=").append(predictedDistance);
		sb.append('}');
		return sb.toString();
	}

	@Override
	public int compareTo(MapNode otherNode) {
		return ((Double) this.getPredictedDistance()).compareTo((Double) otherNode.getPredictedDistance());
	}
}
