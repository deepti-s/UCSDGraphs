package roadgraph;

import geography.GeographicPoint;

/**
 * @author Deepti S
 * 
 * A class which represents a edge (street) between two geographic locations. 
 * Note that each edge is directed, i.e., it only denotes the edge between point A to B (not vice versa).
 */
public class MapEdge {
	private GeographicPoint startPoint;
	private GeographicPoint endPoint;
	private String displayName;
	private String type;
	private double distance;
	
		public MapEdge(GeographicPoint startPoint, GeographicPoint endPoint, String displayName, String type,
			double distance) {
		super();
		this.startPoint = startPoint;
		this.endPoint = endPoint;
		this.displayName = displayName;
		this.type = type;
		this.distance = distance;
	}



	public GeographicPoint getStartPoint() {
		return startPoint;
	}

	public void setStartPoint(GeographicPoint startPoint) {
		this.startPoint = startPoint;
	}

	public GeographicPoint getEndPoint() {
		return endPoint;
	}

	public void setEndPoint(GeographicPoint endPoint) {
		this.endPoint = endPoint;
	}

	public String getDisplayName() {
		return displayName;
	}

	public void setDisplayName(String displayName) {
		this.displayName = displayName;
	}

	public String getType() {
		return type;
	}

	public void setType(String type) {
		this.type = type;
	}

	public double getDistance() {
		return distance;
	}

	public void setDistance(double distance) {
		this.distance = distance;
	}

}
