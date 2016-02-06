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


	@Override
	public int hashCode() {
		int result;
		long temp;
		result = startPoint != null ? startPoint.hashCode() : 0;
		result = 31 * result + (endPoint != null ? endPoint.hashCode() : 0);
		result = 31 * result + (displayName != null ? displayName.hashCode() : 0);
		result = 31 * result + (type != null ? type.hashCode() : 0);
		temp = Double.doubleToLongBits(distance);
		result = 31 * result + (int) (temp ^ (temp >>> 32));
		return result;
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (!(o instanceof MapEdge)) return false;

		MapEdge mapEdge = (MapEdge) o;

		if (Double.compare(mapEdge.distance, distance) != 0) return false;
		if (displayName != null ? !displayName.equals(mapEdge.displayName) : mapEdge.displayName != null) return false;
		if (endPoint != null ? !endPoint.equals(mapEdge.endPoint) : mapEdge.endPoint != null) return false;
		if (startPoint != null ? !startPoint.equals(mapEdge.startPoint) : mapEdge.startPoint != null) return false;
		if (type != null ? !type.equals(mapEdge.type) : mapEdge.type != null) return false;

		return true;
	}


	@Override
	public String toString() {
		return "MapEdge [startPoint=" + startPoint + ", endPoint=" + endPoint + ", displayName=" + displayName
				+ ", type=" + type + ", distance=" + distance + "]";
	}

}
