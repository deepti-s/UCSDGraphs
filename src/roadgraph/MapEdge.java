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
		final int prime = 31;
		int result = 1;
		result = prime * result + ((displayName == null) ? 0 : displayName.hashCode());
		long temp;
		temp = Double.doubleToLongBits(distance);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		result = prime * result + ((endPoint == null) ? 0 : endPoint.hashCode());
		result = prime * result + ((startPoint == null) ? 0 : startPoint.hashCode());
		result = prime * result + ((type == null) ? 0 : type.hashCode());
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
		MapEdge other = (MapEdge) obj;
		if (displayName == null) {
			if (other.displayName != null)
				return false;
		} else if (!displayName.equals(other.displayName))
			return false;
		if (Double.doubleToLongBits(distance) != Double.doubleToLongBits(other.distance))
			return false;
		if (endPoint == null) {
			if (other.endPoint != null)
				return false;
		} else if (!endPoint.equals(other.endPoint))
			return false;
		if (startPoint == null) {
			if (other.startPoint != null)
				return false;
		} else if (!startPoint.equals(other.startPoint))
			return false;
		if (type == null) {
			if (other.type != null)
				return false;
		} else if (!type.equals(other.type))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "MapEdge [startPoint=" + startPoint + ", endPoint=" + endPoint + ", displayName=" + displayName
				+ ", type=" + type + ", distance=" + distance + "]";
	}

}
