package roadgraph;

/**
 * @author Deepti S
 * 
 * A class which represents a edge (street) between two geographic locations. 
 * Note that each edge is directed, i.e., it only denotes the edge between point A to B (not vice versa).
 */
public class MapEdge {
	public static final double DEFAULT_LENGTH = 0.0;

	private MapNode startNode;
	private MapNode endNode;
	private String roadName;
	private String roadType;
	private double length;
	
	public MapEdge(MapNode start, MapNode end, String roadName) {
		this(start, end, roadName, "", DEFAULT_LENGTH);
	}

	public MapEdge(MapNode start, MapNode end, String roadName, String roadType)
	{
		this(start, end, roadName, roadType, DEFAULT_LENGTH);
	}

	public MapEdge(MapNode startNode, MapNode endNode, String roadName, String type, double length) {
		super();
		this.startNode = startNode;
		this.endNode = endNode;
		this.roadName = roadName;
		this.roadType = type;
		this.length = length;
	}

	public MapNode getStartNode() {
		return startNode;
	}

	public void setStartNode(MapNode startNode) {
		this.startNode = startNode;
	}

	public MapNode getEndNode() {
		return endNode;
	}

	public void setEndNode(MapNode endNode) {
		this.endNode = endNode;
	}

	public String getRoadName() {
		return roadName;
	}

	public void setRoadName(String roadName) {
		this.roadName = roadName;
	}

	public String getRoadType() {
		return roadType;
	}

	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}

	public double getLength() {
		return length;
	}

	public void setLength(double length) {
		this.length = length;
	}

	// given one node in an edge, return the other node
	public MapNode getOtherNode(MapNode node) {
		if (node.equals(this.startNode))
			return this.endNode;
		else if (node.equals(this.endNode))
			return this.startNode;
		throw new IllegalArgumentException("Looking for a node that is not in the edge!");
	}

	@Override
	public String toString() {
		return "MapEdge [startNode=" + startNode + ", endNode=" + endNode + ", roadName=" + roadName
				+ ", roadType=" + roadType + ", length=" + length + "]";
	}

}
