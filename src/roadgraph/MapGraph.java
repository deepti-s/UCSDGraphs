/**
 * @author UCSD MOOC development team and Deepti S
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import geography.GeographicPoint;
import util.GraphLoader;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

/**
 * @author UCSD MOOC development team and Deepti S
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	Map<GeographicPoint, MapNode> pointNodeMap;
	Set<MapEdge> edges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph() {
		pointNodeMap = new HashMap<GeographicPoint, MapNode>();
		edges = new HashSet<MapEdge>();
	}
	
	public Map<GeographicPoint, MapNode> getPointNodeMap() {
		return pointNodeMap;
	}

	public void setPointNodeMap(Map<GeographicPoint, MapNode> map) {
		this.pointNodeMap = map;
	}

	public Set<MapEdge> getEdges() {
		return edges;
	}

	public void setEdges(Set<MapEdge> edges) {
		this.edges = edges;
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return pointNodeMap.keySet().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		return pointNodeMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		return edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		if (location != null && !containsVertex(location)) {
			//since the pointNodemap doesn't have the given vertex/location, go ahead with adding the new node (and no neighbors)
			pointNodeMap.put(location, new MapNode(location));
			return true;
			
		}
		return false;
	}
	
	/**
	 * Helper method which returns true if the Map has the given vertex/location, false otherwise.
	 * @param location
	 * @return
	 */
	private boolean containsVertex(GeographicPoint location) {
		return pointNodeMap.containsKey(location);
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		if (from == null || to == null || roadName == null || roadType == null || length < 0) {
			throw new IllegalArgumentException("Invalid arguments! All arguments must have non-null values!");
		}

		if (!containsVertex(from) || !containsVertex(to)) {
			throw new IllegalArgumentException("Invalid locations! Given GeographicPoints are not present in the graph!");
		}

		MapNode fromNode = pointNodeMap.get(from);
		MapNode toNode = pointNodeMap.get(to);

		// check nodes are valid
		if (fromNode == null || toNode == null)
			throw new IllegalArgumentException("Invalid locations! Given GeographicPoints are not present in the graph!");

		addEdge(fromNode, toNode, roadName, roadType, length);
	}

	public void addEdge(MapNode fromNode, MapNode toNode, String roadName, String roadType, double length) {
		// create new edge
		MapEdge edge = new MapEdge(fromNode, toNode, roadName, roadType, length);
		this.edges.add(edge);
		fromNode.addEdge(edge);
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{

		// check if the given start and end vertices are present in the given graph
		// if either of the given nodes are not present in the graph or if the start & goal nodes are same, path cannot be found between them! 
		if (start == null || goal == null || !containsVertex(start) || !containsVertex(goal) || start.equals(goal)) {
			System.out.println("Invalid values for Start or Goal points! Cannot find the path between them!" );
			return null;
		}

		MapNode startNode = pointNodeMap.get(start);
		MapNode goalNode = pointNodeMap.get(goal);
		if (startNode == null || goalNode == null || startNode.equals(goalNode)) {
			System.out.println("Invalid values for Start or Goal points! Cannot find the path between them!" );
			return null;
		}

		// setup to begin BFS
		// initialize queue, visitedNodes Set and parentMap
		Queue<MapNode> toBeExploredNodesQueue = new LinkedList<MapNode>();
		Set<MapNode> visitedNodes = new HashSet<MapNode>();
		Map<MapNode, MapNode> parentMap = new HashMap<MapNode,MapNode>();

		// add the start node to the queue and visitedNodes
		toBeExploredNodesQueue.add(startNode);
		visitedNodes.add(startNode);
		
		MapNode current = null;
		// loop as long as queue is not empty
		while(!toBeExploredNodesQueue.isEmpty()) {
			// get the next item from queue
			current = toBeExploredNodesQueue.remove();
			// Hook for visualization.  See writeup.
			nodeSearched.accept(current.getLocation());
			if (current.equals(goalNode)) {
				// current node is the goal node!! 
				// Return the path from start node to the goal node via parentMap
				return getReconstructedPath(startNode, goalNode, parentMap);
			} 
			for (MapNode node : current.getNeighbors()) {
				if (!visitedNodes.contains(node)) {
					visitedNodes.add(node);
					parentMap.put(node, current);
					toBeExploredNodesQueue.add(node);
				}
			}
		}
		return null;
	}
	
	/**
	 * Helper method that returns the path (traces the path from goal to the start via parentMap) 
	 * @param goalNode
	 * @param parentMap
	 * @return
	 */
	private List<GeographicPoint> getReconstructedPath(MapNode startNode, MapNode goalNode, Map<MapNode, MapNode> parentMap) {
		LinkedList<GeographicPoint> path = new LinkedList<>();
		MapNode currentNode = goalNode;
		
		// trace back from the goal node to the start node via parent map
		while (!currentNode.equals(startNode)) {
			path.addFirst(currentNode.getLocation());
			currentNode = parentMap.get(currentNode);
		}

		// add start
		path.addFirst(startNode.getLocation());
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// check if the given start and end vertices are present in the given graph
		// if either of the given nodes are not present in the graph or if the start & goal nodes are same, path cannot be found between them!
		if (start == null || goal == null || !containsVertex(start) || !containsVertex(goal) || start.equals(goal)) {
			System.out.println("Invalid values for Start or Goal points! Cannot find the path between them!" );
			return null;
		}

		MapNode startNode = pointNodeMap.get(start);
		MapNode goalNode = pointNodeMap.get(goal);
		if (startNode == null || goalNode == null || startNode.equals(goalNode)) {
			System.out.println("Invalid values for Start or Goal points! Cannot find the path between them!" );
			return null;
		}

		// setup to begin BFS
		// initialize queue, visitedNodes Set, parentMap and distancesToNodes
		PriorityQueue<MapNode> toBeExploredNodesQueue = new PriorityQueue<>();
		Set<MapNode> visitedNodes = new HashSet<>();
		Map<MapNode, MapNode> parentMap = new HashMap<>();

		initializeDistances();
		startNode.setPredictedDistance(0.0);

		// add the start node to the queue and visitedNodes
		toBeExploredNodesQueue.add(startNode);
		visitedNodes.add(startNode);

		MapNode current = null;

		// loop as long as queue is not empty
		while(!toBeExploredNodesQueue.isEmpty()) {
			// get the next item from queue
			current = toBeExploredNodesQueue.remove();

			if (!visitedNodes.contains(current)) {
				visitedNodes.add(current);
				// Hook for visualization.  See writeup.
				nodeSearched.accept(current.getLocation());
			}

			if (current.equals(goalNode)) {
				// current node is the goal node!!
				// Return the path from start node to the goal node via parentMap
				return getReconstructedPath(startNode, goalNode, parentMap);
			}

			double cumulativeDistance = 0.0;
			for (MapEdge edge : current.getEdges()) {
				MapNode node = edge.getEndNode();
				if (!visitedNodes.contains(node)) {
					cumulativeDistance = current.getPredictedDistance() + edge.getLength();
				    if (cumulativeDistance < node.getPredictedDistance())   {
						node.setPredictedDistance(cumulativeDistance);
						parentMap.put(node, current);
						toBeExploredNodesQueue.add(node);
					}
				}
			}
		}
		return null;
	}

	private void initializeDistances() {
		for(MapNode node : pointNodeMap.values()) {
			node.setPredictedDistance(Double.POSITIVE_INFINITY);
		}
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	public void printAdjListGraph(MapGraph theMap) {
		System.out.println("Num nodes: " + theMap.getNumVertices());
		System.out.println("Num edges: " + theMap.getNumEdges());
		System.out.println("Edges: ");
		for(MapEdge edge : edges) {
			System.out.println("Edge: " + edge );
		}
	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");

		//Week 2
		/*
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");   */

		theMap.printAdjListGraph(theMap);
		/*
		//BFS
		GeographicPoint start = new GeographicPoint(1.0,1.0);
		GeographicPoint end = new GeographicPoint(8.0,-1.0);
		List<GeographicPoint> route = theMap.bfs(start,end);
		System.out.println(route);               */
		
		// You can use this method for testing.  
		
		// Use this code in Week 3 End of Week Quiz
		/*
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		theMap.printAdjListGraph(theMap);

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);*/


		GraphLoader.loadRoadMap("data/graders/mod3/map3.txt", theMap);
		System.out.println("DONE.");
		theMap.printAdjListGraph(theMap);

		GeographicPoint start = new GeographicPoint(0.0,0.0);
		GeographicPoint end = new GeographicPoint(0.0,4.0);

		List<GeographicPoint> route2 = theMap.dijkstra(start,end);
		System.out.println(route2);

		//List<GeographicPoint> route3 = theMap.aStarSearch(start,end);
		//System.out.println(route3);

	}
	
}
