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

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
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
	Map<MapNode, List<MapNode>> adjListMap;
	Set<MapNode> nodes;
	Set<MapEdge> edges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		nodes = new HashSet<MapNode>();
		edges = new HashSet<MapEdge>();
		adjListMap = new HashMap<MapNode, List<MapNode>>();
	}
	
	public Map<MapNode, List<MapNode>> getAdjacentListMap() {
		return adjListMap;
	}

	public void setAdjacentListMap(Map<MapNode, List<MapNode>> map) {
		this.adjListMap = map;
	}

	public Set<MapNode> getNodes() {
		return nodes;
	}

	public void setNodes(Set<MapNode> nodes) {
		this.nodes = nodes;
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
	public int getNumVertices()
	{
		return (nodes == null) ? 0 : nodes.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		Set<GeographicPoint> vertices = new HashSet<GeographicPoint>();
		for (MapNode node : nodes) {
			// looping through all existing nodes, get the geographic location/vertex, and add it to the new set of vertices.
			vertices.add(node.getLocation());
		}
		return vertices;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return (edges == null) ? 0 : edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if (location != null && !containsVertex(location)) {
			// since list of nodes don't have the given vertex/location, go ahead with the creation of new Node
			//create a new MapNode object
			MapNode node = new MapNode(location);
			//add the new node to the global Set of nodes
			nodes.add(node);
			//add the new node (and no neighbors) to adjacency list map 
			adjListMap.put(node, new ArrayList<MapNode>());
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
		return (getMapNode(location) != null);
	}
	
	/*
	 * Helper method which returns the MapNode in the Map which corresponds to the given location
	 */
	private MapNode getMapNode(GeographicPoint location) {
		for (MapNode node : nodes) {
			if (node.getLocation().equals(location)) {
				return node;
			}
		}
		return null;
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
		 
		MapNode startNode = getMapNode(from);
		MapNode endNode = getMapNode(to);
		
		if (startNode == null || endNode == null) {
			throw new IllegalArgumentException("Invalid locations! Given GeographicPoints are not present in the graph!");
		}
		// create new edge
		MapEdge edge = new MapEdge(from, to, roadName, roadType, length);
		

		List<MapNode> adjacentNodes = this.adjListMap.get(startNode);
		adjacentNodes.add(endNode);

		// add the edge to the node
		startNode.getNeighbors().add(edge);

		// add the edge to the Set of edges
		this.edges.add(edge);

		// add the start node and the adjacent nodes list to the Adjacent List Map
		this.adjListMap.put(startNode, adjacentNodes);
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
		// initialize queue, visitedNodes Set and parentMap
		Queue<MapNode> queue = new LinkedList<MapNode>();
		Set<MapNode> visitedNodes = new HashSet<MapNode>();
		Map<MapNode, MapNode> parentMap = new HashMap<MapNode,MapNode>();
		
		// check if the given start and end vertices are present in the given graph
		MapNode startNode = getMapNode(start);
		MapNode goalNode = getMapNode(goal);
		
		// if either of the given nodes are not present in the graph or if the start & goal nodes are same, path cannot be found between them! 
		if (startNode == null || goalNode == null || startNode == goalNode) {
			System.out.println("Invalid values for Start or Goal points! Cannot find the path between them!" );
			return null;
		}
		// add the startNode to the queue and visitedNodes
		queue.add(startNode);
		visitedNodes.add(startNode);
		
		MapNode currentNode;
		// loop as long as queue is not empty
		while(!queue.isEmpty()) {
			// get the next item from queue
			currentNode = queue.remove();
			// Hook for visualization.  See writeup.
			nodeSearched.accept(currentNode.getLocation());
			if (currentNode.equals(goalNode)) {
				// current node is the goal node!! 
				// Return the path from start node to the goal node via parentMap
				return getPath(startNode, goalNode, parentMap);
			} 
			List<MapNode> neighbors = adjListMap.get(currentNode);
			for (MapNode node : neighbors) {
				if (!visitedNodes.contains(node)) {
					visitedNodes.add(node);
					parentMap.put(node, currentNode);
					queue.add(node);
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
	private List<GeographicPoint> getPath(MapNode startNode, MapNode goalNode, Map<MapNode,MapNode> parentMap) {
		List<GeographicPoint> geographicPointsPath = new ArrayList<GeographicPoint>();
		List<MapNode> nodePath = new ArrayList<MapNode>();
		MapNode parentNode = goalNode;
		
		// trace back from the goal node to the start node via parent map
		nodePath.add(goalNode);
		while (parentNode != startNode) {
			parentNode = parentMap.get(parentNode);
			nodePath.add(parentNode);
		}

		// need to reverse the path to get the path from start node to goal node.
		Collections.reverse(nodePath);

		for (MapNode node : nodePath) {
			geographicPointsPath.add(node.getLocation());
		}
		return geographicPointsPath;
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
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
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

	public void printAdjListGraph() {
		for(MapNode node : adjListMap.keySet()) {
			System.out.print("Node: " + node.getLocation() + " ---> {");
			for (MapNode adjNode : adjListMap.get(node)) {
				System.out.print(adjNode.getLocation() +";\t");
			}
			System.out.println("}");
		}
	}	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		//theMap.printAdjListGraph();
		
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
