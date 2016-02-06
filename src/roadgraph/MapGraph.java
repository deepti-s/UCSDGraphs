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
	Map<GeographicPoint, List<MapEdge>> adjListMap;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		adjListMap = new HashMap<GeographicPoint, List<MapEdge>>();
	}
	
	public Map<GeographicPoint, List<MapEdge>> getAdjacentListMap() {
		return adjListMap;
	}

	public void setAdjacentListMap(Map<GeographicPoint, List<MapEdge>> map) {
		this.adjListMap = map;
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return adjListMap.keySet().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return adjListMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		int numEdges = 0;
		for (GeographicPoint node: adjListMap.keySet()) {
			numEdges += adjListMap.get(node).size();
		}
		return numEdges;
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
			//since the adjacency list map doesn't have the given vertex/location, go ahead with adding the new node (and no neighbors)
			adjListMap.put(location, new ArrayList<MapEdge>());
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
		return adjListMap.containsKey(location);
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
		// create new edge
		MapEdge edge = new MapEdge(from, to, roadName, roadType, length);

		List<MapEdge> edges = this.adjListMap.get(from);
		edges.add(edge);

		// add the start node and the adjacent edges list to the Adjacent List Map
		this.adjListMap.put(from, edges);
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
		Queue<GeographicPoint> queue = new LinkedList<GeographicPoint>();
		Set<GeographicPoint> visitedNodes = new HashSet<GeographicPoint>();
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint,GeographicPoint>();
		
		// check if the given start and end vertices are present in the given graph
		// if either of the given nodes are not present in the graph or if the start & goal nodes are same, path cannot be found between them! 
		if (start == null || goal == null || !containsVertex(start) || !containsVertex(goal) || start == goal) {
			System.out.println("Invalid values for Start or Goal points! Cannot find the path between them!" );
			return null;
		}
		// add the start node to the queue and visitedNodes
		queue.add(start);
		visitedNodes.add(start);
		
		GeographicPoint current;
		// loop as long as queue is not empty
		while(!queue.isEmpty()) {
			// get the next item from queue
			current = queue.remove();
			// Hook for visualization.  See writeup.
			nodeSearched.accept(current);
			if (current.equals(goal)) {
				// current node is the goal node!! 
				// Return the path from start node to the goal node via parentMap
				return geBFSPath(start, goal, parentMap);
			} 
			List<MapEdge> neighbors = adjListMap.get(current);
			for (MapEdge node : neighbors) {
				if (!visitedNodes.contains(node.getEndPoint())) {
					visitedNodes.add(node.getEndPoint());
					parentMap.put(node.getEndPoint(), current);
					queue.add(node.getEndPoint());
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
	private List<GeographicPoint> geBFSPath(GeographicPoint startNode, GeographicPoint goalNode,
											Map<GeographicPoint, GeographicPoint> parentMap) {
		List<GeographicPoint> nodePath = new ArrayList<>();
		GeographicPoint parentNode = goalNode;
		
		// trace back from the goal node to the start node via parent map
		nodePath.add(goalNode);
		while (parentNode != startNode) {
			parentNode = parentMap.get(parentNode);
			nodePath.add(parentNode);
		}

		// need to reverse the path to get the path from start node to goal node.
		Collections.reverse(nodePath);

		return nodePath;
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
		for(GeographicPoint node : adjListMap.keySet()) {
			System.out.print("Node: " + node + " ---> {");
			for (MapEdge adjNode : adjListMap.get(node)) {
				System.out.print(adjNode.getEndPoint() +";\t");
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
		theMap.printAdjListGraph();
		
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
