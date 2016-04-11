/**
 * @author UCSD MOOC development team and Veronika Benkeser
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and Veronika Benkeser
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2

	private Map<GeographicPoint, MapNode> grid;
	private int numVerticies;
	private int numEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		
		grid = new HashMap<GeographicPoint, MapNode>();
		numVerticies=0;
		numEdges=0;
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return numVerticies;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return grid.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
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
		// TODO: Implement this method in WEEK 2
		if(location ==null || grid.containsKey(location)){
			return false;
		}
		
		MapNode elem = new MapNode(location);
		
		grid.put(location, elem);
		numVerticies++;
		return true;
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
		
		//TODO: Implement this method in WEEK 2
		if(!grid.containsKey(from) || !grid.containsKey(to)){
			throw new IllegalArgumentException("The requested intersections have not been added to the graph.");
		}
		
		if(roadType == null){
			throw new IllegalArgumentException("Parameters cannot be null.");
		}
		
		if(length<0){
			throw new IllegalArgumentException("Length cannot be less than 0.");
		}
		
		MapNode fromElem = grid.get(from);
		fromElem.addNeighbor(grid.get(to), roadName, roadType,length);
		
		numEdges++;
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
	 * path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		
		if(start == null || goal==null){
			return null;
		}
		
		MapNode startItem = grid.get(start);
		MapNode endItem = grid.get(goal);
		Map<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		
		boolean found = bfsSuccessful(startItem,endItem,parentMap, nodeSearched);
		if(!found){
			return null;
		}
		return constructPath(goal, parentMap);
	}
	
	/*
	 * Construct a path from the starting node to the goal node.
	 * @param goal  The location of the starting node
	 * @param parentMap  The hashmap that maps each explored node to its 
	 * most-recent parent
	 * @return The list of nodes that form the shortest (unweighted)
	 * path from start to goal (including both start and goal).
	 */
	private List<GeographicPoint> constructPath (GeographicPoint goal, Map<MapNode,MapNode> parentMap){
	
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		path.addFirst(goal);
		MapNode item = grid.get(goal);
		
		while(parentMap.containsKey(item)){
			MapNode elem = parentMap.get(item);
			path.addFirst(elem.getLocation());
			item = elem;
		}
		return path;
	}
	
	/*
	 * Explore the graph via BFS to see if there is path from the start node to 
	 * the end node. Keep track of each node's parent by putting the node and its parent
	 * into a HashMap.
	 * 
	 * @param start  The starting point of the BFS traversal
	 * @param end  The goal we're trying to reach
	 * @param parentMap  The mapping of the child node 
	 * to the parent node (where the search came from)
	 * @param nodeSearched  Consumer that will display the explored node
	 * @return Return whether the node was found
	 */
	private boolean bfsSuccessful(MapNode start, MapNode end,
			Map<MapNode,MapNode> parentMap, Consumer<GeographicPoint> nodeSearched){
		Queue<MapNode> q = new LinkedList<MapNode>();
		Set<MapNode> visited = new HashSet<MapNode>();
		q.add(start);
		visited.add(start);
		
		while(!q.isEmpty()){
			MapNode elem = q.poll();
			
			// Hook for visualization. 
			nodeSearched.accept(elem.getLocation());
			
			if(elem.equals(end)){
				return true;
			}
			for(MapEdge child: elem.getMapEdges()){
				MapNode i = child.connectedTo();
				if(!visited.contains(i)){
					q.add(i);
					visited.add(i);
					parentMap.put(i, elem);
				}
			}
		}
		return false;
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

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
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
