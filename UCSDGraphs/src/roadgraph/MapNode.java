package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

/*
 * A class which represents a node/intersection in the graph.
 * 
 */

public class MapNode {
	private GeographicPoint location;
	private int edgeCount;
	private List<MapEdge> edges;

	/** 
	 * MapNode constructor
	 * @param location  The location of the node/intersection
	 * 
	 */
	public MapNode(GeographicPoint location){
		this.location = location;
		this.edgeCount=0;
		this.edges = new ArrayList<MapEdge>();
	}
	
	/*
	 *  Add a directed edge between the caller of the method and a 
	 *  given node.
	 *  
	 *  @param neighbor  The node this node is connected to
	 *  @param roadName  The name of the connecting edge
	 *  @param roadType  The road type of the connecting edge
	 *  @param length  The length of the connecting edge
	 */
	public void addNeighbor(MapNode neighbor, String roadName, 
							String roadType, double length)
	{
		edges.add(new MapEdge(roadName,roadType,length,neighbor));
		edgeCount++;
	}
	
	/*
	 * Count the number of directed edges that are connected to this node
	 * @return  The number of connected edges
	 * 
	 */
	public int getEdgeCount(){
		return this.edgeCount;
	}
	
	/*
	 * Get a list of directed edges that are connected to this node
	 * @return  The list of edges connected to this node
	 * 
	 */
	public List<MapEdge> getMapEdges(){
		return this.edges;
	}
	
	/*
	 * Find the location of this node/intersection
	 * @return  The location of this node
	 * 
	 */
	public GeographicPoint getLocation(){
		return this.location;
	}
	
}