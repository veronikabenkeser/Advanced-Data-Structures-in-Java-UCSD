package roadgraph;

/*
 * A class which represents directed edges that connect the nodes in the graph
 * 
 */
public class MapEdge {
	private String roadName;
	private String roadType;
	private double length;
	private MapNode endPoint;
	
	/*
	 * Constructor
	 * @param roadName The name of the edge that connects two nodes/intersections
	 * @param roadType  Road type
	 * @param length  Road length
	 * @param endPoint  The node this edge leads to
	 */
	public MapEdge(String roadName, 
			String roadType, double length, MapNode endPoint){
		this.roadName= roadName;
		this.roadType=roadType;
		this.length=length;
		this.endPoint = endPoint;
	}
	
	/*
	 * Get the node this edge leads to
	 * @return The node this edge leads to
	 */
	public MapNode connectedTo(){
		return this.endPoint;
	}
}
	