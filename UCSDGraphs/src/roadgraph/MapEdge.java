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
	private MapNode startPoint;
	
	/*
	 * Constructor
	 * @param roadName The name of the edge that connects two nodes/intersections
	 * @param roadType  Road type
	 * @param length  Road length
	 * @param endPoint  The node this edge leads to
	 */
	public MapEdge(String roadName, 
			String roadType, double length, MapNode startPoint, MapNode endPoint){
		this.roadName= roadName;
		this.roadType=roadType;
		this.length=length;
		this.endPoint = endPoint;
		this.startPoint = startPoint;
	}
	
	/*
	 * Get the node this edge leads to
	 * @return The node this edge leads to
	 */
	public MapNode getEndPoint(){
		return this.endPoint;
	}
	
	public MapNode getStartPoint(){
		return this.startPoint;
	}
	
	public double getDistance(){
		return this.length;
	}
}
	