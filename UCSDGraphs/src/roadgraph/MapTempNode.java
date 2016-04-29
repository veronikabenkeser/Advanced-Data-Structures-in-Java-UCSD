package roadgraph;

public class MapTempNode  implements Comparable<MapTempNode> {
	private MapNode startNode;
	private MapNode endNode;
	private double distance;
	private MapNode parent;
	
	public MapTempNode(MapNode start, MapNode end){
		this.startNode = start;
		this.endNode = end;
		this.distance = Double.POSITIVE_INFINITY; 
		this.parent = null;
	}
	
	public void setDistance(double distance){
		this.distance = distance;
	}
	
	public void setParent(MapNode parent){
		this.parent = parent;
	}
	
	public MapNode getParent(){
		return this.parent;
	}
	
	public double getDistance(){
		return distance;
	}
	
	public MapNode getStartNode(){
		return this.startNode;
	}
	
	public MapNode getEndNode(){
		return this.endNode;
	}
	
	@Override
	public int compareTo(MapTempNode node2) {
		
		if(this.distance < node2.distance){
			return -1;
		} else if(this.distance>node2.distance){
			return 1;
		} 
		return 0;
	}
}
