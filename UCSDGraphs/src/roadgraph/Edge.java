package roadgraph;

public class Edge {
	private String roadName;
	private String roadType;
	private double length;
	private Intersection endPoint;
	
	public Edge(String roadName, 
			String roadType, double length, Intersection endPoint){
		this.roadName= roadName;
		this.roadType=roadType;
		this.length=length;
		this.endPoint = endPoint;
	}
	
	public Intersection getIntersection(){
		return this.endPoint;
	}
}
