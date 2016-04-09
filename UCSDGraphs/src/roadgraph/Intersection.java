package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class Intersection {
	private GeographicPoint location;
	private int edgesCount;
	private List<Edge> edges;

	public Intersection(GeographicPoint location){
		this.location = location;
		this.edgesCount=0;
		this.edges = new ArrayList<Edge>();
	}
	
	public void addNeighbor(Intersection neighbor, String roadName, 
							String roadType, double length)
	{
		edges.add(new Edge(roadName,roadType,length,neighbor));
		edgesCount++;
	}
	
	public int getEdgesCount(){
		return this.edgesCount;
	}
	
	public List<Edge> getEdges(){
		return this.edges;
	}
	
	public GeographicPoint getLocation(){
		return this.location;
	}
	
}
