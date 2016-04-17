package roadgraph;

public class MapTempNodeVector implements Comparable<MapTempNodeVector> {
	private MapNode startNode;
	private MapNode endNode;
	private double actualDistance;
	private double predictedDistance;
	
	public MapTempNodeVector(MapNode start, MapNode end){
		this.startNode = start;
		this.endNode = end;
		this.actualDistance = Double.POSITIVE_INFINITY; 
		this.predictedDistance = Double.POSITIVE_INFINITY; 
	}
	
	private void setPredictedDistance(MapNode start, MapNode goal){
		double predDist= getDistanceFromLatLonInMiles(start.getLocation().getY(),start.getLocation().getX(),
									goal.getLocation().getY(),goal.getLocation().getX());
		this.predictedDistance = predDist+this.actualDistance;
	}
	
	public void setActualDistance(double distance, MapNode start, MapNode goal){
		this.actualDistance = distance;
		setPredictedDistance(start, goal);
	}
	
	public void initializeActualDistance(){
		this.actualDistance = 0;
	}
	
	public void initializePredictedDistance(){
		this.predictedDistance=0;
	}
	
	public MapNode getEndNode(){
		return this.endNode;
	}
	
	public double getPredictedDist(){
		return this.predictedDistance;
	}
	public MapNode getStartNode(){
		return this.startNode;
	}
	
	private double getDistanceFromLatLonInMiles(double lat1,double lon1,double lat2,double lon2) {
		  double r = 3958.756;
		  double dLat = deg2rad(lat2-lat1);  // deg2rad below
		  double dLon = deg2rad(lon2-lon1); 
		  double a = 
		    Math.sin(dLat/2) * Math.sin(dLat/2) +
		    Math.cos(deg2rad(lat1)) * Math.cos(deg2rad(lat2)) * 
		    Math.sin(dLon/2) * Math.sin(dLon/2); 
		  double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a)); 
		  return r * c;
		}

		private double deg2rad(double deg) {
		  return deg * (Math.PI/180);
		}
	
	public int compareTo(MapTempNodeVector node){
		if(this.predictedDistance<node.predictedDistance){
			return -1;
		} else if(this.predictedDistance>node.predictedDistance){
			return 1;
		}
		return 0;
	}
}
