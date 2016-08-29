package roadgraph;

import java.util.Comparator;

import geography.GeographicPoint;

public class RoadStarAstar implements Comparator<RoadStarAstar> {
	private GeographicPoint startPoint;
	private double distance;
	
	private double totalDistance;
	public RoadStarAstar() {
		
		
		}
	public double getTotalDistance() {
		return totalDistance;
	}
	public void setTotalDistance(double totalDistance) {
		this.totalDistance = totalDistance;
	}
	public RoadStarAstar(GeographicPoint point,Double total){
		this.startPoint = point;
		this.totalDistance = total;
	}
	public int compare(RoadStarAstar x, RoadStarAstar y){
		if (x.totalDistance < y.totalDistance)
        {
            return -1;
        }
        if (x.totalDistance > y.totalDistance)
        {
            return 1;
        }
        return 0;
	}
	public GeographicPoint getStartPoint() {
		return startPoint;
	}
	public void setStartPoint(GeographicPoint startPoint) {
		this.startPoint = startPoint;
	}
	public RoadStarAstar(GeographicPoint startPoint, double distance, double total) {
		super();
		this.startPoint = startPoint;
		this.distance = distance;
		this.totalDistance = total;
	}
	public double getDistance() {
		return distance;
	}
	public void setDistance(double distance) {
		this.distance = distance;
	}
	
	
}
