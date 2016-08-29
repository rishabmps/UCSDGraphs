package roadgraph;

import java.util.Comparator;

import geography.GeographicPoint;

public class RoadStart implements Comparator<RoadStart> {
	
	private GeographicPoint startPoint;
	/**
	 * To store distance from 
	 */
	private double distance;
	private double distanceToEnd;
	public RoadStart() {
		super();
	}
	public int compare(RoadStart x,RoadStart y){
		if (x.distance < y.distance)
        {
            return -1;
        }
        if (x.distance > y.distance)
        {
            return 1;
        }
        return 0;
	}
	public RoadStart(GeographicPoint startPoint, double distance) {
		
		this.startPoint = startPoint;
		this.distance = distance;
	}

	public RoadStart(GeographicPoint startPoint) {
		
		this.startPoint = startPoint;
		this.distance = Double.POSITIVE_INFINITY;
		
	}

	public GeographicPoint getStartPoint() {
		return startPoint;
	}

	public void setStartPoint(GeographicPoint startPoint) {
		this.startPoint = startPoint;
	}

	public double getDistance() {
		return distance;
	}

	public void setDistance(double distance) {
		this.distance = distance;
	}
	
	
}
