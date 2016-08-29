package roadgraph;

import geography.GeographicPoint;

/**
 * Class used To store the roadType,roadName,length, endPoint Coordinates of the intersection.
 */
 class Intersection {
	
	private String roadName; //name of the road
	private String roadType; //road type
	private double length;   //length of the road
	public String getRoadName() {
		return roadName;
	}

	public void setRoadName(String roadName) {
		this.roadName = roadName;
	}

	public String getRoadType() {
		return roadType;
	}

	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}

	public double getLength() {
		return length;
	}

	public void setLength(double length) {
		this.length = length;
	}

	private GeographicPoint point;
	
	//constructor for setting initial values.
	public Intersection(String roadName, String roadType, double length, GeographicPoint point) {
		
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
		this.point = point;
	}

	public GeographicPoint getPoint() {
		return point;
	}
	
	public void setPoint(GeographicPoint point) {
		this.point = point;
	}
	
}
