/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;
import geography.GeographicPoint;
import util.GraphLoader;
/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 */
public class MapGraph {
	// AdjList as it is a sparse graph.
	// Intersection is a class defined in the package
	private Map<GeographicPoint,ArrayList<Intersection>> adjListsMap;
	private int numVertices;
	private int numEdges;
	/** 
	 * Creates an new empty MapGraph 
	 */
	public MapGraph()
	{
		this.numEdges=0;
		this.numVertices=0;
		adjListsMap = new HashMap<GeographicPoint,ArrayList<Intersection>>();
	}
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		
		return adjListsMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numEdges;
	}
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	
	public boolean addVertex(GeographicPoint location)
	{
		if(adjListsMap.containsKey(location)|| location==null){
			return false;
		}
		
		ArrayList<Intersection> neighbors = new ArrayList<>();
		adjListsMap.put(location, neighbors);     //adding neighbors to adjList for key=location
		numVertices++; 
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		
		//checking if points are there in adjList or not and if any other argument is zero or not.
		
		if(!adjListsMap.containsKey(from) || !adjListsMap.containsKey(to)){
			throw new IllegalArgumentException();
		}
		else if(roadName==null || roadType==null ||length<0){
			throw new IllegalArgumentException();
		}
		
		Intersection intersection = new Intersection(roadName, roadType, length, to);
		(adjListsMap.get(from)).add(intersection);
		numEdges++;
		
	}
	public List<Intersection> getEdges(GeographicPoint point){
		return  adjListsMap.get(point);
		
	}
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{	
		/** Checking if anything is null*/
		if (start == null || goal == null ) {
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}
		/** parentMap keeping information about path. */
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		/**searching */
		boolean found = bfsSearch(start, goal, parentMap,nodeSearched);
		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		/** reconstruct the path */
		return constructPath(start, goal, parentMap);	
	}
	/**
	 * To do a Breadth First Search.
	 *@param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.
	 * @param parentMap containing information about which road traversal from start in a HashMap.
	 * @return true if goal find else return false.
	 */
	private boolean bfsSearch(GeographicPoint start, GeographicPoint goal,
			HashMap<GeographicPoint, GeographicPoint> parentMap,Consumer<GeographicPoint> nodeSearched){
		int Count =0;
		/**
		 * Initializing queue for toExplore GeoraphicPoints and set for visited points 
		 */
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		Queue<GeographicPoint> toExplore = new LinkedList<GeographicPoint>();
		boolean found = false;
		toExplore.add(start);
		/**
		 * Searching
		 */
		while (!toExplore.isEmpty()) {
			GeographicPoint curr = toExplore.remove();
			Count++;
			nodeSearched.accept(curr);
			if (curr.equals(goal)) {
				found = true;
				break;
			}
			List<Intersection> neighbors = adjListsMap.get(curr);
			ListIterator<Intersection> it = neighbors.listIterator(neighbors.size());
			while (it.hasPrevious()) {
				Intersection temp = it.previous();
				GeographicPoint next = temp.getPoint();
				if (!visited.contains(next)) {
					visited.add(next);
					parentMap.put(next, curr);
					toExplore.add(next);
				}
			}
		}
		System.out.println(Count);
		return found;
	}
	
	/**
	 * To reconstructing the path
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.
	 * @param parentMap containing information about which road traversal from start in a HashMap.
	 * @return The list of Constructed path from start to end
	 */
	private static List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal,
			HashMap<GeographicPoint, GeographicPoint> parentMap) {
		
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		GeographicPoint curr = goal;
		while (curr != start) {
			path.addFirst(curr);
			curr = parentMap.get(curr);
		}
		path.addFirst(start);
		return path;
	}


	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{	int count=0;
		
		if (start == null || goal == null ) {
			//System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}
		/** parentMap keeping information about path. */
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		/**searching */
		
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		Map<GeographicPoint,Double> map = new HashMap<>();
		for(GeographicPoint point : adjListsMap.keySet()){
			map.put(point, Double.POSITIVE_INFINITY);
		}
		
		Comparator<RoadStart> comparator = new RoadStart();
		PriorityQueue<RoadStart> toExplore = new PriorityQueue<>(comparator);
		boolean found = false;
		map.put(start, 0.0);
		toExplore.add(new RoadStart(start,0.0));
		
		/**
		 * Searching
		 */
		while (!toExplore.isEmpty()) {
			RoadStart roadStart = toExplore.remove();
			count++;
			System.out.println("\n\n"+roadStart.getStartPoint());
			GeographicPoint curr = roadStart.getStartPoint();
			nodeSearched.accept(curr);
			
			if(!visited.contains(curr)){
				visited.add(curr);
				if (curr.equals(goal)) {
					found = true;
					break;
				}
				List<Intersection> neighbors = adjListsMap.get(curr);
				ListIterator<Intersection> it = neighbors.listIterator(neighbors.size());
				System.out.println(neighbors.size());
				System.out.println("neigbours of "+curr+" which are visited :" );
				
				while (it.hasPrevious()) {
					Intersection temp = it.previous();
					GeographicPoint next = temp.getPoint();
					
					double length = temp.getLength();
					double distance = length + roadStart.getDistance();
				
					if (!visited.contains(next)) {
						if(distance<map.get(next)){
							System.out.println("Added to parent map : "+next +" with parent: " +curr);
							map.put(next, distance);
							parentMap.put(next, curr);
							toExplore.add(new RoadStart(next,distance));
						}
							
					}
				}
			}
			
		}
		System.out.println(count);
		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		/** reconstruct the path */
		return constructPath(start, goal, parentMap);	

		
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		int count = 0;
		if (start == null || goal == null ) {
			//System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}
		/** parentMap keeping information about path. */
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		/**searching */
		
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		Map<GeographicPoint,Double> map = new HashMap<>();
		for(GeographicPoint point : adjListsMap.keySet()){
			map.put(point, Double.POSITIVE_INFINITY);
		}
		
		Comparator<RoadStarAstar> comparator = new RoadStarAstar();
		PriorityQueue<RoadStarAstar> toExplore = new PriorityQueue<>(comparator);
		boolean found = false;
		map.put(start, 0.0);
		toExplore.add(new RoadStarAstar(start,0.0,0.0));
		
		/**
		 * Searching
		 */
		while (!toExplore.isEmpty()) {
			RoadStarAstar roadStart = toExplore.remove();
			count++;
			System.out.println("\n\n"+roadStart.getStartPoint());
			GeographicPoint curr = roadStart.getStartPoint();
			nodeSearched.accept(curr);
			
			if(!visited.contains(curr)){
				visited.add(curr);
				if (curr.equals(goal)) {
					found = true;
					break;
				}
				List<Intersection> neighbors = adjListsMap.get(curr);
				ListIterator<Intersection> it = neighbors.listIterator(neighbors.size());
				System.out.println(neighbors.size());
				System.out.println("neigbours of "+curr+" which are visited :" );
				
				while (it.hasPrevious()) {
					Intersection temp = it.previous();
					GeographicPoint next = temp.getPoint();
					
					double length = temp.getLength();
					double distance = roadStart.getTotalDistance();
				
					if (!visited.contains(next)) {
						if(distance<map.get(next)){
							System.out.println("Added to parent map : "+next +" with parent: " +curr);
							map.put(next, distance);
							parentMap.put(next, curr);
							toExplore.add(new RoadStarAstar(next,roadStart.getDistance()+length,roadStart.getDistance()+length+next.distance(goal)));
						}
							
					}
				}
			}
			
		}
		System.out.println(count);
		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		/** reconstruct the path */
		return constructPath(start, goal, parentMap);	


	}

	
	
	public static void main(String[] args)
	{
	System.out.print("Making a new map...");
	/*	MapGraph theMap = new MapGraph();
		System.out.println("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
	*/
	/*	for(GeographicPoint point:theMap.getVertices()){
			List<Intersection> list = new ArrayList<>();
			list=theMap.getEdges(point);
			System.out.println("The point is :"+point+"  and the neigbours");
			for(Intersection intersection:list){
				System.out.println(intersection.getPoint() + "  The length :" + intersection.getLength());
			}
		}
	*/	
		
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		 */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		
		System.out.println("DONE.");
	/**
		
		Comparator<RoadStart> comparator = new RoadStart();
		PriorityQueue<RoadStart> toExplore = new PriorityQueue<>(comparator);
		Double i =0.0;
		GeographicPoint start = new GeographicPoint(1,1);
			toExplore.add(new RoadStart(start, i - 10.0));
			toExplore.add(new RoadStart(start, i + 12.0));
			toExplore.add(new RoadStart(start, i + 5.0));
			toExplore.add(new RoadStart(start, i + 2.0));
			toExplore.add(new RoadStart(start, i + 1.0));
			System.out.println(toExplore.size());
			int t = toExplore.size();
		for(int j=0;j<t;j++){
			System.out.println(toExplore.remove().getDistance());
		}
	*/		
		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		//List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		System.out.println("\n \n Hello" + route);
		
	
	}
	
}
