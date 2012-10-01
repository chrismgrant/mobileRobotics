package edu.cmu.ri.mrpl.control;

import java.awt.geom.Point2D;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

import fj.data.List;
import static fj.data.List.list;
import fj.data.Array;
import static fj.data.Array.array;
import fj.Effect;
import fj.F;  
import fj.F2;

import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import edu.cmu.ri.mrpl.kinematics2D.Units;
import edu.cmu.ri.mrpl.maze.MazeWorld;

/**
 * TrackerController handles judging where all objects are around the robot.
 * TC first reads in sonar readings and generates a point cloud.
 * Then, the point cloud gets cleaned to drop outliers, and converted into likely wall candidates.
 * Then, walls are placed (or [god forbid] removed) in a MazeWorld instance.
 * 
 * Trackers in the point cloud are stored relative to the robot.
 * As the robot moves, the trackers will be updated so that their position stays relative to the robot
 * @author WangHeli
 * 
 */
public class TrackerController {

	private static final double SONAR_ROBOT_RADIUS = .1905;
	private static final int TRACKER_DECAY = 10;
	private static final double DISTANCE_MAX = 2.2;
	private static final double DISTANCE_TOLERANCE = .35;
	private static final double DISTANCE_CLOSE_RANGE = .5;
	private static final double FAST_ANGULAR_SPEED = .3;
	private static final int LOST_COUNTER_THRESHOLD = 3;
	private static final int TRACKER_MIN_COUNT = 3;
	
	private Array<List<Tracker>> trackers;
	private List<Tracker> newTrackers;
	private List<Tracker> filteredTrackers;
	private MazeWorld mazeWorld;
//	private Set<Tracker> trackers; //Map of trackers, 
	private Tracker active;
	private Tracker follow;
	private int followLostCounter;
	private int ringCounter;
	
	public TrackerController(){
		ringCounter = 0;
		
		trackers = array();//Creates empty array
		for (int i = 0; i < TRACKER_DECAY; i++){//Adds T_DECAY many array indices with lists 
			List<Tracker> a = list();
			Array<List<Tracker>> b = array(a);
			trackers = trackers.append(b);
		}
		newTrackers = list();

		active = null;
		follow = null;
		followLostCounter = 0;
		filteredTrackers = list();
	}
	/**
	 * Adds trackers from 16 sonar readings
	 * @param sonarReadings standard 16-array of sonar readings
	 * @param robotPose pose of Robot
	 * @param ignore whether to ignore the current readings
	 */
	public void addTrackersFromSonar(double[] sonarReadings){
			newTrackers = list();
			RealPoint2D position;
			double x,y,th;
			for (int i = 0; i < sonarReadings.length; i++){
				if (!Double.isInfinite(sonarReadings[i])){
					th = i * 22.5*Units.degToRad;
					x = Math.cos(th)*(sonarReadings[i]+SONAR_ROBOT_RADIUS);
					y = Math.sin(th)*(sonarReadings[i]+SONAR_ROBOT_RADIUS);
					position = new RealPoint2D(x,y);
					addTracker(position);
				
			}
		}
	}
	/**
	 * Adds a tracker to the tracking list
	 * @param position Position of point relative to robot
	 */
	void addTracker(RealPoint2D position){
		Tracker newTracker = new Tracker(position);
		newTrackers = newTrackers.cons(newTracker);
	}
	/**
	 * Update tracker states. Removes any beyond decay time.
	 * Call after other TC setter methods to apply updates.
	 */
	public void updateTrackers(final RealPose2D delta){
		
		//Update robot positions of old trackers
		trackers.map(new F<List<Tracker>,List<Tracker>>() {
			public List<Tracker> f(List<Tracker> l){
				l.foreach(new Effect<Tracker>() {
					public void e(Tracker t){
						t.updateRobotCoords(delta);
					}
				});
				return l;
			}
		});
		
		trackers.set(ringCounter, newTrackers);//Remove old trackers and add new trackers
		ringCounter = (ringCounter >= TRACKER_DECAY-1) ? 0 : ringCounter+1;
		
		//Begin point cloud filter
		double x,y;
		RealPoint2D p;
		List<Tracker> trackerList = list();
		Map<RealPoint2D,Integer> pointCloud = new HashMap<RealPoint2D,Integer>();
		filteredTrackers = list();
		//Collapse array
		for (List<Tracker> l : trackers){
			trackerList.append(l);
		}
		//Add trackers to pointcloud
		for (Tracker t : trackerList){
			x = ((int)(t.getX()*100))/100.0;
			y = ((int)(t.getY()*100))/100.0;
			p = new RealPoint2D(x,y);
			if (pointCloud.containsKey(p)){
				pointCloud.put(p, pointCloud.get(p)+1);
			} else {
				pointCloud.put(p, 1);
			}
		}
		//Filter pointCloud
		for (Map.Entry<RealPoint2D, Integer> e : pointCloud.entrySet()){
			if (e.getValue() > TRACKER_MIN_COUNT){
				filteredTrackers.cons(new Tracker(e.getKey()));
			}
		}
	}
	/**
	 * Calculates the offset using the oldPose, sonar readings, and grid alignment
	 * getMazeOffset is called after new sonar readings are passed in
	 * @param oldMazePose
	 * @return Pose with offset from closest reading to walls
	 */
	public RealPose2D getMazeOffset(RealPose2D oldMazePose){
		
		//TODO complete
		return oldMazePose;
	}
	/**
	 * Adds walls where sonar readings suggest a wall will be.
	 * updateMazeWalls is called after robot position is set
	 */
	public void updateMazeWalls(){
		//TODO complete
	}
	/**
	 * Gets all tracker positions relative to robot
	 * @return fj's List of RealPoint2D, relative to robot
	 */
	public List<RealPoint2D> getAllTrackerRPos(final RealPose2D robotPose){
		List<Tracker> e = list();
		List<Tracker> l = list();
		if (trackers.isNotEmpty()){
//			Collection<List<Tracker>> a = trackers.toCollection();
//			for (List<Tracker> li : a) {
//				l=l.append(li);
//			}
			l = trackers.foldLeft(new F2<List<Tracker>, List<Tracker>, List<Tracker>>() {
				public List<Tracker> f(List<Tracker> out, List<Tracker> current){
					return out.append(current);
				}
			},e);
			System.out.println(l.length());
		}
		
		return l.map(new F<Tracker, RealPoint2D>() {
			public RealPoint2D f(Tracker t){
				Point2D sol = t.getRPos();
				return new RealPoint2D(sol.getX(),sol.getY()) ;
			}
		});
	}
	/**
	 * Gets all new tracker positions from this step relative to robot
	 * @return fj's List of RealPoint2D, relative to robot
	 */
	public List<RealPoint2D> getNewTrackerRPos(final RealPose2D robotPose){
		return newTrackers.map(new F<Tracker, RealPoint2D>() {
			public RealPoint2D f(Tracker t){
				Point2D sol = t.getRPos();
				return new RealPoint2D(sol.getX(),sol.getY()) ;
			}
		});
	}
	/**
	 * Gets all tracker positions relative to world
	 * @return fj's List of RealPoint2D 
	 */
	public List<RealPoint2D> getAllTrackerWPos(final RealPose2D robotPose){
		List<Tracker> e = list();
		List<Tracker> l = trackers.foldLeft(new F2<List<Tracker>, List<Tracker>, List<Tracker>>() {
			public List<Tracker> f(List<Tracker> out, List<Tracker> current){
				return out.append(current);
			}
		},e);
		return l.map(new F<Tracker, RealPoint2D>() {
			public RealPoint2D f(Tracker t){
				return Convert.WRTWorld(robotPose, t.getRPos());
			}
		});
	}
//	private int adjacentDirection(int lastDirection, int delta){
//		if (delta >= 0){
//			int d = (lastDirection >= 16-delta) ? lastDirection - 16 + delta: lastDirection + delta;
//			//System.out.print(d+",");
//			return d;
//		} else {
//			int d = (lastDirection < -delta) ? 16 + delta + lastDirection : lastDirection + delta;
//			//System.out.print(d+",");
//			return d;
//		}
//	}
//	/**
//	 * Updates the tracker which robot is following.
//	 * @param sonarReadings array of sonar readings to determine tracker position.
//	 * @return Updated tracker. Null if tracker not initially found.
//	 */
//	public Tracker updateTracker(double[] sonarReadings, double angularVelocity){
//		if (follow == null){//If tracker isn't set, find tracker
//
//			ArrayList<Integer> hasObject = new ArrayList<Integer>();//Get all non-infinite sonar readings
//			for (int i = 0; i < sonarReadings.length; i++){
//				if (sonarReadings[i] < DISTANCE_MAX){
//					hasObject.add(i);
//				}
//			}
//			double minDist = Double.POSITIVE_INFINITY; //Find smallest sonar reading
//			int minDir = -1;
//			for (int a : hasObject){
//				if (sonarReadings[a] < minDist){
//					minDist = sonarReadings[a];
//					minDir = a;
//				}
//			}
//			if (minDir != -1){//Make a tracker
//				ArrayList<Integer> directions = new ArrayList<Integer>();
//				directions.add(minDir);
//				follow = new Tracker(sonarReadings[minDir],directions);
//			}
//			return follow;
//		} else {//If tracker is set, update position
//			int isLost = 0;
//			int lastDirection = follow.getAngleIndex(true);
//			double lastDistance = follow.getDistance(true);
//			ArrayList<Integer> adjacentDirections = new ArrayList<Integer>();
//			adjacentDirections.add(adjacentDirection(lastDirection,0));
//			adjacentDirections.add(adjacentDirection(lastDirection,1));
//			adjacentDirections.add(adjacentDirection(lastDirection,-1));
//			adjacentDirections.add(adjacentDirection(lastDirection,2));
//			adjacentDirections.add(adjacentDirection(lastDirection,-2));
//
//			if (lastDistance < DISTANCE_CLOSE_RANGE){
//				adjacentDirections.add(adjacentDirection(lastDirection,3));
//				adjacentDirections.add(adjacentDirection(lastDirection,-3));
//			}
//			if (angularVelocity > FAST_ANGULAR_SPEED){
//				adjacentDirections.add(adjacentDirection(lastDirection,3));
//				adjacentDirections.add(adjacentDirection(lastDirection,4));
//			} else if (angularVelocity < -FAST_ANGULAR_SPEED){
//				adjacentDirections.add(adjacentDirection(lastDirection,-3));
//				adjacentDirections.add(adjacentDirection(lastDirection,-4));
//			}
//			
//			double sumDistance = 0;
//			ArrayList<Integer> newDirection = new ArrayList<Integer>();
//			for (int dir : adjacentDirections){
//				//System.out.print(dir);
//				if (follow.getDeltaDistance(sonarReadings[dir]) < 4*DISTANCE_TOLERANCE/3 && follow.getDeltaDistance(sonarReadings[dir]) > -2*DISTANCE_TOLERANCE/3){
//					sumDistance += sonarReadings[dir];
//					newDirection.add(dir);
//				} else if (follow.getDeltaDistance(sonarReadings[dir]) > 4*DISTANCE_TOLERANCE/3){
//					isLost++;
//				}
//			}
//			if (newDirection.size()>0){
//				follow.updatePos(sumDistance / newDirection.size(), newDirection);
//			}
//			if (isLost == ((lastDistance < DISTANCE_CLOSE_RANGE) ? 7 : 5)){
//				if (followLostCounter < LOST_COUNTER_THRESHOLD){
//					followLostCounter++;
//				} else {
//					ArrayList<Integer> dirTemp = new ArrayList<Integer>();
//					dirTemp.add(0);
//					if (follow.getDistance(true) > DISTANCE_MAX){
//						follow.updatePos(.5, dirTemp);
//					}
//					follow.lost();
//				}
//			} else {
//				followLostCounter = 0;
//			}
//			return follow;
//		}
//	}
//	/**
//	 * Gets distance between robot and follow tracker
//	 * @param ignoreLost ignore whether tracker is lost
//	 * @return distance, in meter
//	 */
//	public double getFollowDistance(boolean ignoreLost){
//		if (follow == null) {return -1;}
//		else {return follow.getDistance(ignoreLost);}
//	}
//	/**
//	 * Gets position of follow tracker
//	 * @return RealPoint2D of tracker's position
//	 */
//	public RealPoint2D getFollowPoint(){
//		if (follow == null) {return null;}
//		else return new RealPoint2D(follow.getX(),follow.getY());
//	}
//	/**
//	 * Gets direction of follow tracker
//	 * @param ignoreLost ignore whether tracker is lost
//	 * @return index of sonar tracker is closest to
//	 */
//	public int getFollowDirection(boolean ignoreLost){
//		if (follow == null) {return -1;}
//		else {return follow.getAngleIndex(ignoreLost);}
//	}
//	/**
//	 * Gets if follow tracker is lost
//	 * @return whether tracker is lost
//	 */
//	public boolean isFollowLost(){
//		return (follow == null)?true:follow.isLost();
//	}
}
