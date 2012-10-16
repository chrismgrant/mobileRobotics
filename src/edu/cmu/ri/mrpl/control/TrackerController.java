package edu.cmu.ri.mrpl.control;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.math.BigDecimal;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import edu.cmu.ri.mrpl.kinematics2D.*;
import fj.Effect;
import fj.data.List;

import static fj.data.List.fromString;

import fj.F;
import fj.F2;

import edu.cmu.ri.mrpl.maze.MazeState;
import edu.cmu.ri.mrpl.maze.MazeWorld;

/**
 * TrackerController handles judging where all objects are around the robot.
 * TC first reads in sonar readings and generates a point cloud.
 * Then, the point cloud gets cleaned to drop outliers, and converted into likely wall candidates.
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
    private static final double PRECISION = 0;
	private static final int TRACKER_MIN_COUNT = 2;
	private static final double EPSILON = .001;
	private static final double T9inchesToMeters = 0.7366;
    private static final double UPDATE_DISTANCE = .01;

    private List<Tracker> trackers;
	private List<Tracker> newTrackers;
	private List<Tracker> filteredTrackers;
	private MazeWorld mazeWorld;
    private RealPose2D last;
	private Tracker active;
	private Tracker follow;
	private int followLostCounter;
	private int ringCounter;
    private double lastSonarRecordDistance;

    public TrackerController(RealPose2D initPose, String in){
		ringCounter = 0;
        lastSonarRecordDistance = 0;
		trackers = List.list();
		newTrackers = List.list();
        last = initPose;
		active = null;
		follow = null;
		followLostCounter = 0;
		filteredTrackers = List.list();
		try {
			mazeWorld = new MazeWorld(in);
		} catch (IOException e) {}
	}
	/**
	 * Gets the maze world
	 * @return
	 */
	public MazeWorld getMaze(){
		return mazeWorld;
	}
	public MazeState getMazeInit(){
		Set<MazeState> inits = mazeWorld.getInits();
		for (MazeState i : inits){
			return i;
		}
		return null;
	}
	/**
	 * Adds trackers from 16 sonar readings
     * @param totalDistance total distance traveled since last update
	 * @param sonarReadings standard 16-array of sonar readings
	 */
	public void addTrackersFromSonar(double totalDistance, double[] sonarReadings){
        if (totalDistance - lastSonarRecordDistance > UPDATE_DISTANCE) {
			forceAddTrackersFromSonar(sonarReadings);
            lastSonarRecordDistance = totalDistance;
        }
	}

    /**
     * Adds trackers from 16 sonar readings regardless of last update
     * @param sonarReadings standard 16-array of sonar readings
     */
    public void forceAddTrackersFromSonar(double[] sonarReadings) {
        newTrackers = List.list();
        RealPoint2D position;
        double x,y,th;
        for (int i = 0; i < sonarReadings.length; i++){
            if (SonarController.isWithinRange(sonarReadings[i])){
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
	 * Update tracker states.
	 * Call after other TC setter methods to apply updates.
     * @param newPose deltaPose between robot's last pose and current pose.
	 */
	public void updateTrackers(RealPose2D newPose){
		//Update robot positions of old trackers
        final RealPose2D forwardDelta = Convert.inverseMultiply(last,newPose);
		trackers.foreach(new Effect<Tracker>() {
            public void e(Tracker t) {
                t.updateRobotCoords(forwardDelta);
            }
        });
        last = newPose;
//		System.out.println(newTrackers.length());
        if (newTrackers.length() > 0) {
            trackers = trackers.append(newTrackers);
            newTrackers = List.list();
        }
    }

    class PointCloudKey {
        long x, y;
        Tracker t;
        PointCloudKey(Tracker tracker) {
            t = tracker;
            double pow = Math.pow(10.0,PRECISION);
            x = Math.round(t.getX()*pow);
            y = Math.round(t.getY()*pow);
        }
        public boolean equals(Object other) {
            if (other == null) return false;
            if (!this.getClass().equals(other.getClass())) return false;
            PointCloudKey otherP = (PointCloudKey) other;
            return (x == otherP.x && y == otherP.y);
        }
        public int hashCode() {
            return (int) (x*y);
        }
    }
	/**
	 * Calculates the offset using the oldPose, sonar readings, and grid alignment
	 * getMazeOffset is called after new sonar readings are passed in
	 * @param oldMazePose pose relative to maze, in meters
	 * @return new pose relative to maze
	 */
	public RealPose2D getMazeCorrection(RealPose2D oldMazePose){

        //Begin point cloud filter
        int x,y;
        PointCloudKey p;
        RealPoint2D tempPoint;
        Map<PointCloudKey,Integer> pointCloud = new HashMap<PointCloudKey,Integer>();
        filteredTrackers = List.list();
        Line2D[] border = getBorder(oldMazePose);
        //Add trackers to point cloud
        for (Tracker t : trackers){
            if (isClose(border, oldMazePose, t)) {
                p = new PointCloudKey(t);
                if (pointCloud.containsKey(p)){
                    pointCloud.put(p, pointCloud.get(p)+1);
                } else {
                    pointCloud.put(p, 1);
                }
            }
        }
        System.out.print(pointCloud.size()+" trackers total, ");
        //Filter pointCloud
        for (Map.Entry<PointCloudKey, Integer> e : pointCloud.entrySet()){
            if (e.getValue() >= TRACKER_MIN_COUNT){
                filteredTrackers = filteredTrackers.cons(e.getKey().t);
            }
        }
        System.out.println("Filtered "+filteredTrackers.length()+" trackers.");

        System.out.println("PointError: "+getPointError(oldMazePose));
		//Compute gradient

        double lastError = Double.POSITIVE_INFINITY;
        RealPose2D nextPose = oldMazePose.clone();
        double nextError = getPointError(nextPose);
        double dx = 0, dy = 0, dth = 0;
        double[] gradient;
        while (nextError < lastError) {
            lastError = nextError;
            gradient = getGradient(nextPose);

            System.out.println("Gradient: ["+gradient[0]+","+gradient[1]+","+gradient[2]+"]");

            dx = -EPSILON * gradient[0];
            dy = -EPSILON * gradient[1];
            dth = -EPSILON * gradient[2];
            nextPose.add(dx,dy,dth);
            nextError = getPointError(nextPose);

            System.out.println("nextError: "+getPointError(nextPose));

        }
        nextPose.add(-dx,-dy,-dth);

        //Resets trackers after computing
        trackers = List.list();
        lastSonarRecordDistance = 0;
        return nextPose;
	}

    private boolean isClose(Line2D[] border, RealPose2D robotPose, Tracker t) {
        double distance, minDistance = Double.POSITIVE_INFINITY;
        Point2D holder = new Point2D.Double();
        for (Line2D l : border) {
            distance = LineSegment.closestPointOnLineSegment(l,Convert.WRTWorld(robotPose, t.getRPoint()),holder);
            if (distance < minDistance) {
                minDistance = distance;
            }
        }
        return Math.sqrt(minDistance) < .18;
    }

    private Line2D[] getBorder(final RealPose2D mazePose) {
        Line2D[] border = new Line2D[4];
        double half = T9inchesToMeters / 2;
        double x = mazePose.getX(), y = mazePose.getY();
        double ex = (x + half) % T9inchesToMeters, ey = (y + half) % T9inchesToMeters;
        double x1 = x - ex, y1 = y - ey;
        double x2 = x1 + T9inchesToMeters, y2 = y1 + T9inchesToMeters;
        border[0] = new Line2D.Double(x1,y1,x1,y2);
        border[1] = new Line2D.Double(x1,y1,x2,y1);
        border[2] = new Line2D.Double(x1,y2,x2,y2);
        border[3] = new Line2D.Double(x2,y1,x2,y2);
        return border;
    }

    private double[] getGradient(RealPose2D pose) {
        double[] gradient = new double[3];
        double dx, dy, dth;
        dx = pose.getX()+EPSILON;
        dy = pose.getY()+EPSILON;
        dth = Angle.normalize(pose.getRotateTheta()+EPSILON);
        gradient[0] = (getPointError(new RealPose2D(dx, pose.getY(),pose.getRotateTheta())) -
                getPointError(pose))/EPSILON;
        gradient[1] = (getPointError(new RealPose2D(pose.getX(),dy,pose.getRotateTheta())) -
                getPointError(pose))/EPSILON;

        gradient[2] = (getPointError(new RealPose2D(pose.getX(), pose.getY(),dth)) -
                getPointError(pose))/(EPSILON);
        return gradient;
    }
	private double getPointError(final RealPose2D inputPose){
		List<Double> offsets = filteredTrackers.map(new F<Tracker, Double>() {
			public Double f(Tracker t){
                RealPoint2D worldPoint = Convert.WRTWorld(inputPose, t.getRPoint());
                double xerr = Math.abs(worldPoint.getX()%T9inchesToMeters-T9inchesToMeters/2);
                double yerr = Math.abs(worldPoint.getY()%T9inchesToMeters-T9inchesToMeters/2);
                return xerr*yerr;

			}
		});
//        for (Double d : offsets){
//            System.out.print(d+",");
//        }
//        System.out.println(offsets.length());
		Double error = offsets.foldRight(new F2<Double,Double,Double>(){
			public Double f(Double a, Double b){
				return a+b;
			}
		}, 0.0);
		return error;
	}
	/**
	 * Adds walls where sonar readings suggest a wall will be.
	 * updateMazeWalls is called after robot position is set
	 */
	public void updateMazeWalls(RealPose2D mazePose){
		for (int i = 0; i < filteredTrackers.length(); i++){
			RealPoint2D convertedPoint = Convert.WRTWorld(mazePose, filteredTrackers.index(i).getRPoint());
			int x = (int) Math.rint(Convert.meterToMazeUnit(convertedPoint.x));
			int y = (int)Math.rint(Convert.meterToMazeUnit(convertedPoint.y));
			double distance =  Convert.mazeUnitToMeter((int)((x / T9inchesToMeters)));
			MazeWorld.Direction direction = Convert.getDirection(convertedPoint, filteredTrackers.index(i).getRPoint());
			if (!mazeWorld.isWall(x, y, direction)){
				mazeWorld.addWall(x, y, direction);
			}
		}
	}
	
	/**
	 * Gets all tracker positions relative to robot
	 * @return fj's List of RealPoint2D, relative to robot
	 */
	public List<RealPoint2D> getAllTrackerRPos(final RealPose2D robotPose){
		return trackers.map(new F<Tracker, RealPoint2D>() {
			public RealPoint2D f(Tracker t){
				return t.getRPoint() ;
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
				Point2D sol = t.getRPoint();
				return new RealPoint2D(sol.getX(),sol.getY()) ;
			}
		});
	}

    /**
     * Gets all filtered tracker positions relative to robot
     * @return fj's List of RealPoint2D, relative to robot
     */
    public List<RealPoint2D> getFilteredTrackerRPos(){
        return filteredTrackers.map(new F<Tracker, RealPoint2D>() {
            @Override
            public RealPoint2D f(Tracker tracker) {
                return tracker.getRPoint();
            }
        });
    }
    /**
     * Gets all filtered tracker positions relative to world
     * @return fj's List of RealPoint2D
     */
    public List<RealPoint2D> getFilteredTrackerWPos(final RealPose2D robotPose){
        return filteredTrackers.map(new F<Tracker, RealPoint2D>() {
            public RealPoint2D f(Tracker t){
                return Convert.WRTWorld(robotPose, t.getRPoint());
            }
        });
    }
	/**
	 * Gets all tracker positions relative to world
	 * @return fj's List of RealPoint2D 
	 */
	public List<RealPoint2D> getAllTrackerWPos(final RealPose2D robotPose){
		return trackers.map(new F<Tracker, RealPoint2D>() {
			public RealPoint2D f(Tracker t){
				return Convert.WRTWorld(robotPose, t.getRPoint());
			}
		});
	}
    public List<RealPoint2D> getNewTrackerWPos(final RealPose2D robotPose) {
        return newTrackers.map(new F<Tracker, RealPoint2D>() {
            @Override
            public RealPoint2D f(Tracker tracker) {
                return Convert.WRTWorld(robotPose,tracker.getRPoint());
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
