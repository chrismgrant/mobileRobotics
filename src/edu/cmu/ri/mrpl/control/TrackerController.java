package edu.cmu.ri.mrpl.control;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import edu.cmu.ri.mrpl.kinematics2D.*;
import edu.cmu.ri.mrpl.maze.MazePos;
import fj.Effect;
import fj.data.Array;
import fj.data.List;

import static fj.data.List.fromString;
import static fj.data.List.list;

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
    private static final double PRECISION = 3;
	private static final int TRACKER_MIN_COUNT = 1;
    private static final int WALL_TRACKER_MIN = 10;
    private static final double EPSILON = .001;
	private static final double T9inchesToMeters = 0.7366;
    private static final double UPDATE_DISTANCE = .01;
    private static final double WALL_DISTANCE = .01;
    private static final double WALL_CORNER_CUTOFF = T9inchesToMeters / 4;
    private static final double MIN_HIT_RATIO = .8;
    private static final double MAX_HIT_RATIO = 1.1;
    private static final double WALL_EXPAND = .1;

    private List<Tracker> trackers;
	private List<Tracker> newTrackers;
	private List<Tracker> filteredTrackers;
    private List<Tracker> updateTrackers;
	private MazeWorld mazeWorld;
    private RealPose2D last;
    private Cell lastCell;
	private Tracker active;
	private Tracker follow;
	private int followLostCounter;
	private int ringCounter;
    private double lastSonarRecordDistance;
    private boolean isApproaching;

    public TrackerController(RealPose2D initPose, String in){
		ringCounter = 0;
        lastSonarRecordDistance = 0;
		trackers = List.list();
		newTrackers = List.list();
        isApproaching = false;
        last = initPose.clone();
		active = null;
		follow = null;
		followLostCounter = 0;
		filteredTrackers = List.list();
		try {
			mazeWorld = new MazeWorld(in);
		} catch (IOException e) {
            System.out.printf("Error: cannot read mazefile\n");
        }
        lastCell = new Cell(getMazeInit());

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
     * Sets the goals in mazeWorld to free Golds
     */
    void targetGold() {
        mazeWorld.removeAllGoals();
        //TODO for competition, first target all golds that other team can reach, then go for our gold.
        for (MazeState state : mazeWorld.getFreeGolds()) {
            mazeWorld.addGoal(state);
        }
    }

    /**
     * Sets the goals in mazeWorld to free Drops
     */
    void targetDrop() {
        mazeWorld.removeAllGoals();
        for (MazeState state : mazeWorld.getDropGolds()) {
            mazeWorld.addGoal(state);
        }
    }

    /**
     * Removes the gold at goldState from mazeWorld
     * Handles the symmetric case
     * @param goldState gold to be removed
     */
    void removeGold (MazeState goldState) {
        mazeWorld.removeGold(goldState);
    }

    /**
     * Removes the drop at dropState from mazeWorld
     * Handles the symmetric case
     * @param dropState drop to be removed
     */
    void removeDrop (MazeState dropState) {
        mazeWorld.removeDrop(dropState);
    }

    /**
     * Removes the goal at goalState from mazeWorld
     * @param goalState goal to be removed
     */
    void removeGoal (MazeState goalState) {
        if (mazeWorld.atGoal(goalState)) {
            mazeWorld.removeGoal(goalState);
        }
    }

	/**
	 * Adds trackers from 16 sonar readings
     * @param totalDistance total distance traveled since last update
	 * @param sonarReadings standard 16-array of sonar readings
	 */
	public void addTrackersFromSonar(RealPose2D robotPose, double totalDistance, double[] sonarReadings){
        newTrackers = list();
        if (totalDistance - lastSonarRecordDistance > UPDATE_DISTANCE) {
			forceAddTrackersFromSonar(robotPose, sonarReadings);
            lastSonarRecordDistance = totalDistance;
        }
	}

    /**
     * Adds trackers from 16 sonar readings regardless of last update
     * @param sonarReadings standard 16-array of sonar readings
     */
    public void forceAddTrackersFromSonar(RealPose2D robotPose, double[] sonarReadings) {
        newTrackers = List.list();
        RealPoint2D position;
        double x,y,th;
        for (int i = 0; i < sonarReadings.length; i++){
            if (SonarController.isWithinRange(sonarReadings[i])){
                th = i * 22.5*Units.degToRad;
                x = Math.cos(th)*(sonarReadings[i]+SONAR_ROBOT_RADIUS);
                y = Math.sin(th)*(sonarReadings[i]+SONAR_ROBOT_RADIUS);
                position = new RealPoint2D(x,y);
                addTracker(robotPose, position);

            }
        }
    }

	/**
	 * Adds a tracker to the tracking list
	 * @param position Position of point relative to robot
	 */
	void addTracker(RealPose2D robotPose, RealPoint2D position){
        Line2D[] border = getExpandBorder(robotPose);
        Line2D trackerVector = new Line2D.Double(robotPose.getPosition(),Convert.WRTWorld(robotPose,position));
        Point2D temp = new Point2D.Double();
        for (Line2D l : border) {
            if (LineSegment.intersectLines(l,trackerVector,temp)) {
                return;
            }
        }
		Tracker newTracker = new Tracker(position);
		newTrackers = newTrackers.cons(newTracker);
	}
	/**
	 * Update tracker states.
	 * Call after other TC setter methods to apply updates.
     * @param newPose deltaPose between robot's last pose and current pose.
     * @return true if wall changed
	 */
	public boolean updateTrackers(RealPose2D newPose){
        boolean wallChanged = false;
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
        }

        //Update walls if close to cell center
//        if (atCellEdge(newPose)) {
        if (atCellCenter(newPose)) {
            wallChanged = updateMazeWalls(newPose);
        }
        return wallChanged;
    }

    private boolean atCellCenter(RealPose2D newPose) {
        double half = T9inchesToMeters / 2;
        double x = (newPose.getX()+half) % T9inchesToMeters - half;
        double y = (newPose.getY()+half) % T9inchesToMeters - half;

        RealPoint2D point2D = new RealPoint2D(x,y);
        return (point2D.distance(0,0) < .1);
    }

    private boolean atCellEdge(RealPose2D newPose) {
        if (!lastCell.isInCell(newPose)) {
            System.out.println("Leaving cell "+lastCell.toString());

            lastCell.setCell(newPose);
            return true;
        }
        return false;
    }
    class Cell {
        int x, y;
        Cell(RealPose2D pose2D) {
            setCell(pose2D);
        }
        Cell(int x, int y) {
            setCell(x,y);
        }
        Cell(MazeState state) {
            this(state.pos());
        }
        Cell(MazePos position) {
            this(position.x(),position.y());
        }
        Cell() {
            this(0,0);
        }

        void setCell(int x, int y) {
            this.x = x;
            this.y = y;
        }
        void setCell(RealPose2D pose2D) {
            MazeState mazeState = Convert.RealPoseToMazeState(pose2D);
            setCell(mazeState.x(),mazeState.y());
        }
        int getX() {
            return x;
        }
        int getY() {
            return y;
        }
        boolean isInCell(RealPose2D pose2D) {
            MazeState mazeState = Convert.RealPoseToMazeState(pose2D);
            return (mazeState.x() == x && mazeState.y() == y);
        }
        public String toString() {
            return "x: "+x+", y: "+y;
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
        System.out.println("Current pose: "+oldMazePose.toString());
        System.out.println("PointError: "+getPointError(oldMazePose));
		//Compute gradient

        double lastError = Double.POSITIVE_INFINITY;
        RealPose2D nextPose = oldMazePose.clone();
        double nextError = getPointError(nextPose);
        double dx = 0, dy = 0, dth = 0;
        double[] gradient;
        while (lastError - nextError > 0.0005) {
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

    /**
     * Gets the border of the cell the mazePose is in
     * @param mazePose
     * @return array of four lines. [0] is east, [1] is north, [2] is west, [3] is south
     */
    private Line2D[] getBorder(final RealPose2D mazePose) {
        Line2D[] border = new Line2D[4];
        double half = T9inchesToMeters / 2;
        double x = mazePose.getX(), y = mazePose.getY();
        double ex = (x + half) % T9inchesToMeters, ey = (y + half) % T9inchesToMeters;
        double x1 = x - ex, y1 = y - ey;
        double x2 = x1 + T9inchesToMeters, y2 = y1 + T9inchesToMeters;
        border[0] = new Line2D.Double(x2,y1,x2,y2);//e
        border[1] = new Line2D.Double(x1,y2,x2,y2);//n
        border[2] = new Line2D.Double(x1,y1,x1,y2);//w
        border[3] = new Line2D.Double(x1,y1,x2,y1);//s
        return border;
    }

    private Line2D[] getExpandBorder(RealPose2D mazePose) {
        Line2D[] border = getBorder(mazePose);
        Line2D[] expBorder = new Line2D[4];
        double d = WALL_EXPAND;
        expBorder[0] = new Line2D.Double(border[0].getX1()+d,border[0].getY1()-d,border[0].getX2()+d,border[0].getY2()+d);
        expBorder[1] = new Line2D.Double(border[1].getX1()-d,border[1].getY1()+d,border[1].getX2()+d,border[1].getY2()+d);
        expBorder[2] = new Line2D.Double(border[2].getX1()-d,border[2].getY1()-d,border[2].getX2()-d,border[2].getY2()+d);
        expBorder[3] = new Line2D.Double(border[3].getX1()-d,border[3].getY1()-d,border[3].getX2()+d,border[3].getY2()-d);
        return expBorder;
    }

    private Line2D[] getShortBorder(RealPose2D mazePose) {
        Line2D[] border = getBorder(mazePose);
        Line2D[] shortBorder = new Line2D[4];
        double d = WALL_CORNER_CUTOFF;
        shortBorder[0] = new Line2D.Double(border[0].getX1(),border[0].getY1()+d,border[0].getX2(),border[0].getY2()-d);
        shortBorder[1] = new Line2D.Double(border[1].getX1()+d,border[1].getY1(),border[1].getX2()-d,border[1].getY2());
        shortBorder[2] = new Line2D.Double(border[2].getX1(),border[2].getY1()+d,border[2].getX2(),border[2].getY2()-d);
        shortBorder[3] = new Line2D.Double(border[3].getX1()+d,border[3].getY1(),border[3].getX2()-d,border[3].getY2());
        return shortBorder;
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
                if (xerr - yerr < .05) return 0.0;
                double err = Math.pow(Math.min(xerr,yerr),2);
                if (Double.isNaN(err)) return 0.0;
                return err;

			}
		});
//        for (Double d : offsets){
//            System.out.print(d+",");
//        }
//        System.out.println(offsets.length());
        offsets = offsets.filter(new F<Double, Boolean>() {
            @Override
            public Boolean f(Double aDouble) {
                return (aDouble != 0.0);
            }
        });
		Double error = offsets.foldRight(new F2<Double,Double,Double>(){
			public Double f(Double a, Double b){
				return a+b;
			}
		}, 0.0);
		return (offsets.length() == 0) ? 0 : Math.sqrt(error/offsets.length());
	}
	/**
	 * Adds walls where sonar readings suggest a wall will be.
	 * updateMazeWalls is called after robot position is set
     * @return true if wall is updated
	 */
	public boolean updateMazeWalls(RealPose2D mazePose){
        Line2D[] border = getShortBorder(mazePose);
        RealPoint2D point2D, temp = new RealPoint2D();
        double distance, half = T9inchesToMeters / 2;
        int cell_x = -1, cell_y = -1;
        boolean updated = false;
        MazeWorld.Direction direction = null;
        Array<Integer> trackerCount = Array.array(0,0,0,0);
        updateTrackers = list();

        if (trackers.length() > WALL_TRACKER_MIN) {
            // check each wall for trackers, and count 'em
            for (int i = 0; i < 4; i++) {
                // count how many trackers are on each wall
                trackerCount.set(i,0);
                for (Tracker t : trackers) {
                    point2D = Convert.WRTWorld(mazePose, t.getRPoint());
                    distance = LineSegment.closestPointOnLineSegment(border[i],point2D, temp);
                    if (distance < WALL_DISTANCE) {
                        trackerCount.set(i,trackerCount.get(i)+1);
                        updateTrackers = updateTrackers.cons(t);
                    }
                }
            }
        }
        Integer trackerCountAvg = trackerCount.foldLeft(new F2<Integer, Integer, Integer>() {
            @Override
            public Integer f(Integer integer, Integer integer1) {
                return integer+integer1;
            }
        }, 0)/4;
        //update walls
        for (int i = 0; i < 4; i++) {
            // add or remove walls as necessary
            cell_x = (int) ((mazePose.getX()+half) / T9inchesToMeters);
            cell_y = (int) ((mazePose.getY()+half) / T9inchesToMeters);
            switch (i) {
                case 0:
                    direction = MazeWorld.Direction.East;
                    break;
                case 1:
                    direction = MazeWorld.Direction.North;
                    break;
                case 2:
                    direction = MazeWorld.Direction.West;
                    break;
                case 3:
                    direction = MazeWorld.Direction.South;
            }

//            System.out.println(" Wall"+cell_x+","+cell_y+","+direction+" Trackers:"+trackerCount.get(i));
            if (trackerCountAvg > 0 && trackerCount.get(i)/(double) trackerCountAvg < MIN_HIT_RATIO) {
                if (mazeWorld.isWall(cell_x,cell_y,direction)) {
//                    System.out.print(" removed.");
                    updated = removeWall(cell_x,cell_y,direction);
                }
            }
            if (trackerCountAvg > 0 && trackerCount.get(i)/(double) trackerCountAvg > MAX_HIT_RATIO) {
                if (!mazeWorld.isWall(cell_x,cell_y, direction)) {
                    updated = true;
//                    System.out.print(" added.");
                    mazeWorld.addWall(cell_x,cell_y,direction);
                }
            }
//            System.out.print("\n");
        }
        System.out.printf("At cell %d, %d; Getting %d updates\n",cell_x, cell_y, updateTrackers.length());

        return updated;
	}

    private boolean removeWall(int x, int y, MazeWorld.Direction direction) {
        if (x == 0 && direction == MazeWorld.Direction.West) {
            return false;
        }
        if (x == mazeWorld.getWidth() - 1 && direction == MazeWorld.Direction.East) {
            return false;
        }
        if (y == 0 && direction == MazeWorld.Direction.South) {
            return false;
        }
        if (y == mazeWorld.getHeight() - 1 && direction == MazeWorld.Direction.North){
            return false;
        }
        mazeWorld.removeWall(x,y,direction);
        return true;
    }

	/**
	 * Gets all tracker positions relative to robot
	 * @return fj's List of RealPoint2D, relative to robot
	 */
	public List<RealPoint2D> getAllTrackerRPos(){
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
	public List<RealPoint2D> getNewTrackerRPos(){
		return newTrackers.map(new F<Tracker, RealPoint2D>() {
			public RealPoint2D f(Tracker t){
				RealPoint2D sol = t.getRPoint();
				return new RealPoint2D(sol) ;
			}
		});
	}
    public List<RealPoint2D> getUpdateTrackerRPos(){
        List<RealPoint2D> l = updateTrackers.map(new F<Tracker, RealPoint2D>() {
            @Override
            public RealPoint2D f(Tracker tracker) {
                RealPoint2D sol = tracker.getRPoint();
                return new RealPoint2D(sol);
            }
        });
        updateTrackers = list();
        return l;
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
