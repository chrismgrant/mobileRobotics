package edu.cmu.ri.mrpl.control;

import java.util.Date;

import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.kinematics2D.Angle;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import edu.cmu.ri.mrpl.maze.MazeState;
/**
 * Handles updating position and orientation of robot at every step
 * @author WangHeli
 *
 */
public class BearingController {

    private static final double UPDATE_DISTANCE = 5.0;
	
	private RealPose2D pose;
	/**
	 * mazePose is a realPose relative to the maze origin, with units in m.
	 */
    private final RealPose2D initPose;
	private RealPose2D mazePose, lastPose, deltaPose, initMazePose;

	private Date clock;
	private long lastClock;
	private double xError;
	private double yError;
	private double thError;
    private double distLastUpdate;
	/**
	 * Creates new bearing controller and initializes values
	 */
	public BearingController(MazeState init, RealPose2D robotInit){
		pose = new RealPose2D();
		lastClock = 0;
		clock = new Date();
		xError = 0;
		yError = 0;
		thError = 0;
        distLastUpdate = 0;
        initPose = robotInit.clone();
        lastPose = initPose.clone();
		double x,y,th;
		x = Convert.mazeUnitToMeter(init.x());
		y = Convert.mazeUnitToMeter(init.y());
		switch (init.dir()){
		case East: {
			th = 0;
			break;
		}
		case North: {
			th = 1;
			break;
		}
		case West: {
			th = 2;
			break;
		}
		case South: {
			th = 3;
			break;
		}
		default:{
			th = -1;
			break;
		}
		}
		th = Convert.mazeDirectionToRadian(th);
		mazePose = new RealPose2D(x,y,th);
        initMazePose = mazePose.clone();

	}

    public RealPose2D getInitMazePose(){
        return initMazePose;
    }
	/**
	 * Updates robot's pose using linear and angular velocity.
	 * Robot velocity will be freshest for use here
	 * @param linearVelocity True linear velocity of robot
	 * @param angularVelocity True angular velocity of robot
	 */
	public void updateBearing(double linearVelocity, double angularVelocity){
		double x = pose.getX(), y = pose.getY();
		Angle th = new Angle(pose.getTh());
		lastClock = clock.getTime();
		clock.setTime(System.currentTimeMillis());
		Angle dth = new Angle(angularVelocity * (clock.getTime() - lastClock) /1000);
		th = new Angle(th.add(dth));
		double dr = linearVelocity * (clock.getTime() - lastClock) / 1000;
		double dx = dr * Math.cos(th.angleValue());
		x += dx;
		double dy = dr * Math.sin(th.angleValue());
		y += dy;
		pose.setPose(x,y,th.angleValue());
	}
	/**
	 * Gets x position of robot
	 * @return double x position
	 */
	public synchronized double getX(){
		return pose.getX();
	}
	/**
	 * Gets y position of robot
	 * @return double y position
	 */
	public synchronized double getY(){
		return pose.getY();
	}
	/**
	 * Gets direction of robot
	 * @return double direction, from 0 to 2Pi
	 */
	public synchronized double getDirection(){
		return pose.getTh();
	}
	/**
	 * Gets pose of robot
	 * @return pose of robot
	 */
	public synchronized RealPose2D getPose(){
		return pose.clone();
	}
	/**
	 * Gets the difference between last known pose and current pose.
	 * Updates after updateMazePoseByBearing is called
	 * @return difference between last and this pose
	 */
	public synchronized RealPose2D getDeltaPose(){
		return deltaPose;
	}
	/**
	 * Gets point of robot
	 * @return point of robot
	 */
	public synchronized RealPoint2D getPosition(){
		return getPose().getPosition();
	}
	/**
	 * Gets robot-encoded pose of robot
	 * @param r robot object
	 * @return calculated robot pose WRT world
	 */
	public synchronized static RealPose2D getRPose(Robot r){
		return new RealPose2D(r.getPosX(),r.getPosY(),r.getHeading());
	}
	/**
	 * Gets robot-encoded pose, error corrected
	 * Use as intended pose
	 * @param r robot object
	 * @return calculated intended robot pose WRT world
	 */
	public RealPose2D getRPoseWithError(Robot r){
		return new RealPose2D(r.getPosX() - xError,r.getPosY() - yError, Angle.normalize(r.getHeading()-thError));
	}
	/**
	 * Gets robot-encoded direction of robot
	 * @param r robot object
	 * @return calculated direction WRT world
	 */
	public static double getRDirection(Robot r){
		return r.getHeading();
	}
	/**
	 * Gets robot-encoded X of robot
	 * @param r robot object
	 * @return calculated x pos WRT world
	 */
	public static double getRX(Robot r){
		return r.getPosX();
	}
	/**
	 * Gets robot-encoded Y of robot
	 * @param r robot object
	 * @return calculated y pos WRT world
	 */
	public static double getRY(Robot r){
		return r.getPosY();
	}
	/**
	 * Updates error of robot's real pose from intended pose
	 * @param xErr error in X direction, in meters
	 * @param yErr error in Y direction, in meters
	 * @param thErr error in heading, in radians
	 */
	public void updateError(double xErr, double yErr, double thErr){
		xError += xErr;
		yError += yErr;
		thError = Angle.normalize(thError + thErr);
	}
	/**
	 * Gets X error of robot's real pose from intended pose
	 * @return x error in meters
	 */
	public double getXError(){
		return xError;
	}
	/**
	 * Gets Y error of robot's real pose from intended pose
	 * @return y error in meters
	 */
	public double getYError(){
		return yError;
	}
	/**
	 * Gets th error of robot's real pose from intended pose
	 * @return th error in radians
	 */
	public double getThError(){
		return thError;
	}
	/**
	 * Updates the robot's maze pose by providing a delta vector specifying how much x, y, and th have changed.
	 * The delta pose is vector-added to the maze pose.
	 * @param newRobotPose robot's new pose in world.
     * @return distance since last update. If 0, then robot needs updating
	 */
	public double updateMazePoseByBearing(RealPose2D newRobotPose){
        deltaPose = Convert.inverseMultiply(lastPose,newRobotPose);
        mazePose = Convert.multiply(mazePose,deltaPose);
		lastPose = newRobotPose.clone();
        distLastUpdate += deltaPose.getPosition().distance(0,0);
        if (distLastUpdate > UPDATE_DISTANCE) {
            distLastUpdate = 0;
        }
        return distLastUpdate;
	}
	/**
	 * Updates the robot's maze pose by looking at sonars, then correcting mazePose to match sonar readings to wall
	 * @param sonarOffset calculated offset using sonars. Done by trc. 
	 */
	public void updateMazePoseBySonar(RealPose2D sonarOffset){
        System.out.println("MazePose updated to: "+sonarOffset.toString());
        System.out.println("Error corrected: "+Convert.inverseMultiply(mazePose,sonarOffset).toString());
		mazePose = sonarOffset.clone();
	}
	/**
	 * Gets the mazePose of the robot, in meters relative to maze origin
	 * @return mazePose
	 */
	public RealPose2D getMazePose(){
		return mazePose.clone();
	}
	/**
	 * Gets mazePose of the robot, in Maze units.
	 * Primarily used for drawing
	 * @return double of [x,y, direction] for drawing robot
	 */
	public double[] getMazePoseInMazeCoordinates(){
		double[] ret = new double[3];
		ret[0] = Convert.meterToMazeUnit(mazePose.getX());
		ret[1] = Convert.meterToMazeUnit(mazePose.getY());
		ret[2] = Convert.radianToMazeDirection(mazePose.getTh());
        ret[2] = (ret[2] >= 0)? ret[2] : 4 + ret[2];
		return ret;
	}
    public RealPose2D getInitPose(){
        return initPose;
    }
}
