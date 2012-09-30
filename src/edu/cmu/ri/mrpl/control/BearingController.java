package edu.cmu.ri.mrpl.control;

import java.util.Date;

import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.kinematics2D.Angle;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
/**
 * Handles updating position and orientation of robot at every step
 * @author WangHeli
 *
 */
public class BearingController {

	private static final double meterToMazeUnit = 1.3576;//39.3701/29; 
	private static final double radianToMazeUnit = 2/Math.PI;
	
	private RealPose2D pose;
	/**
	 * mazePose is a realPose relative to the maze origin, with units in m.
	 */
	private RealPose2D mazePose;
	private RealPose2D lastPose;
	private RealPose2D initPose;

	private Date clock;
	private long lastClock;
	private double xError;
	private double yError;
	private double thError;
	/**
	 * Creates new bearing controller and initializes values
	 */
	public BearingController(){
		pose = new RealPose2D();
		lastClock = 0;
		clock = new Date();
		xError = 0;
		yError = 0;
		thError = 0;
		mazePose = new RealPose2D();
		lastPose = new RealPose2D();
		initPose = new RealPose2D();
	}
	public BearingController(Robot r){
		this();
		initPose = new RealPose2D(r.getPosX(),r.getPosY(),r.getHeading());
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
	 * Converts pose from WRT world to WRT robot
	 * @param robot robot object
	 * @param worldPose target pose WRT world
	 * @return target pose WRT robot
	 */
	public static RealPose2D WRTRobot(Robot robot, RealPose2D worldPose){
		return inverseMultiply(getRPose(robot), worldPose);
	}
	/**
	 * Converts point from WRT world to WRT robot
	 * @param robot robot object
	 * @param worldPoint target point WRT world
	 * @return target point WRT robot
	 */
	public static RealPoint2D WRTRobot(Robot robot, RealPoint2D worldPoint){
		return WRTRobot(robot, new RealPose2D(worldPoint.getX(),worldPoint.getY(),0)).getPosition();
	}
	/**
	 * Converts pose from WRT robot to WRT world
	 * @param robot robot object
	 * @param worldPose target pose WRT robot
	 * @return target pose WRT world
	 */
	public static RealPose2D WRTWorld(Robot robot, RealPose2D robotPose){
		return multiply(getRPose(robot), robotPose);
	}
	/**
	 * Converts point from WRT robot to WRT world
	 * @param robot robot object
	 * @param worldPoint target point WRT robot
	 * @return target point WRT world
	 */
	public static RealPoint2D WRTWorld(Robot robot, RealPoint2D worldPoint){
		return WRTWorld(robot, new RealPose2D(worldPoint.getX(),worldPoint.getY(),0)).getPosition();
	}
	/**
	 * Multiplies two poses together, as [a] x [b]
	 * @param a pose on left
	 * @param b pose on right
	 * @return resultant pose
	 */
	public static RealPose2D multiply(RealPose2D a, RealPose2D b){
		return RealPose2D.multiply(a,b);
	}
	/**
	 * Multiplies inverse of pose with other pose, as [a]^-1 x [b]
	 * @param a pose on left, to be inversed
	 * @param b pose on right
	 * @return resultant pose
	 */
	public static RealPose2D inverseMultiply(RealPose2D a, RealPose2D b){
		return RealPose2D.multiply(a.inverse(), b);
	}
	/**
	 * Updates the robot's maze pose by providing a delta vector specifying how much x, y, and th have changed.
	 * The delta pose is vector-added to the maze pose.
	 * @param newRobotPose robot's new pose in world
	 */
	public void updateMazePoseByBearing(RealPose2D newRobotPose){
		RealPose2D deltaPose = inverseMultiply(lastPose, newRobotPose);
		mazePose = multiply(mazePose,deltaPose);
	}
	/**
	 * Updates the robot's maze pose by looking at sonars, then correcting mazePose to match sonar readings to wall
	 * @param sonarOffset calculated offset using sonars. Done by trc. 
	 */
	public void updateMazePoseBySonar(RealPose2D sonarOffset){
		mazePose = multiply(mazePose,sonarOffset);
	}
	/**
	 * Gets the mazePose of the robot, in meters relative to maze origin
	 * @return mazePose
	 */
	public RealPose2D getMazePose(){
		return mazePose;
	}
	/**
	 * Gets mazePose of the robot, in Maze units.
	 * Primarily used for drawing
	 * @return double of [x,y, direction] for drawing robot
	 */
	public double[] getMazePoseInMazeCoordinates(){
		double[] ret = new double[3];
		ret[0] = mazePose.getX() * meterToMazeUnit;
		ret[1] = mazePose.getY() * meterToMazeUnit;
		ret[2] = mazePose.getTh() * radianToMazeUnit;
		return ret;
	}
}
