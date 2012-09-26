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

	private RealPose2D pose;
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
	}
	/**
	 * Updates robot's pose using linear and angular velocity.
	 * Robot velocity will be freshest for use here
	 * @param linearVelocity True linear velocity of robot
	 * @param angularVelocity True angular velocit of robot
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
		return RealPose2D.multiply(getRPose(robot).inverse(), worldPose);
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
		return RealPose2D.multiply(getRPose(robot), robotPose);
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
}
