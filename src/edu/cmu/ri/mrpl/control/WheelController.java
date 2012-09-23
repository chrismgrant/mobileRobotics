package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;

/**
 * Controller handling wheels.
 * Should only deal with getting and setting wheel status
 * @author WangHeli
 *
 */
public class WheelController {

	static final double BRAKING_COEFFICIENT = 1;
	static final double SPEED = 1.2;
	static final double MIN_SPEED = .001;
	static final double MAX_SPEED = .886;
	static final double ROB_WIDTH = .355;
	private double lVel, aVel, curv, lWVel, rWVel;
	private static enum MOVE_FLAGS {WHEEL, AVEL, CURVVEL};
	private MOVE_FLAGS flag;
	
	/**
	 * Initialize flags for wheel controller
	 */
	public WheelController(){
		lVel = 0;
		aVel = 0;
		curv = 0;
		lWVel = 0;
		rWVel = 0;
		flag = MOVE_FLAGS.AVEL;
	}
	/**
	 * Sets individual wheel speeds
	 * @param leftVel Speed of left wheel, meters/s
	 * @param rightVel Speed of right wheel, meters/s
	 */
	public synchronized void setWheelVel(double leftVel, double rightVel){
		lWVel = leftVel;
		rWVel = rightVel;
		flag = MOVE_FLAGS.WHEEL;
	}
	/**
	 * Set angular and linear velocity
	 * @param angularVel angular velocity, rad/s
	 * @param linearVel linear velocity, m/s
	 */
	public synchronized void setALVel(double angularVel, double linearVel){
		setLVel(linearVel);
		setAVel(angularVel);
		flag = MOVE_FLAGS.AVEL;
	}
	/**
	 * Sets linear velocity of robot
	 * @param linearVel velocity, m/s
	 */
	private void setLVel(double linearVel){
		lVel = linearVel;
	}
	/**
	 * Sets angular velocity of robot
	 * @param angularVel angular velocity, rad/s, positive anticlockwise
	 */
	private void setAVel(double angularVel){
		aVel = angularVel;
	}
	/**
	 * Sets curvature and linear velocity of robot
	 * @param curvature curvature
	 * @param linearVel linear velocity, in m/s
	 */
	public synchronized void setCLVel(double curvature, double linearVel){
		setCurv(curvature);
		setLVel(linearVel);
		flag = MOVE_FLAGS.CURVVEL;
	}
	/**
	 * Sets curvature of robot path
	 * Overwrites setAVel
	 * @param curvature
	 */
	private void setCurv(double curvature){
		curv = curvature;
	}
	/**
	 * Sends velocity command to robot
	 * @param r robot object
	 */
	public synchronized void updateWheels(Robot r, boolean isBumped){
		if (!isBumped){
			switch(flag){
				case AVEL: {
					r.setVel(lVel - aVel*ROB_WIDTH/2,lVel + aVel*ROB_WIDTH/2);
					break;
				}
				case CURVVEL:{
					aVel = curv * lVel;
					r.setVel(lVel - aVel*ROB_WIDTH/2,lVel + aVel*ROB_WIDTH/2);
					break;
				}
				case WHEEL: {
					r.setVel(lWVel, rWVel);
					break;
				}
				default:{}
			}
		} else {
			r.setVel(0,0);
		}
	}
	/**
	 * Points the robot in the direction specified. Utilizes current wheel speed for smooth turning
	 * @param direction direction, from 0-2pi anticlockwise, in radians. -1 specifies no direction
	 */
	public void pointToDirection(double direction){
		double targetAVel, d = direction % (Math.PI);
		boolean isNeg = (direction < Math.PI) ? false : true;
		if (d < 0){
			targetAVel = 0;
		} else if (d < Math.PI / 2){
			targetAVel = (isNeg) ? -SPEED * Math.PI/2 : d * SPEED;
		} else if (d <= Math.PI){
			targetAVel = (isNeg) ? (d - Math.PI) * SPEED : SPEED * Math.PI/2; 
		} else {
			targetAVel = 0;
		}
		//System.out.println("Target speed: " + targetAVel);
		setAVel(targetAVel);
	}
	/**
	 * Gets target linear velocity of robot
	 * @return target linear velocity, m/s
	 */
	public double getLVel(){
		return lVel;
	}
	/**
	 * Gets target angular velocity of robot
	 * @return target angular velocity, radians/s
	 */
	public double getAVel(){
		return aVel;
	}
	/**
	 * Gets encoded linear velocity of robot
	 * @param r robot
	 * @return encoded linear velocity, m/s
	 */
	public synchronized static double getRobLVel(Robot r){
		return (r.getVelLeft()+r.getVelRight())/2;
	}
	/**
	 * Gets encoded angular velocity of robot
	 * @param r robot 
	 * @return angular velocity, radians per second
	 */
	public synchronized static double getRobAVel(Robot r){
		return (r.getVelRight() - r.getVelLeft()) / ROB_WIDTH;
	}
	/**
	 * Gets encoded velocity of robot left wheel
	 * @param r robot
	 * @return linear velocity, m/s
	 */
	public synchronized static double getRobLeftWheelVel(Robot r){
		return (r.getVelLeft());
	}
	/**
	 * Gets encoded velocity of robot right wheel
	 * @param r robot
	 * @return linear velocity, m/s
	 */
	public synchronized static double getRobRightWheelVel(Robot r){
		return (r.getVelRight());
	}
	
	
	/**
	 * Returns the linear velocity cap when approaching an object, to clamp max speed and halt oscillation
	 * @param distance distance from object to approach
	 * @param maxSpeed maximum speed for robot
	 * @param minSpeed minimum speed, below which the robot will simply stop
	 * @return speed to travel for appropriate distance
	 */
	public static double getCappedLVel(double distance, double maxSpeed, double minSpeed){
		return (distance > maxSpeed) ? maxSpeed : ((Math.abs(distance)<=minSpeed)? 0 : distance);
	}
}
