package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;

public class WheelController {

	private static final double BRAKING_COEFFICIENT = 1;
	private static final double SPEED = 1;
	private static final double MIN_SPEED = .03;
	private static final double ROB_WIDTH = .355;
	private double lVel;
	private double aVel;
	private double curv;
	
	public WheelController(){
		lVel = 0;
		aVel = 0;
		curv = 0;
	}
	/**
	 * Sets linear velocity of robot
	 * @param linearVel velocity, m/s
	 */
	public void setLVel(double linearVel){
		lVel = linearVel;
	}
	/**
	 * Sets angular velocity of robot
	 * @param angularVel angular velocity, rad/s, positive anticlockwise
	 */
	public void setAVel(double angularVel){
		aVel = angularVel;
	}
	/**
	 * Sets curvature of robot path
	 * Overwrites setAVel
	 * @param curvature
	 */
	public void setCurv(double curvature){
		curv = curvature;
		aVel = curv * lVel;
	}
	/**
	 * Sends velocity command to robot
	 * @param r robot object
	 */
	public void updateWheels(Robot r, boolean isBumped){
		if (!isBumped){
			r.setVel(lVel - aVel*ROB_WIDTH/2,lVel + aVel*ROB_WIDTH/2);
		} else {
			r.setVel(0,0);
		}
		//System.out.println("L: "+lVel+", A: "+aVel+", c: "+curv);
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
	public static double getRobLVel(Robot r){
		return (r.getVelLeft()+r.getVelRight())/2;
	}
	/**
	 * Gets encoded angular velocity of robot
	 * @param r robot 
	 * @return angular velocity, radians per second
	 */
	public static double getRobAVel(Robot r){
		return (r.getVelRight() - r.getVelLeft()) / ROB_WIDTH;
	}
	/**
	 * Gets encoded velocity of robot left wheel
	 * @param r robot
	 * @return linear velocity, m/s
	 */
	public static double getRobLeftWheelVel(Robot r){
		return (r.getVelLeft());
	}
	/**
	 * Gets encoded velocity of robot right wheel
	 * @param r robot
	 * @return linear velocity, m/s
	 */
	public static double getRobRightWheelVel(Robot r){
		return (r.getVelRight());
	}
	/**
	 * Shadows a point, maintaining a constant distance from the tracker
	 * @param p target point, robot-centric
	 * @param isLost whether reporting point is lost
	 * @param frontSonar distance front sonar is recording
	 * @param distance distance to maintain while shadowing, in meters. Set to zero to move to point
	 */
	public void shadowPoint(RealPoint2D p, boolean isLost, double frontSonar, double distance){
		try{
			if (!isLost){
				setCurv(1/calculateRadiusOfTurning(p));
				setLVel(getCappedLVel(Math.min(p.distance(0,0) - distance, frontSonar),SPEED,MIN_SPEED) / BRAKING_COEFFICIENT);
				if (getLVel() < .1){
					pointToDirection(Math.atan2(p.getY(), p.getX()));
				}	
			} else {
				setCurv(0);
				setLVel(0);
			}
			
		}catch(NullPointerException e){
			setCurv(0);
			setLVel(0);
		}
		
	}
	
	/**
	 * Determines radius of turning for a given point. Useful for determining curvature of path
	 * @param p Target point to turn to
	 * @return radius in m of how far to turn
	 */
	public static double calculateRadiusOfTurning(RealPoint2D p){
		return (Math.pow(p.getX(), 2)+Math.pow(p.getY(),2))/(2*p.getY());
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
