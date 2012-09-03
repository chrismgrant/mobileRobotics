package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Robot;

public class WheelController {

	private static double SPEED = 1;
	private static double ROB_WIDTH = .355;
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
	}
	/**
	 * Points the robot in the direction specified. Utilizes current wheel speed for smooth turning
	 * @param r robot object
	 * @param direction direction, from 0-2pi anticlockwise, in radians. -1 specifies no direction
	 */
	public void pointToDirection(Robot r, double direction){
		double targetAVel, d = direction % (Math.PI);
		boolean isNeg = (direction < Math.PI) ? false : true;
		if (d <= -1){
			targetAVel = 0;
		} else if (d < Math.PI / 2){
			targetAVel = (isNeg) ? -SPEED * Math.PI/2 : d * SPEED;
		} else if (d <= Math.PI){
			targetAVel = (isNeg) ? (d - Math.PI) * SPEED : SPEED * Math.PI/2; 
		} else {
			targetAVel = 0;
		}
		System.out.println("Target speed: " + targetAVel);
		setAVel(targetAVel);
	}
	public double getLVel(){
		return lVel;
	}
	public double getAVel(){
		return aVel;
	}
	/**
	 * Returns the angular velocity of the robot
	 * @param r robot object
	 * @return angular velocity in radians per second
	 */
	public double getRobAVel(Robot r){
		double speed = (r.getVelRight() - r.getVelLeft()) / ROB_WIDTH;
		System.out.println("Current speed: " + speed + ", right wheel speed: " + r.getVelRight() + ", left wheel speed: " + r.getVelLeft());
		return speed;
	}
}
