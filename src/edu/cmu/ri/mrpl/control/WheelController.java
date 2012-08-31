package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Robot;

public class WheelController {

	private static double SPEED = .125;
	private static double ROB_WIDTH = .355;
	private double lVel;
	private double aVel;
	public Robot robot;

	
	public WheelController(Robot r){
		lVel = 0;
		aVel = 0;
		robot = r;
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
	 * Sends velocity command to robot
	 * @param r robot object
	 */
	public void updateWheels(Robot r){
		r.setVel(lVel - aVel*ROB_WIDTH/2,lVel + aVel*ROB_WIDTH/2);
	}
	/**
	 * Points the robot in the direction specified. Utilizes current wheel speed for smooth turning
	 * @param r robot object
	 * @param direction direction, from 0-2pi anticlockwise, in radians. -1 specifies no direction
	 */
	public void pointToDirection(Robot r, double direction){
		double targetAVel;
		if (direction <= -1){
			targetAVel = 0;
		} else if (direction < Math.PI) {
			targetAVel = (direction) * SPEED;	
		} else if (direction < 2*Math.PI) {
			targetAVel = ((direction - 2 * Math.PI)) * SPEED;
		} else {
			targetAVel = 0;
		}
		System.out.println("Target speed: " + targetAVel);
		if (direction != 0 && direction != -1){
			setAVel(targetAVel );
		} else {
			setAVel(targetAVel);
		}
	}
	/**
	 * Returns the angular velocity of the robot
	 * @param r robot object
	 * @return angular velocity in radians per second
	 */
	public double getAVel(Robot r){
		double speed = (r.getVelRight() - r.getVelLeft()) / ROB_WIDTH;
		System.out.println("Current speed: " + speed + ", right wheel speed: " + r.getVelRight() + ", left wheel speed: " + r.getVelLeft());
		return speed;
	}
}
