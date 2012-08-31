package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Robot;

public class WheelController {

	private static double SPEED = .15;
	private static double ROB_WIDTH = .33;
	private double lVel;
	private double aVel;
	public Robot robot;

	
	public WheelController(Robot r){
		lVel = 0;
		aVel = 0;
		robot = r;
	}
	
	public void setLVel(double linearVel){
		lVel = linearVel;
	}
	
	public void setAVel(double angularVel){
		aVel = angularVel;
	}
	public void updateWheels(Robot r){
		r.setVel(lVel + aVel*ROB_WIDTH/2,lVel - aVel*ROB_WIDTH/2);
	}
	public void pointToRelativeDirection(Robot r, int direction){
		double targetAVal;
		if (direction == -1){
			targetAVal = 0;
		} else if (direction < 8) {
			targetAVal = -direction * SPEED * 22.5/2/Math.PI;	
		} else {
			targetAVal = (-direction + 16) * SPEED* 22.5/2/Math.PI;
		}
		System.out.println("Target speed: " + targetAVal);
		setAVel(targetAVal + getAVel(r));
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
