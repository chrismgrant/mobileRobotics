package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Robot;

public class WheelController {

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
		r.setVel(lVel + aVel,lVel - aVel);
	}
	public void pointToRelativeDirection(Robot r, int direction){
		double targetAVal;
		if (direction < 8){
			targetAVal = -direction;
		} else {
			targetAVal = direction - 16;
		}
		setAVel(targetAVal - getAVel(r));
	}
	public double getAVel(Robot r){
		return (r.getVelRight() - r.getVelLeft()) / .33;
	}
}
