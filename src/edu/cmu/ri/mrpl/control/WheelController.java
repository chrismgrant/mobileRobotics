package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Robot;

public class WheelController {

	double lVel;
	double aVel;

	
	public WheelController(){
		lVel = 0;
		aVel = 0;
	}
	
	public void setLVel(double linearVel){
		lVel = linearVel;
	}
	
	public void setAVel(double angularVel){
		aVel = angularVel;
	}
	public void updateWheels(Robot robot){
		robot.setVel(lVel,lVel);
	}
}
