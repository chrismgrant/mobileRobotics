package edu.cmu.ri.mrpl.control;

import java.util.Date;

import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;

public class BearingController {

	private double x, y, direction;
	private Date clock;
	private long lastClock;
	
	public BearingController(){
		x = 0;
		y = 0;
		direction = 0.0;
		lastClock = 0;
		clock = new Date();
	}
	public void updateBearing(double linearVelocity, double angularVelocity){
		lastClock = clock.getTime();
		clock.setTime(System.currentTimeMillis());
		double dd = angularVelocity * (clock.getTime() - lastClock) / 1000;
		direction += dd;
		if (direction > 2*Math.PI) {direction -= 2*Math.PI;}
		double dr = linearVelocity * (clock.getTime() - lastClock) / 1000;
		double dx = dr * Math.cos(direction);
		x += dx;
		double dy = dr * Math.sin(direction);
		y += dy;
	}
	public double getX(){
		return x;
	}
	public double getY(){
		return y;
	}
	public double getDirection(){
		return direction;
	}
	public RealPose2D getPose(){
		return new RealPose2D(x,y,direction);
	}
}
