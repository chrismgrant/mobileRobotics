package edu.cmu.ri.mrpl.control;

import java.util.Date;

import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.kinematics2D.Angle;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;

public class BearingController {

	private RealPose2D pose;
	private Date clock;
	private long lastClock;
	
	public BearingController(){
		pose = new RealPose2D();
		lastClock = 0;
		clock = new Date();
	}
	public void updateBearing(double linearVelocity, double angularVelocity){
		double x = pose.getX(), y = pose.getY();
		Angle th = new Angle(pose.getTh());
		lastClock = clock.getTime();
		clock.setTime(System.currentTimeMillis());
		Angle dth = new Angle(angularVelocity * (clock.getTime() - lastClock) / 1000);
		th = new Angle(th.add(dth));
		double dr = linearVelocity * (clock.getTime() - lastClock) / 1000;
		double dx = dr * Math.cos(th.angleValue());
		x += dx;
		double dy = dr * Math.sin(th.angleValue());
		y += dy;
		pose.setPose(x,y,th.angleValue());
	}
	public double getX(){
		return pose.getX();
	}
	public double getY(){
		return pose.getY();
	}
	public double getDirection(){
		return pose.getTh();
	}
	public RealPose2D getPose(){
		return pose;
	}
	public static RealPose2D getRPose(Robot r){
		return new RealPose2D(r.getPosX(),r.getPosY(),r.getHeading());
	}
}
