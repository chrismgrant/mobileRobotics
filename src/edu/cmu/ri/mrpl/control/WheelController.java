package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Robot;

public class WheelController {

	private static final double BRAKING_COEFFICIENT = 1;
	private static final double SPEED = 1;
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
		System.out.println("L: "+lVel+", A: "+aVel+", c: "+curv);
	}
	/**
	 * Points the robot in the direction specified. Utilizes current wheel speed for smooth turning
	 * @param direction direction, from 0-2pi anticlockwise, in radians. -1 specifies no direction
	 */
	public void pointToDirection(double direction){
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
	/**
	 * Shadows a tracker, maintaining a constant distance from the tracker
	 * @param t target tracker
	 * @param distance distance to maintain while shadowing, in meters
	 */
	public void shadowTracker(Tracker t, double distance){
		try{
			
			setCurv(1/calculateRadiusOfTurning(t.getX(),t.getY()));
			setLVel(Math.pow(t.getDistance() - distance, 1) / BRAKING_COEFFICIENT);
			if (getLVel() < .1){
				pointToDirection(t.getAngleIndex()*22.5/180*Math.PI);
			}
		}catch(NullPointerException e){
			setCurv(0);
			setLVel(0);
		}
		
	}
	public void moveToPoint(double x, double y){
		
	}
	public void moveToTracker(Tracker t){
		try{
			setCurv(1/calculateRadiusOfTurning(t.getX(), t.getY()));
			setLVel(Math.pow(t.getDistance(),1) / BRAKING_COEFFICIENT);
			if (getLVel() < .1){
				pointToDirection(t.getAngleIndex()*22.5/180*Math.PI);
			}
		} catch (NullPointerException e){
			setCurv(0);
			setLVel(0);
		}
	}
	private double calculateRadiusOfTurning(double x, double y){
		return (Math.pow(x, 2)+Math.pow(y,2))/(2*y);
	}
}
