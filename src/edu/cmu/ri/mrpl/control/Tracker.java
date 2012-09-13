package edu.cmu.ri.mrpl.control;

import java.awt.Toolkit;

import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;

/**
 * 
 * @author WangHeli
 *
 */
public class Tracker {

	private boolean lost;
	private RealPoint2D worldCoord;
	private RealPoint2D robotCoord;
	/**
	 * Creates a tracker
	 * @param worldPos Position of tracker relative to world
	 * @param robotPos Position of tracker relative to robot
	 */
	public Tracker(RealPoint2D worldPos, RealPoint2D robotPos){
		updatePos(worldPos, robotPos);
		lost = false;
	}
	/**
	 * Updates position of the tracker
	 * @param worldPos Position of tracker relative to world
	 * @param robotPos Position of tracker relative to robot
	 */
	public void updatePos(RealPoint2D worldPos, RealPoint2D robotPos){
		worldCoord = (RealPoint2D) worldPos.clone();
		robotCoord = (RealPoint2D) robotPos.clone();
		if (lost) {lost = false;}
	}
	public void lost(){
		if (!lost) {Toolkit.getDefaultToolkit().beep();}     
		lost = true;
	}
	public void found(){
		lost = false;
	}
	public boolean isLost(){
		return lost;
	}
	/**
	 * Gets distance from robot
	 * @param ignoreLost whether to ignore if tracker is lost
	 * @return distance, in meters
	 */
	public double getDistance(boolean ignoreLost){
		return (lost && !ignoreLost) ? -1 : robotCoord.distance(0, 0);
	}
	/**
	 * Gets index of sonar on sensor
	 * @param ignoreLost whether to ignore if tracker is lost
	 * @return sonar index
	 */
	public double getDirection(boolean ignoreLost){
		return (lost && !ignoreLost) ? -1 : Math.atan2(robotCoord.getY(), robotCoord.getX());
	}
	/**
	 * Gets x position relative to 0,0 of robot
	 * @return relative x position, in meters
	 */
	public double getX(){
		return robotCoord.getX();
	}
	/**
	 * Gets y position relative to 0,0 of robot
	 * @return relative y position, in meters
	 */
	public double getY(){
		return robotCoord.getY(); 
	}
	/**
	 * Gets x position relative to 0,0 of world
	 * @return absolute x position, in meters
	 */
	public double getWX(){
		return worldCoord.getX();
	}
	/**
	 * Gets y position relative to 0,0 of world
	 * @return absolute y position, in meters
	 */
	public double getWY(){
		return worldCoord.getY();
	}
	/**
	 * Gets RealPoint2D of tracker relative to robot
	 * @return relative position
	 */
	public RealPoint2D getRPos(){
		return robotCoord;
	}
	/**
	 * Gets RealPoint2D of tracker relative to world
	 * @return absolute position
	 */
	public RealPoint2D getWPos(){
		return worldCoord;
	}
	/**
	 * Returns the error between saved distance and new distance
	 * @param newPoint point of new object relative to robot
	 * @return error as (d'-d), where d' is new distance
	 */
	public double getDeltaDistance(RealPoint2D newPoint){
		return robotCoord.distance(newPoint); 
	}
	/**
	 * Returns error between last known angleIndex and specified angleIndex
	 * @param newPoint point of new object relative to robot
	 * @return Angular difference th'-th, where th' is new angle.
	 */
	public double getDeltaAngle(RealPoint2D newPoint){
		double oldTh = Math.atan2(robotCoord.getY(), robotCoord.getX());
		double newTh = Math.atan2(robotCoord.getY(), robotCoord.getX());
		return newTh - oldTh;
	}
}
