package edu.cmu.ri.mrpl.control;

import java.awt.Toolkit;

import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;

/**
 * 
 * @author WangHeli
 *
 */
public class Tracker {

	private boolean lost;
	private RealPoint2D robotCoord;
	/**
	 * Creates a tracker
	 * @param robotPos Position of tracker relative to robot
	 */
	public Tracker(RealPoint2D robotPos){
		updatePos(robotPos);
		lost = false;
	}
	/**
	 * Updates position of the tracker
	 * @param robotPos Position of tracker relative to robot
	 */
	public void updatePos(RealPoint2D robotPos){
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
	 * Gets RealPoint2D of tracker relative to robot
	 * @return relative position
	 */
	public RealPoint2D getRPos(){
		return robotCoord;
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
	/**
	 * Updates the robot-centric coordinates given a delta pose
	 * @param delta delta pose between robot poses
	 */
	public void updateRobotCoords(RealPose2D delta){
		robotCoord = Convert.inverseMultiply(delta, new RealPose2D(robotCoord.getX(),robotCoord.getY(),0)).getPosition();
	}
}
