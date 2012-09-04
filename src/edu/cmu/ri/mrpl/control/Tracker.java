package edu.cmu.ri.mrpl.control;

import java.awt.Toolkit;
import java.util.ArrayList;

public class Tracker {

	double dist;
	ArrayList<Integer> angIdx;
	int size;
	boolean lost;
	
	public Tracker(double distance, ArrayList<Integer> angleIndex){
		updatePos(distance, angleIndex);
		size = 0;
		lost = false;
	}
	/**
	 * Update the position of the tracker
	 * @param distance distance from robot, in meters
	 * @param angleIndex index of sonar on sensor
	 */
	public void updatePos(double distance, ArrayList<Integer> angleIndex){
		dist = distance;
		angIdx = angleIndex;
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
	 * @return distance, in meters
	 */
	public double getDistance(boolean ignoreLost){
		return (lost && !ignoreLost) ? -1 : dist;
	}
	/**
	 * Gets index of sonar on sensor
	 * @return sonar index
	 */
	public int getAngleIndex(boolean ignoreLost){
		return (lost && !ignoreLost) ? -1 : angIdx.get(0);
	}
	/**
	 * Gets x position relative to 0,0 of robot
	 * @return relative x position, in meters
	 */
	public double getX(){
		return dist * Math.cos(angIdx.get(0)*22.5/180*Math.PI);
	}
	/**
	 * Gets y position relative to 0,0 of robot
	 * @return relative y position, in meters
	 */
	public double getY(){
		return dist * Math.sin(angIdx.get(0)*22.5/180*Math.PI); 
	}
	/**
	 * Returns the error between saved distance and new distance
	 * @param distance double indicating distance of new object
	 * @return error as (d'-d)/d, where d' is new distance
	 */
	public double getDistanceError(double distance){
		return (distance - dist); 
	}
	/**
	 * Returns error between last known angleIndex and specified angleIndex
	 * @param angleIndex Index of sonar sensor
	 * @return Index difference between sonars.
	 */
	public int getAngleError(int angleIndex){
		int delta = angleIndex - angIdx.get(0);
		if (delta > 8) {return delta - 16;}
		else if (delta < -8) {return delta + 16;}
		else {return delta;}
	}
}
