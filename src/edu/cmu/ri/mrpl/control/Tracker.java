package edu.cmu.ri.mrpl.control;

public class Tracker {

	double dist;
	int angIdx;
	
	public Tracker(double distance, int angleIndex){
		updatePos(distance, angleIndex);
	}
	
	public Tracker(){
		updatePos(0.0,0);
	}
	
	public void updatePos(double distance, int angleIndex){
		dist = distance;
		angIdx = angleIndex;
	}
	public double getDistance(){
		return dist;
	}
	public int getAngleIndex(){
		return angIdx;
	}
	/**
	 * Returns the error between saved distance and new distance
	 * @param distance double indicating distance of new object
	 * @return error as (d'-d)/d, where d' is new distance
	 */
	public double getDistanceError(double distance){
		return (distance - dist)/dist; 
	}
	/**
	 * Returns error between last known angleIndex and specified angleIndex
	 * @param angleIndex Index of sonar sensor
	 * @return Index difference between sonars.
	 */
	public int getAngleError(int angleIndex){
		int delta = angleIndex - angIdx;
		if (delta > 8) {return delta - 16;}
		else if (delta < -8) {return delta + 16;}
		else {return delta;}
	}
}
