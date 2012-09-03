package edu.cmu.ri.mrpl.control;

import java.util.HashSet;
import java.util.Iterator;
import java.util.ArrayList;
import java.util.Set;
import java.util.Stack;

public class TrackerController {

	private static final double DISTANCE_MAX = 2.2;
	private static final double DISTANCE_TOLERANCE = .25;
	private Set<Tracker> trackers; //Map of trackers, 
	private Tracker active;
	private Tracker follow;
	
	public TrackerController(){
		trackers = new HashSet<Tracker>();
		active = null;
		follow = null;
	}
	
	public void addTracker(double distance, ArrayList<Integer> angleIndex){
		trackers.add(new Tracker(distance, angleIndex));
	}
	public void setActive(Tracker t){
		active = t;
	}
	public int getActiveDirection(){
		if (active == null) {return -1;}
		else {return active.getAngleIndex();}
	}
	public double getActiveDistance(){
		if (active == null) {return -1;}
		else {return active.getDistance();}
	}
	public Tracker getActiveTracker(){
		return active;
	}
	/**
	 * First account for all trackers. If any are out of range, then remove.
	 * Then, add trackers for new objects.
	 * @param sonarReadings readings of all 16 sonars, in meters
	 */
	public void updateTrackers(double[] sonarReadings){
//		System.out.println(trackers.size());
		Iterator<Tracker> iter = trackers.iterator();
		boolean[] accounted = new boolean[16];
		Tracker next, closest = null;
		Stack<Tracker> toRemove = new Stack<Tracker>();
		int lastDirection;
		while (iter.hasNext()){
			next = iter.next();
			lastDirection = next.getAngleIndex();
			int[] adjacentDirections = {
					adjacentDirection(lastDirection,0),
					adjacentDirection(lastDirection,1),
					adjacentDirection(lastDirection,-1),
					adjacentDirection(lastDirection,2),
					adjacentDirection(lastDirection,-2)
//					adjacentDirection(lastDirection,3),
//					adjacentDirection(lastDirection,-3)
			};
			double sumDistance = 0;
			ArrayList<Integer> newDirection = new ArrayList<Integer>();
			for (int dir : adjacentDirections){
				if (next.getDistanceError(sonarReadings[dir]) < DISTANCE_TOLERANCE){
					sumDistance += sonarReadings[dir];
					newDirection.add(dir);
					accounted[dir] = true;
				}
			}
			if (newDirection.size()>0){
				next.updatePos(sumDistance / newDirection.size(), newDirection);
			}
			
			if (active == null){
				if (((closest == null) ? 5 : closest.getDistance()) > next.getDistance()) {closest = next;}
			}
			if (next.getDistance() > DISTANCE_MAX){
				toRemove.push(next);
			}
		}
		for (int i = 0; i < 16; i++){
			if (!accounted[i] && sonarReadings[i] < DISTANCE_MAX){
				ArrayList<Integer> dir = new ArrayList<Integer>();
				dir.add(i);
				addTracker(sonarReadings[i],dir);
			}
		}
		while (!toRemove.empty()){
			removeTracker(toRemove.pop());
		}
		if (active == null){
			setActive(closest);
		}
	}
	private int adjacentDirection(int lastDirection, int delta){
		if (delta >= 0){
			int d = (lastDirection >= 16-delta) ? lastDirection - 16 + delta: lastDirection + delta;
//			System.out.println(d);
			return d;
		} else {
			int d = (lastDirection < -delta) ? 16 + delta + lastDirection : lastDirection + delta;
//			System.out.println(d);
			return d;
		}
	}
	public void removeTracker(Tracker t){
		trackers.remove(t);
	}
	/**
	 * Updates the tracker which robot is following.
	 * @param sonarReadings array of sonar readings to determine tracker position.
	 * @return Updated tracker. Null if tracker not initially found.
	 */
	public Tracker updateTracker(double[] sonarReadings){
		if (follow == null){//If tracker isn't set, find tracker

			ArrayList<Integer> hasObject = new ArrayList<Integer>();
			for (int i = 0; i < sonarReadings.length; i++){
				if (sonarReadings[i] < DISTANCE_MAX){
					hasObject.add(i);
				}
			}
			double minDist = Double.POSITIVE_INFINITY;
			int minDir = -1;
			for (int a : hasObject){
				if (sonarReadings[a] < minDist){
					minDist = sonarReadings[a];
					minDir = a;
				}
			}
			if (minDir != -1){//Make a tracker
				ArrayList<Integer> directions = new ArrayList<Integer>();
				directions.add(minDir);
				follow = new Tracker(sonarReadings[minDir],directions);
			}
			return follow;
		} else {//If tracker is set, update position
			boolean[] accounted = new boolean[16];
			int lastDirection = follow.getAngleIndex();
			int[] adjacentDirections = {
					adjacentDirection(lastDirection,0),
					adjacentDirection(lastDirection,1),
					adjacentDirection(lastDirection,-1),
					adjacentDirection(lastDirection,2),
					adjacentDirection(lastDirection,-2)
//					adjacentDirection(lastDirection,3),
//					adjacentDirection(lastDirection,-3)
			};
			double sumDistance = 0;
			ArrayList<Integer> newDirection = new ArrayList<Integer>();
			for (int dir : adjacentDirections){
				if (follow.getDistanceError(sonarReadings[dir]) < DISTANCE_TOLERANCE){
					sumDistance += sonarReadings[dir];
					newDirection.add(dir);
					accounted[dir] = true;
				}
			}
			if (newDirection.size()>0){
				follow.updatePos(sumDistance / newDirection.size(), newDirection);
			}
			return follow;
		}
	}
	public double getFollowDistance(){
		if (follow == null) {return -1;}
		else {return follow.getDistance();}
	}
	public int getFollowDirection(){
		if (follow == null) {return -1;}
		else {return follow.getAngleIndex();}
	}
	public Tracker getFollowTracker(){
		return follow;
	}
}
