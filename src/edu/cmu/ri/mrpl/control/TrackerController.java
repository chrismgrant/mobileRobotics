package edu.cmu.ri.mrpl.control;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

public class TrackerController {

	private static double DISTANCE_TOLERANCE = .5;
	private Set<Tracker> trackers; //Map of trackers, 
	private Tracker active;
	
	public TrackerController(){
		trackers = new HashSet<Tracker>();
		active = null;
	}
	
	public void addTracker(double distance, int angleIndex){
		trackers.add(new Tracker(distance, angleIndex));
	}
	public void setActive(Tracker t){
		active = t;
	}
	public int getActiveDirection(){
		if (active == null) {return -1;}
		else {return active.getAngleIndex();}
	}
	/**
	 * First account for all trackers. If any are out of range, then remove.
	 * Then, add trackers for new objects.
	 * @param sonarReadings
	 */
	public void updateTrackers(double[] sonarReadings){
		Iterator<Tracker> iter = trackers.iterator();
		boolean[] accounted = new boolean[16];
		Tracker next, closest = null;
		int lastDirection;
		double measuredDistance;
		while (iter.hasNext()){
			next = iter.next();
			lastDirection = next.getAngleIndex();
			measuredDistance = sonarReadings[lastDirection];
			if (next.getDistanceError(measuredDistance) < DISTANCE_TOLERANCE){
				next.updatePos(measuredDistance, lastDirection);
				accounted[lastDirection] = true;
			} else {
				int[] adjacentDirections = {
						(lastDirection <= 1) ? 14 + lastDirection : lastDirection - 2,
						(lastDirection == 0) ? 15 : lastDirection - 1,
						(lastDirection == 15) ? 0 : lastDirection + 1,
						(lastDirection >= 14) ? lastDirection - 14: lastDirection + 2};
				for (int dir : adjacentDirections){
					if (next.getDistanceError(sonarReadings[dir]) < DISTANCE_TOLERANCE){
						next.updatePos(sonarReadings[dir], dir);
						accounted[dir] = true;
					}
				}
			}
			if (active == null){
				if (((closest == null) ? 5 : closest.getDistance()) > next.getDistance()) {closest = next;}
			}
			if (next.getDistance() > 3.3){
				removeTracker(next);
				iter.remove();
			}
		}
		for (int i = 0; i < 16; i++){
			if (!accounted[i] && sonarReadings[i] < DISTANCE_TOLERANCE){
				addTracker(sonarReadings[i],i);
			}
		}
		if (active == null){
			setActive(closest);
		}
	}
	public void removeTracker(Tracker t){
		trackers.remove(t);
	}
}
