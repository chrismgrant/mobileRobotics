package edu.cmu.ri.mrpl.control;
import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.Path;

public class PathFinder extends Path {
	//creates a pathfinder given the robot centric x and y coord
	double minDistance = .5;
	double currentPositionX;
	double currentPositionY;
	double theta;
	double objectPositionX;
	double objectPositionY;
	public PathFinder(double xr, double yr, double theta, double xo, double yo){
		currentPositionX = xr;
		currentPositionY = yr;
		objectPositionX = xo;
		objectPositionY = yo;
		if ((objectPositionX > minDistance) || (objectPositionY > minDistance)){
			
			
		}
	}
	public double getRobotX(){
		return currentPositionX;
	}
	public double getRobotY(){
		return currentPositionY;
	}
	public void setRobotX(double x){
		currentPositionX = x;
	}
	public void setRobotY(double y){
		currentPositionY = y;
	}
}
