package edu.cmu.ri.mrpl.control;
import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.Path;

public class PathFinder extends Path {
	//creates a pathfinder given the robot centric x and y coord
	double minDistance = .5;
	double currentPositionX;
	double currentPositionY;
	double theta;
	double deltaR = 1.5;
	double pi = Math.PI;
	double turnConstant = 1;
	double objectPositionX;
	double objectPositionY;
	boolean objectInFront = false;
/**
 *The general pathfinder method store the current position (via robot centric x and y coords), 
 *theta with respect to the object, and the object's position (via object centric x and y coords). 
 *From there the method compares the minimum allowable distance to the objects position.
 *Helps to find if object is in front or not and then whether to change path or not.
 * @param xr
 * @param yr
 * @param theta
 * @param xo
 * @param yo
 */
	public PathFinder(double xr, double yr, double theta, double xo, double yo){
		currentPositionX = xr;
		currentPositionY = yr;
		objectPositionX = xo;
		objectPositionY = yo;
		if ((objectPositionX > minDistance) || (objectPositionY > minDistance)){
				objectInFront = false; 
				//if nothing in front go straight
				straightPath();
		}
		else{
			objectInFront = true;
			//if wall, check left and right
			if ((objectPositionY <= minDistance) && (objectPositionX <= minDistance)){
				rightTurn();
			}
			else if ((objectPositionY > minDistance) && (objectPositionX <= minDistance)){
				leftTurn();
			}
			else{
				uTurn();
			}
			// if dead end, u turn
			//adjust path
			// adjustPath method call
			adjustPath();
		}
	}
	public void straightPath(){
		currentPositionX += deltaR*Math.cos(theta);
		currentPositionY += deltaR*Math.sin(theta);
	}
	public void uTurn(){
		currentPositionX -= deltaR*Math.cos(theta);
		currentPositionY -= deltaR*Math.sin(theta);
	}
	//each turn a 90 degree turn
	public void rightTurn(){
		currentPositionX += deltaR*Math.cos(theta) + turnConstant*Math.cos(theta - pi / 2);	
		currentPositionY += deltaR*Math.sin(theta) + turnConstant*Math.sin(theta - pi / 2);	
	}
	public void leftTurn(){
		currentPositionX += deltaR*Math.cos(theta) + turnConstant*Math.cos(theta + pi / 2);	
		currentPositionY += deltaR*Math.sin(theta) + turnConstant*Math.sin(theta + pi / 2);	
	}
	public double[] adjustPath(){
		double[] adjustedPath= new double[3];
		adjustedPath[0] = currentPositionX;
		adjustedPath[1] = currentPositionY;
		adjustedPath[2] = theta;
		return adjustedPath;
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
