package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import edu.cmu.ri.mrpl.maze.MazeWorld;

public final class Convert {
	private static final double meterToMazeUnit = 1.3576;//39.3701/29; 
	private static final double radianToMazeUnit = 2/Math.PI;
	private static final double T9inchesToMeters = 0.7366;
	/**
	 * Multiplies two poses together, as [a] x [b]
	 * @param a pose on left
	 * @param b pose on right
	 * @return resultant pose
	 */
	public static RealPose2D multiply(RealPose2D a, RealPose2D b){
		return RealPose2D.multiply(a,b);
	}
	/**
	 * Multiplies inverse of pose with other pose, as [a]^-1 x [b]
	 * @param a pose on left, to be inversed
	 * @param b pose on right
	 * @return resultant pose
	 */
	public static RealPose2D inverseMultiply(RealPose2D a, RealPose2D b){
		return RealPose2D.multiply(a.inverse(), b);
	}
	/**
	 * Converts pose from WRT world to WRT robot
	 * @param robotPose pose of robot WRT world
	 * @param goalPose target pose WRT world
	 * @return target pose WRT robot
	 */
	public static RealPose2D WRTRobot(RealPose2D robotPose, RealPose2D goalPose){
		return inverseMultiply(robotPose, goalPose);
	}
	/**
	 * Converts point from WRT world to WRT robot
	 * @param robotPose pose of robot WRT world
	 * @param goalPoint target point WRT world
	 * @return target point WRT robot
	 */
	public static RealPoint2D WRTRobot(RealPose2D robotPose, RealPoint2D goalPoint){
		return WRTRobot(robotPose, new RealPose2D(goalPoint.getX(),goalPoint.getY(),0)).getPosition();
	}
	/**
	 * Converts pose from WRT robot to WRT world
	 * @param robotPose pose of robot WRT world
	 * @param goalPose target pose WRT robot
	 * @return target pose WRT world
	 */
	public static RealPose2D WRTWorld(RealPose2D robotPose, RealPose2D goalPose){
		return multiply(robotPose, goalPose);
	}
	/**
	 * Converts point from WRT robot to WRT world
	 * @param robotPose pose of robot WRT world
	 * @param goalPoint target point WRT robot
	 * @return target point WRT world
	 */
	public static RealPoint2D WRTWorld(RealPose2D robotPose, RealPoint2D goalPoint){
		return WRTWorld(robotPose, new RealPose2D(goalPoint.getX(),goalPoint.getY(),0)).getPosition();
	}
	/**
	 * Converts distance to maze units
	 * @param meters input meters
	 * @return equivalent maze units
	 */
	public static double meterToMazeUnit(double meters){
		return meters * meterToMazeUnit;
	}
	/**
	 * Converts mazeUnits to distance in meters
	 * @param mazeUnit input units
	 * @return equivalent meters
	 */
	public static double mazeUnitToMeter(double mazeUnit){
		return mazeUnit/meterToMazeUnit;
	}
	/**
	 * Converts radians to maze directions
	 * @param radians input radians
	 * @return equivalent maze direction
	 */
	public static double radianToMazeDirection(double radians){
		return radians * radianToMazeUnit;
	}
	/**
	 * Converts maze directions to radians
	 * @param mazeDirection input maze direction
	 * @return equivalent radians
	 */
	public static double mazeDirectionToRadian(double mazeDirection){
		return mazeDirection/radianToMazeUnit;
	}
	/**
	 * Gets pose of robot
	 * @param r robot object
	 * @return RealPose2D of robot
	 */
	public static RealPose2D getRobotPose(Robot r){
		return new RealPose2D(r.getPosX(),r.getPosY(),r.getHeading());
	}
	
	/**
	 * 
	 * @param cellCenter
	 * @param trackerPositionWTM
	 * @return a direction
	 * 
	 * This function computes a MazeWorld direction based upon the cell center and a point
	 */
	public static MazeWorld.Direction getDirection(RealPoint2D cellCenter, RealPoint2D trackerPositionWTM){
			if (Math.abs(cellCenter.x - trackerPositionWTM.x) > (Math.abs(cellCenter.y - trackerPositionWTM.y))){
				if ((cellCenter.x - trackerPositionWTM.x) > 0){
					return MazeWorld.Direction.West;
				}
				else{
					return MazeWorld.Direction.East;
				}
			}
			else{
				if((cellCenter.y - trackerPositionWTM.y) > 0){
					return MazeWorld.Direction.South;
				}
				else{
					return MazeWorld.Direction.North;
				}
			}		
	} 
}
