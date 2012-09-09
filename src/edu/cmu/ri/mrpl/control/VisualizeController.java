package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.RobotModel;
import edu.cmu.ri.mrpl.gui.PointsConsole;
import edu.cmu.ri.mrpl.kinematics2D.Angle;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import edu.cmu.ri.mrpl.kinematics2D.Units;

public class VisualizeController {

	public VisualizeController(){
		
	}
	public void updateRobotPos(PointsConsole pc,RealPose2D robotPose){
		pc.setReference(robotPose);
	}
	public void addPoint(PointsConsole pc, RealPoint2D pointToRobot){
		pc.addPoint(new RealPose2D(pointToRobot.getX(),pointToRobot.getY(),0.0));
	}
	// PMF: Putting points in Robot Frame for PointsConsole
	
//	for( int i = 0; i < sonars.length; ++i){
//		Angle a = new Angle(22.5*i*Units.degToRad);
//		RealPose2D sonar = new RealPose2D(sonars[i],0.0,0.0);
//		RealPose2D sonarToRobot = new RealPose2D(0.19*Math.cos(a.angleValue()),0.19*Math.sin(a.angleValue()),a.angleValue());
//		pc.addPoint(RealPose2D.multiply(sonarToRobot, sonar));
//	}
	// PMF: End using PointsConsole 
	public void updateVisualizer(PointsConsole pc, Robot r){
		pc.drawAll(r, RobotModel.ROBOT_RADIUS);
	}
}
