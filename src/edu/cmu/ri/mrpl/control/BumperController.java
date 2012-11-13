package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Robot;

public class BumperController {
	
	public static synchronized boolean isBumped(Robot robot){
		if (robot != null && robot.getBumpers()!=0) {
			System.err.println("detecting bumping!");
			return true;
		}
		else return false;
	}
	
}
