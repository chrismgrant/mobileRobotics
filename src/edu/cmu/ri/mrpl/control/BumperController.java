package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Robot;

public class BumperController {

	
	public BumperController(){
		
	}
	
	public boolean isBumped(Robot robot){
		if (robot.getBumpers()!=0) {
			System.err.println("detecting bumping!");
			return true;
		}
		else return false;
	}
	
}
