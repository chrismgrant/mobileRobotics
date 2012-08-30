package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Robot;



public class SonarController {
	
	double[] sonars;
	Robot robot;
	
	public SonarController(Robot r){
		sonars = new double[16];
		robot = r;
	}
	
	
	public void updateSonars(double[] sonarVals){
		for (int i = 0; i < 16; i++){
			sonars[i] = (9*sonars[i]+sonarVals[i])/10; // Padding to dampen noise
		}
	}
	public int getPosShortestSonar(){
		int minInd = 0;
		double minVal = Double.POSITIVE_INFINITY;
		for (int i = 0; i < sonars.length; i++){
			if (minVal > sonars[i]){
				minVal = sonars[i];
				minInd = i;
			}
		}
		return minInd;
	}
}
