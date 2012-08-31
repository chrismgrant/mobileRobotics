package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Robot;
import java.io.*;


public class SonarController {
	
	double[] sonars;
	
	
	public SonarController(){
		sonars = new double[16];
	}
	
	
	public void updateSonars(double[] sonarVals){
		for (int i = 0; i < 16; i++){
			sonars[i] = (35*sonars[i]+65*sonarVals[i])/100; // Padding to dampen noise
//			System.out.println("True sonar: "+sonarVals[i]+", Padded sonar: "+sonars[i]);
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
		System.out.println("Shortest length: " + minVal);
		return minInd;
	}
	public double getForwardSonarReading(){
		return Math.min(Math.min(sonars[0],sonars[1]*Math.cos(Math.PI/8)),sonars[15]*Math.cos(Math.PI/8));
	}
//	public int determineClosestObject(double[] sonarVals){
//		
//	}
}
