package edu.cmu.ri.mrpl.control;



public class SonarController {
	
	double[] sonars;
	
	public SonarController(){
		sonars = new double[16];
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
