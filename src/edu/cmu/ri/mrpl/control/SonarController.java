package edu.cmu.ri.mrpl.control;



public class SonarController {
	
	double[][] sonars;
	int current;
	
	public SonarController(){
		sonars = new double[3][16];
		current = 0;
	}
	
	
	public void updateSonars(double[] sonarVals){
		for (int i = 0; i < 16; i++){
			if (isOutOfRange(sonarVals[i])){
				sonars[current][i] = Double.POSITIVE_INFINITY;
			} else {
				sonars[current][i] = (isNoise(sonarVals[i], i)) ? sonars[current][i] : filter(sonarVals[i]);
			}
//			sonars[current][i] = (5*sonars[0][i]+5*sonars[1][i]+5*sonars[2][i]+15*sonarVals[i])/30; // Padding to dampen noise
			System.out.println(sonarVals[i] + ", "+sonars[current][i]);
		}
		current = (current == 2) ? 0 : current+1;
	}
	private double filter(double input){
		return Double.valueOf(String.format("%.2g%n", input));
	}
	private boolean isNoise(double sonarVal, int index){
		double filterVal = filter(sonarVal);
		return filterVal == sonars[0][index] && filterVal == sonars[1][index] && filterVal == sonars[2][index];
	}
	private boolean isOutOfRange(double sonarVal){
		return (sonarVal > 3.5) ? true : false;
	}
	public int getPosShortestSonar(){
		int minInd = 0;
		double minVal = Double.POSITIVE_INFINITY;
		for (int i = 0; i < 16; i++){
			if (minVal > sonars[current][i]){
				minVal = sonars[current][i];
				minInd = i;
			}
		}
		System.out.println("Shortest distance to sensor: " + minVal);
		System.out.println("Direction: " + minInd);
		return minInd;
	}
	public double getForwardSonarReading(){
		return Math.min(Math.min(sonars[current][0],sonars[current][1]*Math.cos(Math.PI/8)),sonars[current][15]*Math.cos(Math.PI/8));
	}
//	public int determineClosestObject(double[] sonarVals){
//		
//	}
}
