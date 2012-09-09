package edu.cmu.ri.mrpl.control;

import java.text.DecimalFormat;
import java.text.NumberFormat;




public class SonarController {
	
	private double[][] sonars;
	private double[] debounceVal;
	private int[] debounceBuffer;
	private double[] avgSonars;
	private NumberFormat filterFormat;
	private static final double SONAR_RANGE = 2.5;
	private static final double SONAR_TOLERANCE = 0.025;
	private static final int SONAR_NOISE_FILTER = 8;
	private static final double DEBOUNCE_TOLERANCE = 0.1;
	private static final int DEBOUNCE_SUSTAIN = 0;
	private int current;
	
	public SonarController(){
		sonars = new double[SONAR_NOISE_FILTER][16];
		for (int i = 0; i < sonars.length; i++){
			for (int j = 0; j < sonars[i].length; j++){
				sonars[i][j] = Double.POSITIVE_INFINITY;
			}
		}
		debounceVal = new double[16];
		debounceBuffer = new int[16];
		avgSonars = new double[16];
		for (int i = 0; i < avgSonars.length; i++){
			avgSonars[i] = Double.POSITIVE_INFINITY;
		}
		current = 0;
		filterFormat = new DecimalFormat("#0.00");
	}
	
	public void updateSonars(double[] sonarVals){
		current = (current == (sonars.length - 1)) ? 0 : current+1;
		for (int i = 0; i < avgSonars.length; i++){
			if (isNoise(sonarVals[i],i)){//If sonarvals is different, add to debounce buffer. else, feed to noise filter;
				//System.out.print("u");
				sonars[current][i] = filter(sonarVals[i]);
			} else {
				//System.out.print("d");
				debounce(filter(sonarVals[i]), i);
			} 
			
			avgSonars[i] = getAvgReading(i);
			//System.out.println(sonarVals[i] + ", "+avgSonars[i]);

		}
//		//System.out.println("["+debounceBuffer[0]+","+debounceBuffer[1]+","+debounceBuffer[2]+","+debounceBuffer[3]+","+debounceBuffer[4]+",]");
	}
	private void debounce(double sonarVal, int index){
		if (Math.abs(debounceVal[index] - sonarVal) > DEBOUNCE_TOLERANCE){//Set new debouncer
//			overwriteSonars(sonarVal, index);	
			debounceVal[index] = sonarVal;
			debounceBuffer[index] = 0;
			//System.out.print("#");
		} else if (debounceBuffer[index] < DEBOUNCE_SUSTAIN) {//Update debouncer
			debounceBuffer[index]++;
			//System.out.print("+");
		} else {//Record debouncer
			debounceBuffer[index]++;
			//System.out.print("*");
			overwriteSonars(sonarVal, index);
		}
	}
	private void overwriteSonars(double sonarVal, int index){
		if (isOutOfRange(sonarVal)){
			for (int i = 0; i < sonars.length; i++){
				sonars[i][index] = Double.POSITIVE_INFINITY;
			}
		} else {
			for (int i = 0; i < sonars.length; i++){
				sonars[i][index] = sonarVal;
			}
		}
	}
	private double filter(double input){
		return Double.valueOf(filterFormat.format(input));
	}
	private boolean isNoise(double sonarVal, int index){
//		//System.out.print((Math.abs(sonarVal - sonars[current][index]))+", ");
		Double d = sonarVal;
		return (d.isInfinite() && sonars[current][index] < SONAR_RANGE) || (Math.abs(sonarVal - sonars[current][index]) <= SONAR_TOLERANCE);
	}
	private double getAvgReading(int index){
		double sum = 0;
		for (int i = 0; i < sonars.length; i++){
			sum += sonars[i][index];
		}
		return sum / sonars.length;
	}
	private boolean isOutOfRange(double sonarVal){
		return (sonarVal > SONAR_RANGE) ? true : false;
	}
	private boolean isWithinRange (double sonarVal){
		return (sonarVal < SONAR_RANGE) ? true:false;
	}
	public int getPositionShortestSonar(){
		int minInd = 0;
		double minVal = Double.POSITIVE_INFINITY;
		for (int i = 0; i < avgSonars.length; i++){
			if (minVal > avgSonars[i]){
				minVal = avgSonars[i];
				minInd = i;
			}
		}
		//System.out.println("Shortest distance to sensor: " + minVal);
		
		return minInd;
	}
	public double[] getSonarReadings(){
		return avgSonars;
	}
	public double getForwardSonarReading(){
		return Math.min(Math.min(avgSonars[0],avgSonars[1]*Math.cos(Math.PI/8)),avgSonars[15]*Math.cos(Math.PI/8));
	}
//	public int determineClosestObject(double[] sonarVals){
//		
//	}
}
