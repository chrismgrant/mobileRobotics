package edu.cmu.ri.mrpl.control;

import java.text.DecimalFormat;
import java.text.NumberFormat;




public class SonarController {
	
	private double[][] sonars;
	private double[] avgSonars;
	private NumberFormat filterFormat;
	private static double SONAR_TOLERANCE = 0.05;
	private int current;
	
	public SonarController(){
		sonars = new double[10][16];
		for (int i = 0; i < sonars.length; i++){
			for (int j = 0; j < sonars[i].length; j++){
				sonars[i][j] = Double.POSITIVE_INFINITY;
			}
		}
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
			if (isOutOfRange(sonarVals[i])){
				sonars[current][i] = Double.POSITIVE_INFINITY;
			} else {
				sonars[current][i] = (isNoise(sonarVals[i], i)) ? sonars[current][i] : filter(sonarVals[i]);
			}
			avgSonars[i] = getAvgReading(i);
			System.out.println(sonarVals[i] + ", "+avgSonars[i]);

		}
	}
	private double filter(double input){
		return Double.valueOf(filterFormat.format(input));
	}
	private boolean isNoise(double sonarVal, int index){
		double filterVal = filter(sonarVal);
		return (filterVal <= getAvgReading(index) + SONAR_TOLERANCE) && (filterVal >= getAvgReading(index) - SONAR_TOLERANCE);
	}
	private double getAvgReading(int index){
		double sum = 0;
		for (int i = 0; i < sonars.length; i++){
			sum += sonars[i][index];
		}
		return sum / sonars.length;
	}
	private boolean isOutOfRange(double sonarVal){
		return (sonarVal > 3.5) ? true : false;
	}
	public int getPosShortestSonar(){
		int minInd = 0;
		double minVal = Double.POSITIVE_INFINITY;
		for (int i = 0; i < avgSonars.length; i++){
			if (minVal > avgSonars[i]){
				minVal = avgSonars[i];
				minInd = i;
			}
		}
		System.out.println("Shortest distance to sensor: " + minVal);
		
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
