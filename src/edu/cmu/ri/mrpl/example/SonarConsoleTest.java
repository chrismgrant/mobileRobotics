package edu.cmu.ri.mrpl.example;

/*
 * Sample code for the 16x62 class taught at the Robotics Institute
 * of Carnegie Mellon University
 *
 * written from scratch by Martin Stolle in 2006
 *
 */

import javax.swing.*;

import edu.cmu.ri.mrpl.*;

public class SonarConsoleTest {

  public final static int NUM_SONARS = 16;

  public static void main(String[] argv) {

    final SonarConsole sc = new SonarConsole();

    javax.swing.SwingUtilities.invokeLater(new Runnable() {
      public void run() {
	JFrame frame = new JFrame("Sonar Console Random Test");

	frame.setLocationByPlatform(true);
	frame.getContentPane().add(sc);

	frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);

	frame.pack();
	frame.setVisible(true);
      }
    });

    double[] bogusSonar = new double[NUM_SONARS];

    while (true) {
      for (int ray=0; ray<NUM_SONARS; ray++)
	bogusSonar[ray] = (int)(Math.random()*(ray+1)*10);

      sc.setSonars(bogusSonar);
      try {
	Thread.sleep(500);
      } catch(InterruptedException iex) {
      }
    }
  }
}

