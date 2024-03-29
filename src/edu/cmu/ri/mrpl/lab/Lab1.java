//package edu.cmu.ri.mrpl.lab;
///*
//
//* Sample code for the 16x62 class taught at the Robotics Institute
//* of Carnegie Mellon University
//*
//* written from scratch by Martin Stolle in 2006
//*
//* inspired by code started by Illah Nourbakhsh and used
//* for many years.
//*/
//
//import java.io.*;
//import java.awt.*;
//import java.awt.event.*;
//import javax.swing.*;
//
//import edu.cmu.ri.mrpl.*;
//import edu.cmu.ri.mrpl.Robot;
//import edu.cmu.ri.mrpl.control.BearingController;
//import edu.cmu.ri.mrpl.control.BumperController;
//import edu.cmu.ri.mrpl.control.SonarController;
//import edu.cmu.ri.mrpl.control.TrackerController;
//import edu.cmu.ri.mrpl.control.WheelController;
//
//public class Lab1 extends JFrame implements ActionListener, TaskController {
//
//	private Robot robot;
//	private SonarConsole sc;
//	private SonarConsole sc2;
//	private JFrame scFrame;
//	private JFrame sc2Frame;
//
//	private JButton connectButton;
//	private JButton disconnectButton;
//
//	private JButton rotateButton;
//	private JButton forwardButton;
//	private JButton bothButton;
//
//	private JButton stopButton;
//	private JButton quitButton;
//	private JButton button3;
//	private JButton button4;
//
//	private JTextField textField1;
//	private JTextField textField2;
//
//	private WheelController wc;
//	private SonarController soc;
//	private BumperController bc;
//	private TrackerController trc;
//	private BearingController bac;
//
//	private RotateTask programTask;
//	private ForwardTask sonarTask;
//	private TrackTask bothTask;
//
//	private Task curTask = null;
//
//	public Lab1() {
//		super("Lab1");
//
//		connectButton = new JButton("connect");
//		disconnectButton = new JButton("disconnect");
//
//		rotateButton = new JButton("Run Rotate!");
//		forwardButton = new JButton("Run Forward!");
//		bothButton = new JButton("Run Track!");
//		stopButton = new JButton(">> stop <<");
//		quitButton = new JButton(">> quit <<");
//
//		button3 = new JButton("button3");
//		button4 = new JButton("button4");
//		textField1 = new JTextField("textField1");
//		textField2 = new JTextField("textField2");
//
//		connectButton.addActionListener(this);
//		disconnectButton.addActionListener(this);
//
//		rotateButton.addActionListener(this);
//		forwardButton.addActionListener(this);
//		bothButton.addActionListener(this);
//		stopButton.addActionListener(this);
//		quitButton.addActionListener(this);
//
//		button3.addActionListener(this);
//		button4.addActionListener(this);
//
//		Container main = getContentPane();
//		main.setLayout(new BoxLayout(main, BoxLayout.Y_AXIS));
//		Box box;
//
//		main.add(Box.createVerticalStrut(30));
//
//		box = Box.createHorizontalBox();
//		main.add(box);
//		box.add(Box.createHorizontalStrut(30));
//		box.add(connectButton);
//		box.add(Box.createHorizontalStrut(30));
//		box.add(disconnectButton);
//		box.add(Box.createHorizontalStrut(30));
//
//		main.add(Box.createVerticalStrut(30));
//
//		box = Box.createHorizontalBox();
//		main.add(box);
//		box.add(Box.createHorizontalStrut(30));
//		box.add(rotateButton);
//		box.add(Box.createHorizontalStrut(30));
//		box.add(forwardButton);
//		box.add(Box.createHorizontalStrut(30));
//
//		main.add(Box.createVerticalStrut(30));
//
//		box = Box.createHorizontalBox();
//		main.add(box);
//		box.add(bothButton);
//
//		main.add(Box.createVerticalStrut(30));
//
//		box = Box.createHorizontalBox();
//		main.add(box);
//		box.add(Box.createHorizontalStrut(30));
//		box.add(button3);
//		box.add(Box.createHorizontalStrut(30));
//		box.add(textField1);
//		box.add(Box.createHorizontalStrut(30));
//
//		main.add(Box.createVerticalStrut(30));
//
//		box = Box.createHorizontalBox();
//		main.add(box);
//		box.add(Box.createHorizontalStrut(30));
//		box.add(button4);
//		box.add(Box.createHorizontalStrut(30));
//		box.add(textField2);
//		box.add(Box.createHorizontalStrut(30));
//
//		main.add(Box.createVerticalStrut(30));
//
//		box = Box.createHorizontalBox();
//		main.add(box);
//		box.add(stopButton);
//		box.add(Box.createHorizontalStrut(30));
//		box.add(quitButton);
//
//		main.add(Box.createVerticalStrut(30));
//
//
//		this.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
//		this.addWindowListener(new WindowAdapter() {
//			public void windowClosing(WindowEvent e) {
//				quit();
//			}
//		});
//
//
//		this.setLocationByPlatform(true);
//		this.pack();
//		this.setVisible(true);
//
//		// construct the SonarConsole, but don't make it visible
//		scFrame = new JFrame("Sonar Console");
//		scFrame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
//		sc = new SonarConsole();
//		scFrame.add(sc);
//		scFrame.pack();
//
//		sc2Frame = new JFrame("Live Sonar Console");
//		sc2Frame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
//		sc2 = new SonarConsole();
//		sc2Frame.add(sc2);
//		sc2Frame.pack();
//
//		// construct tasks
//		programTask = new RotateTask(this);
//		sonarTask = new ForwardTask(this);
//		bothTask = new TrackTask(this);
//
//
//
//		robot=new SimRobot();
//		//robot=new ScoutRobot();
//	}
//
//	// call from GUI thread
//	public void connect() {
//		try {
//			robot.connect();
//		} catch(IOException ioex) {
//			System.err.println("couldn't connect to robot:");
//			ioex.printStackTrace();
//		}
//	}
//
//	// call from GUI thread
//	public void disconnect() {
//		try {
//			robot.disconnect();
//		} catch(IOException ioex) {
//			System.err.println("couldn't disconnect from robot:");
//			ioex.printStackTrace();
//		}
//	}
//
//
//	// call from GUI thread
//	public synchronized void stop() {
//		if (curTask!=null)
//			curTask.pleaseStop();
//	}
//
//	// call from GUI thread
//	public void quit() {
//
//		synchronized(this) {
//			if (curTask!=null) {
//				curTask.pleaseStop();
//
//				try {
//					this.wait(1000);
//				} catch(InterruptedException iex) {
//					System.err.println("interrupted while waiting for worker termination signal");
//				}
//			}
//
//			if (curTask!=null) {
//				System.err.println("exiting even though robot is running!");
//			} else {
//				disconnect();
//			}
//		}
//
//		System.exit(0);
//	}
//
//	// invoked from non-GUI thread to hide/show SonarConsole
//	public void showSC() {
//		SwingUtilities.invokeLater(new Runnable() {
//			public void run() {
//				scFrame.setLocationByPlatform(true);
//				scFrame.setVisible(true);
//				sc2Frame.setLocationByPlatform(true);
//				sc2Frame.setVisible(true);
//			}
//		});
//	}
//
//	// invoked from non-GUI thread to hide/show SonarConsole
//	public void hideSC() {
//		SwingUtilities.invokeLater(new Runnable() {
//			public void run() {
//				scFrame.setVisible(false);
//				sc2Frame.setVisible(false);
//			}
//		});
//	}
//
//	public void actionPerformed(ActionEvent e) {
//		Object source = e.getSource();
//		if (source==connectButton) {
//			connect();
//		} else if ( source==disconnectButton ) {
//			disconnect();
//		} else if ( source==stopButton ) {
//			stop();
//		} else if ( source==quitButton ) {
//			quit();
//		} else if ( source==rotateButton ) {
//			(new Thread(programTask)).start();
//		} else if ( source==forwardButton ) {
//			(new Thread(sonarTask)).start();
//		} else if ( source==bothButton ) {
//			(new Thread(bothTask)).start();
//		} else if ( source==button3) {
//
//		}
//	}
//
//	public synchronized boolean canStart(Task t) {
//		if (curTask!=null)
//			return false;
//
//		curTask = t;
//		return true;
//	}
//
//	public synchronized void finished(Task t) {
//		if (curTask!=t) {
//			System.err.println("ignoring finished() from unknown task "+t+"!");
//			return;
//		}
//		curTask=null;
//		this.notifyAll();
//	}
//
//	public static void main(String[] argv) {
//		javax.swing.SwingUtilities.invokeLater(new Runnable() {
//			public void run() {
//				new Lab1();
//			}
//		});
//	}
//
//
//	// here we implement the different robot controllers
//	// as Tasks
//	//
//
//	class RotateTask extends Task {
//
//		RotateTask(TaskController tc) {
//			super(tc);
//			wc = new WheelController();
//			soc = new SonarController();
//			bc = new BumperController();
//			bac = new BearingController();
//		}
//
//		public void taskRun() {
//			showSC();
//			robot.turnSonarsOn();
//
//			double[] sonars = new double[16];
//			double direction = 0;
//			wc.setALVel(0, 0);
//			try{
//				FileWriter outFileRawSonar = new FileWriter("RotRawSonarData");
//				FileWriter outFileFiltSonar = new FileWriter("RotFiltSonarData");
//				PrintWriter outRawSonar = new PrintWriter(outFileRawSonar);
//				PrintWriter outFiltSonar = new PrintWriter(outFileFiltSonar);
//				while(!shouldStop()) {
//					robot.updateState();
//					robot.getSonars(sonars);
//					soc.updateSonars(sonars);
//					sc.setSonars(soc.getSonarReadings());
//					sc2.setSonars(sonars);
//					bac.updateBearing(wc.getLVel(), wc.getAVel());
//					direction = soc.getPositionShortestSonar() * 22.5 * Math.PI/180;
//					System.out.println("Shortest sensor: " + soc.getPositionShortestSonar());
//					System.out.println("Target direction: " + direction);
//					wc.pointToDirection(direction);
//					wc.updateWheels(robot, bc.isBumped(robot));
//					try {
//						Thread.sleep(50);
//					} catch(InterruptedException iex) {
//						System.out.println("sample program sleep interrupted");
//					}
//				}
//				outRawSonar.close();
//				outFiltSonar.close();
//			} catch (IOException e){
//				e.printStackTrace();
//			}
//			robot.turnSonarsOff();
//			robot.setVel(0,0);
//			hideSC();
//		}
//
//		public String toString() {
//			return "sample program";
//		}
//	}
//
//	class ForwardTask extends Task {
//
//		ForwardTask(TaskController tc) {
//			super(tc);
//			wc = new WheelController();
//			soc = new SonarController();
//			bc = new BumperController();
//			bac = new BearingController();
//		}
//
//		public void taskRun() {
//			showSC();
//			robot.turnSonarsOn();
//			wc.setALVel(0,0);
//			double[] sonars = new double[16];
//			double speed = 0, target = 0;
//			try{
//				FileWriter outFileRawSonar = new FileWriter("FwdRawSonarData");
//				FileWriter outFileFiltSonar = new FileWriter("FwdFiltSonarData");
//				PrintWriter outRawSonar = new PrintWriter(outFileRawSonar);
//				PrintWriter outFiltSonar = new PrintWriter(outFileFiltSonar);
//				while(!shouldStop()) {
//					robot.updateState();
//					robot.getSonars(sonars);
//					soc.updateSonars(sonars);
//					bac.updateBearing(wc.getLVel(), wc.getAVel());
//					speed = soc.getForwardSonarReading() - .5;
//					target = (speed > .5) ? .5 : ((Math.abs(speed)<=.03)? 0 : speed);
//					for(int i =0 ; i < 16; i ++){
//					 	outRawSonar.print(sonars[i]+",");
//					 	outRawSonar.println(";");
//					}
//					for(int i =0 ; i < 16; i ++){
//					 	outFiltSonar.print(soc.getSonarReadings()[i]+",");
//					 	outFiltSonar.println(";");
//					}
//					wc.setALVel(0, target);
//					wc.updateWheels(robot, bc.isBumped(robot));
//					sc.setSonars(soc.getSonarReadings());
//					sc2.setSonars(sonars);
//					try {
//						Thread.sleep(50);
//					} catch(InterruptedException iex) {
//						System.out.println("Sonar sleep interrupted");
//					}
//				}
//				outRawSonar.close();
//				outFiltSonar.close();
//			} catch (IOException e){
//				e.printStackTrace();
//			}
//			robot.turnSonarsOff();
//			hideSC();
//		}
//
//		public String toString() {
//			return "sonar loop";
//		}
//	}
//
//	class TrackTask extends Task {
//
//		TrackTask(TaskController tc) {
//			super(tc);
//			wc = new WheelController();
//			soc = new SonarController();
//			bc = new BumperController();
//			trc = new TrackerController();
//		}
//		int counter = 25, a = 1;
//		public void taskRun() {
//			showSC();
//			robot.turnSonarsOn();
//			wc.setALVel(0, 0);
//			double[] sonars = new double[16];
//			double direction = -1;
//			try{
//				FileWriter outFileRawSonar = new FileWriter("TrackRawSonarData");
//				FileWriter outFileFiltSonar = new FileWriter("TrackFiltSonarData");
//				FileWriter outFileFollowTracker = new FileWriter("TrackFollowData");
//				PrintWriter outRawSonar = new PrintWriter(outFileRawSonar);
//				PrintWriter outFiltSonar = new PrintWriter(outFileFiltSonar);
//				PrintWriter outFollowTracker = new PrintWriter(outFileFollowTracker);
//				while(!shouldStop()) {
//					robot.updateState();
//					robot.getSonars(sonars);
//					soc.updateSonars(sonars);
//					for(int i =0 ; i < 16; i ++){
//					 	outRawSonar.print(sonars[i]+",");
//					 	outRawSonar.println(";");
//					}
//					for(int i =0 ; i < 16; i ++){
//					 	outFiltSonar.print(soc.getSonarReadings()[i]+",");
//					 	outFiltSonar.println(";");
//					}
//					sc.setSonars(soc.getSonarReadings());
//					sc2.setSonars(sonars);
////					trc.updateTracker(soc.getSonarReadings(), wc.getAVel());
////					outFollowTracker.println(trc.getFollowDirection(true)+","+trc.getFollowDistance(true)+";");
////					direction = trc.getFollowDirection(false) * 22.5 * Math.PI/180;
////					wc.pointToDirection(direction);
////					if (direction < Math.PI/2 || direction > 3*Math.PI/2){
////						wc.shadowPoint(trc.getFollowPoint(),trc.isFollowLost(), soc.getForwardSonarReading(), .5);
////					}
//////					wc.setCurv(2);
//////					wc.setLVel(.5);
////					System.out.println("Tracker Direction: " + trc.getFollowDirection(false));
////					System.out.println("Tracker distance: " + trc.getFollowDistance(false));
//					wc.updateWheels(robot, bc.isBumped(robot));
//					try {
//						Thread.sleep(25);
//					} catch(InterruptedException iex) {
//						System.out.println("\"Both\" sleep interrupted");
//					}
//				}
//				outRawSonar.close();
//				outFiltSonar.close();
//				outFollowTracker.close();
//			} catch (IOException e){
//				e.printStackTrace();
//			}
//
//
//			robot.turnSonarsOff();
//			robot.setVel(0.0f, 0.0f);
//			hideSC();
//		}
//
//		public String toString() {
//			return "\"both\"";
//		}
//	}
//
//	private static final long serialVersionUID = 0;
//}
//
