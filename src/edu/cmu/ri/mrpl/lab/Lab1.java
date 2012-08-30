package edu.cmu.ri.mrpl.lab;
/*

 * Sample code for the 16x62 class taught at the Robotics Institute
 * of Carnegie Mellon University
 *
 * written from scratch by Martin Stolle in 2006
 *
 * inspired by code started by Illah Nourbakhsh and used
 * for many years.
 */

import java.io.*;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;

import edu.cmu.ri.mrpl.*;
import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.control.WheelController;

public class Lab1 extends JFrame implements ActionListener, TaskController {

	private Robot robot;
	private SonarConsole sc;
	private JFrame scFrame;

	private JButton connectButton;
	private JButton disconnectButton;

	private JButton programButton;
	private JButton sonarButton;
	private JButton bothButton;

	private JButton stopButton;
	private JButton quitButton;
	private JButton button3;
	private JButton button4;

	private JTextField textField1;
	private JTextField textField2;
	
	private WheelController wheels;

	private RotateTask programTask;
	private SonarTask sonarTask;
	private BothTask bothTask;

	private Task curTask = null;

	public Lab1() {
		super("Kick Ass Sample Robot App");

		connectButton = new JButton("connect");
		disconnectButton = new JButton("disconnect");

		programButton = new JButton("Run Rotate!");
		sonarButton = new JButton("Run Sonar Loop!");
		bothButton = new JButton("Run both!");
		stopButton = new JButton(">> stop <<");
		quitButton = new JButton(">> quit <<");

		button3 = new JButton("button3");
		button4 = new JButton("button4");
		textField1 = new JTextField("textField1");
		textField2 = new JTextField("textField2");

		connectButton.addActionListener(this);
		disconnectButton.addActionListener(this);

		programButton.addActionListener(this);
		sonarButton.addActionListener(this);
		bothButton.addActionListener(this);
		stopButton.addActionListener(this);
		quitButton.addActionListener(this);

		button3.addActionListener(this);
		button4.addActionListener(this);

		Container main = getContentPane();
		main.setLayout(new BoxLayout(main, BoxLayout.Y_AXIS));
		Box box;

		main.add(Box.createVerticalStrut(30));

		box = Box.createHorizontalBox();
		main.add(box);
		box.add(Box.createHorizontalStrut(30));
		box.add(connectButton);
		box.add(Box.createHorizontalStrut(30));
		box.add(disconnectButton);
		box.add(Box.createHorizontalStrut(30));

		main.add(Box.createVerticalStrut(30));

		box = Box.createHorizontalBox();
		main.add(box);
		box.add(Box.createHorizontalStrut(30));
		box.add(programButton);
		box.add(Box.createHorizontalStrut(30));
		box.add(sonarButton);
		box.add(Box.createHorizontalStrut(30));

		main.add(Box.createVerticalStrut(30));

		box = Box.createHorizontalBox();
		main.add(box);
		box.add(bothButton);

		main.add(Box.createVerticalStrut(30));

		box = Box.createHorizontalBox();
		main.add(box);
		box.add(Box.createHorizontalStrut(30));
		box.add(button3);
		box.add(Box.createHorizontalStrut(30));
		box.add(textField1);
		box.add(Box.createHorizontalStrut(30));

		main.add(Box.createVerticalStrut(30));

		box = Box.createHorizontalBox();
		main.add(box);
		box.add(Box.createHorizontalStrut(30));
		box.add(button4);
		box.add(Box.createHorizontalStrut(30));
		box.add(textField2);
		box.add(Box.createHorizontalStrut(30));

		main.add(Box.createVerticalStrut(30));

		box = Box.createHorizontalBox();
		main.add(box);
		box.add(stopButton);
		box.add(Box.createHorizontalStrut(30));
		box.add(quitButton);

		main.add(Box.createVerticalStrut(30));


		this.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
		this.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) {
				quit();
			}
		});


		this.setLocationByPlatform(true);
		this.pack();
		this.setVisible(true);

		// construct the SonarConsole, but don't make it visible
		scFrame = new JFrame("Sonar Console");
		scFrame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
		sc = new SonarConsole();
		scFrame.add(sc);
		scFrame.pack();


		// construct tasks
		programTask = new RotateTask(this);
		sonarTask = new SonarTask(this);
		bothTask = new BothTask(this);



		robot=new SimRobot();
		//robot=new ScoutRobot();
	}

	// call from GUI thread
	public void connect() {
		try {
			robot.connect();
		} catch(IOException ioex) {
			System.err.println("couldn't connect to robot:");
			ioex.printStackTrace();
		}
	}

	// call from GUI thread
	public void disconnect() {
		try {
			robot.disconnect();
		} catch(IOException ioex) {
			System.err.println("couldn't disconnect from robot:");
			ioex.printStackTrace();
		}
	}


	// call from GUI thread
	public synchronized void stop() {
		if (curTask!=null)
			curTask.pleaseStop();
	}

	// call from GUI thread
	public void quit() {

		synchronized(this) {
			if (curTask!=null) {
				curTask.pleaseStop();

				try {
					this.wait(1000);
				} catch(InterruptedException iex) {
					System.err.println("interrupted while waiting for worker termination signal");
				}
			}

			if (curTask!=null) {
				System.err.println("exiting even though robot is running!");
			} else {
				disconnect();
			}
		}

		System.exit(0);
	}

	// invoked from non-GUI thread to hide/show SonarConsole
	public void showSC() {
		SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				scFrame.setLocationByPlatform(true);
				scFrame.setVisible(true);
			}
		});
	}

	// invoked from non-GUI thread to hide/show SonarConsole
	public void hideSC() {
		SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				scFrame.setVisible(false);
			}
		});
	}

	public void actionPerformed(ActionEvent e) {
		Object source = e.getSource();
		if (source==connectButton) {
			connect();
		} else if ( source==disconnectButton ) {
			disconnect();
		} else if ( source==stopButton ) {
			stop();
		} else if ( source==quitButton ) {
			quit();
		} else if ( source==programButton ) {
			(new Thread(programTask)).start();
		} else if ( source==sonarButton ) {
			(new Thread(sonarTask)).start();
		} else if ( source==bothButton ) {
			(new Thread(bothTask)).start();
		}
	}

	public synchronized boolean canStart(Task t) {
		if (curTask!=null)
			return false;

		curTask = t;
		return true;
	}

	public synchronized void finished(Task t) {
		if (curTask!=t) {
			System.err.println("ignoring finished() from unknown task "+t+"!");
			return;
		}
		curTask=null;
		this.notifyAll();
	}

	public static void main(String[] argv) {
		javax.swing.SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				new Lab1();
			}
		});
	}


	// here we implement the different robot controllers
	// as Tasks
	//

	class RotateTask extends Task {
		
		RotateTask(TaskController tc) {
			super(tc);
			wheels = new WheelController();
		}

		public void taskRun() {
			robot.turnSonarsOn();

			double[] sonars = new double[16];
			while(!shouldStop()) {
				robot.updateState();
				robot.getSonars(sonars);

//				double frontSonar = sonars[0];
//				double backSonar = sonars[8];
//
//				System.out.println("Front sonar: " + frontSonar + 
//						"  Back sonar: " + backSonar);
//
//				if (robot.getBumpers()!=0) {
//					System.err.println("detecting bumping, stopping!");
//					robot.turnSonarsOff();
//					robot.setVel(0,0);
//					break;
//				}
//
//				if (frontSonar < backSonar) {
//					robot.setVel(-0.05, -0.05);
//				} else {
//					robot.setVel(0.05, 0.05);
//				}
				
				wheels.setAVel(.1);//Debug
				wheels.setLVel(.05);
				wheels.updateWheels(robot);

				try {
					Thread.sleep(500);
				} catch(InterruptedException iex) {
					System.out.println("sample program sleep interrupted");
				}
			}
			robot.turnSonarsOff();
			robot.setVel(0,0);
		}

		public String toString() {
			return "sample program";
		}
	}

	class SonarTask extends Task {

		SonarTask(TaskController tc) {
			super(tc);
		}

		public void taskRun() {
			showSC();
			robot.turnSonarsOn();

			double[] sonars = new double[16];
			while(!shouldStop()) {
				robot.updateState();
				robot.getSonars(sonars);
				sc.setSonars(sonars);

				try {
					Thread.sleep(500);
				} catch(InterruptedException iex) {
					System.out.println("Sonar sleep interrupted");
				}
			}

			robot.turnSonarsOff();
			hideSC();
		}

		public String toString() {
			return "sonar loop";
		}
	}

	class BothTask extends Task {

		BothTask(TaskController tc) {
			super(tc);
		}

		public void taskRun() {
			showSC();
			robot.turnSonarsOn();

			double[] sonars = new double[16];
			while(!shouldStop()) {
				robot.updateState();
				robot.getSonars(sonars);
				sc.setSonars(sonars);

				double frontSonar = sonars[0];
				double backSonar = sonars[8];

				System.out.println("Front sonar: " + frontSonar + 
						"  Back sonar: " + backSonar);

				if (robot.getBumpers()!=0) {
					System.err.println("detecting bumping, stopping!");
					break;
				}

				if (frontSonar < backSonar) {
					robot.setVel(-0.05, -0.05);
				} else {
					robot.setVel(0.05, 0.05);
				}

				try {
					Thread.sleep(500);
				} catch(InterruptedException iex) {
					System.out.println("\"Both\" sleep interrupted");
				}
			}

			robot.turnSonarsOff();
			robot.setVel(0.0f, 0.0f);
			hideSC();
		}

		public String toString() {
			return "\"both\"";
		}
	}
	
	private static final long serialVersionUID = 0;
}

