/*
 * Sample code for the 16x62 class taught at the Robotics Institute
 * of Carnegie Mellon University
 *
 */
package edu.cmu.ri.mrpl.lab;

import java.io.*;
import java.util.ArrayList;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;

import edu.cmu.ri.mrpl.*;
import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.kinematics2D.Angle;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import edu.cmu.ri.mrpl.maze.MazeGraphics;

import edu.cmu.ri.mrpl.control.BearingController;
import edu.cmu.ri.mrpl.control.BehaviorController;
import edu.cmu.ri.mrpl.control.BumperController;
import edu.cmu.ri.mrpl.control.Convert;
import edu.cmu.ri.mrpl.control.PathFinder;
import edu.cmu.ri.mrpl.control.SonarController;
import edu.cmu.ri.mrpl.control.TrackerController;
import edu.cmu.ri.mrpl.control.VisualizeController;
import edu.cmu.ri.mrpl.control.WheelController;
import edu.cmu.ri.mrpl.gui.PointsConsole;
import edu.cmu.ri.mrpl.maze.MazeGraphicsSwing;
import edu.cmu.ri.mrpl.maze.MazeViewer;
import quicktime.streaming.SourcerTimingParams;

/**
 * A class that 
 * @author furlong
 *
 */
public class Lab5 extends JFrame implements ActionListener, TaskController {

	private Robot robot;
	private SonarConsole sc1;
	private SonarConsole sc2;

	private PointsConsole pc; // Added for displaying points. PMF
    private VisualizeController.MazeViewer mv;
    private JFrame scFrame;
	private JFrame sc2Frame;

	private JButton connectButton;
	private JButton disconnectButton;

	private JButton reactButton;
	private JButton sonarButton;
	private JButton bothButton;

	private JButton stopButton;
	private JButton quitButton;
	private JButton button3;
	private JButton button4;

	private JTextField textField1;
	private JTextField textField2;

	private WheelController wc;
	private SonarController soc;
	private BumperController bc;
	private TrackerController trc;
	private BearingController bac;
	private VisualizeController vc;
	private BehaviorController bvc;
	
	private ReactTask reactTask;
	private LocalizeTask sonarTask;
	private MapLocalizationTask bothTask;

	private Task curTask = null;
	
	static final int DEFAULT_ROOM_SIZE = 4;

	public Lab5() {
		super("Lab 5");

		connectButton = new JButton("connect");
		disconnectButton = new JButton("disconnect");

		reactButton = new JButton("Run Reactive Wander!");
		sonarButton = new JButton("Run Visualizer!");
		bothButton = new JButton("Run Stateful Wander!");
		stopButton = new JButton(">> stop <<");
		quitButton = new JButton(">> quit <<");

		button3 = new JButton("button3");
		button4 = new JButton("button4");
		textField1 = new JTextField("textField1");
		textField2 = new JTextField("textField2");

		connectButton.addActionListener(this);
		disconnectButton.addActionListener(this);

		reactButton.addActionListener(this);
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
		box.add(reactButton);
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
		scFrame = new JFrame("Live Sonar Console");
		scFrame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
		sc1 = new SonarConsole();
		scFrame.add(sc1);
		scFrame.pack();

		sc2Frame = new JFrame("Filter Sonar Console");
		sc2Frame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
		sc2 = new SonarConsole();
		sc2Frame.add(sc2);
		sc2Frame.pack();
		
		// PMF: Creating a new points console for displaying robot-relative points.
		// takes arguments( top-left-x, top-left-y, window-width,window-height)
		// next set the bounds of the viewport.
//		pc = new PointsConsole(768-640, 1024-640, 640, 640);
//		pc.setWorldViewport(-1.0 * DEFAULT_ROOM_SIZE, -1.0 * DEFAULT_ROOM_SIZE, 1.0 * DEFAULT_ROOM_SIZE - 0.5, 1.0 * DEFAULT_ROOM_SIZE - 0.5);
//		// PMF: End new code for Points console
//		mc = new MazeGraphics();
		
		// construct tasks
		reactTask = new ReactTask(this);
		sonarTask = new LocalizeTask(this);
		bothTask = new MapLocalizationTask(this);

		// PMF: Creating a frame to put the PointsConsole panel in. 
		// get the panel that 
//		Frame f = new Frame();
//		f.setTitle("Points Console");
//		f.add(pc.getPanel());
//		f.setSize(pc.getPanel().getSize());
//		f.setVisible(true);
		// PMF: end of displaying PointsConsole

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
				sc2Frame.setLocationByPlatform(true);
				sc2Frame.setVisible(true);
			}
		});
	}

	// invoked from non-GUI thread to hide/show SonarConsole
	public void hideSC() {
		SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				scFrame.setVisible(false);
				sc2Frame.setVisible(false);

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
		} else if ( source==reactButton ) {
			(new Thread(reactTask)).start();
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
				new Lab5();
			}
		});
	}


	// here we implement the different robot controllers
	// as Tasks
	//

	class ReactTask extends Task {

		ReactTask(TaskController tc) {
			super(tc);
//			pf = new PathFinder();
		}

		public void taskRun() {
			robot.turnSonarsOn();

			double[] sonars = new double[16];
			double near = 1;
			double avel = 0, lvel = 0, front, left, right,speed = .6, vision = 1.2;
			while(!shouldStop()) {
				robot.updateState();
				robot.getSonars(sonars);
				front = soc.getFrontReadings(sonars);
				left = soc.getLeftReadings(sonars);
				right = soc.getRightReadings(sonars);
//				pf.get(pf.size()-1);
				
				right = (right < vision)? right/vision*speed : speed;
				front = (front < vision /2)? speed-(speed*front/(vision/2)):0;
				front = (right < left) ? -front/2 : front/2;
				left = (left < vision)? left/vision*speed : speed;

				System.out.println("right: "+right+" left: "+left+" front: "+front);

				wc.setWheelVel(right + front, left-front);
				wc.updateWheels(robot, bc.isBumped(robot));
				

				try {
					Thread.sleep(50);
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
    //This function requires in.maze to have an init pose. If not, the task will fail horridly.
	class LocalizeTask extends Task {
		MazeGraphics mg;
		LocalizeTask(TaskController tc) {
			super(tc);
			wc = new WheelController();
			soc = new SonarController();
			bc = new BumperController();
			trc = new TrackerController("in.maze");
			vc = new VisualizeController();
			bac = new BearingController(trc.getMazeInit());
			mg = new MazeGraphics(trc.getMaze());
            mv = vc.new MazeViewer(mg);
		}

		public void taskRun() {
			double[] heading;
			showSC();
			robot.turnSonarsOn();
            robot.updateState();
            bac.setInitPose(Convert.getRobotPose(robot));
			ArrayList<MazeGraphics.ContRobot> robots = new ArrayList<MazeGraphics.ContRobot>(2);
			robots.add(null);
			robots.add(null);
			double[] sonars = new double[16];
			while(!shouldStop()) {
				robot.updateState();
				robot.getSonars(sonars);
				sc1.setSonars(sonars);
				soc.updateSonars(sonars);
				sc2.setSonars(soc.getSonarReadings());
				bac.updateMazePoseByBearing(Convert.getRobotPose(robot));
//				trc.addTrackersFromSonar(soc.getSonarReadings());
//				trc.updateTrackers(bac.getDeltaPose());
//				bac.updateMazePoseBySonar(trc.getMazeCorrection(bac.getMazePose()));
//				trc.updateMazeWalls(RealPose2D pose);

//				trc.addTrackersFromSonar(sonars, bac.getPose());
				
				robots.set(0, new MazeGraphics.ContRobot(Convert.meterToMazeUnit(bac.getMazePose()), Color.GREEN));
				robots.set(1, new MazeGraphics.ContRobot(Convert.meterToMazeUnit(
                        Convert.inverseMultiply(bac.getInitPose(), Convert.getRobotPose(robot))), Color.RED));
				
				heading = bac.getMazePoseInMazeCoordinates();
				System.out.println(heading[0]+","+heading[1]+","+heading[2]);
//                System.out.println(bac.getMazePose().getX()+","+
//                        bac.getMazePose().getY()+","+
//                        bac.getMazePose().getRotateTheta());
				mg.setContRobots(robots);
                mv.recreateMazeGraphics(mg);
//				mg.paint(g)
				
				wc.setALVel(.5,.2);
				wc.updateWheels(robot, bc.isBumped(robot));
				
				try {
					Thread.sleep(20);
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

	class MapLocalizationTask extends Task {
        MazeGraphics mg;
		MapLocalizationTask(TaskController tc) {
            super(tc);
            wc = new WheelController();
            soc = new SonarController();
            bc = new BumperController();
            trc = new TrackerController("in.maze");
            vc = new VisualizeController();
            bac = new BearingController(trc.getMazeInit());
            mg = new MazeGraphics(trc.getMaze());
            mv = vc.new MazeViewer(mg);
		}

		public void taskRun() {
            double[] heading;
            showSC();
			robot.turnSonarsOn();
            robot.updateState();
            bac.setInitPose(Convert.getRobotPose(robot));
            ArrayList<MazeGraphics.ContRobot> robots = new ArrayList<MazeGraphics.ContRobot>(2);
            robots.add(null);
            robots.add(null);
			double[] sonars = new double[16];
			try{
				FileWriter outFileRawSonar = new FileWriter("TrackRawSonarData");
				FileWriter outFileFiltSonar = new FileWriter("TrackFiltSonarData");
				FileWriter outFileFollowTracker = new FileWriter("TrackFollowData");
				PrintWriter outRawSonar = new PrintWriter(outFileRawSonar);
				PrintWriter outFiltSonar = new PrintWriter(outFileFiltSonar);
				PrintWriter outFollowTracker = new PrintWriter(outFileFollowTracker);
				double near = 1;
				double avel = 0, lvel = 0, front, left, right,speed = .6, vision = 1.2;
				while(!shouldStop()) {
                    robot.updateState();
                    robot.getSonars(sonars);
                    sc1.setSonars(sonars);
                    soc.updateSonars(sonars);
                    sc2.setSonars(soc.getSonarReadings());
                    bac.updateMazePoseByBearing(Convert.getRobotPose(robot));
					trc.addTrackersFromSonar(soc.getSonarReadings());
					trc.updateTrackers(bac.getPose());
					bac.updateMazePoseBySonar(trc.getMazeCorrection(bac.getMazePose()));
					trc.updateMazeWalls(bac.getMazePose());
                    System.out.println(bac.getMazePose().toString());

                    robots.set(0, new MazeGraphics.ContRobot(Convert.meterToMazeUnit(bac.getMazePose()), Color.GREEN));
                    robots.set(1, new MazeGraphics.ContRobot(Convert.meterToMazeUnit(
                            Convert.inverseMultiply(bac.getInitPose(), Convert.getRobotPose(robot))), Color.RED));

                    heading = bac.getMazePoseInMazeCoordinates();
//                    System.out.println(heading[0]+","+heading[1]+","+heading[2]);
//                System.out.println(bac.getMazePose().getX()+","+
//                        bac.getMazePose().getY()+","+
//                        bac.getMazePose().getRotateTheta());
                    mg.setContRobots(robots);
                    mv.recreateMazeGraphics(mg);

//                    bvc.updateBehavior(bac.getPose(), trc.getAllTrackerRPos(bac.getPose()));
					//System.out.println("bvc target: "+bvc.getTarget().x+" , "+bvc.getTarget().y+"\n");
				
				/*
				 * dumb wanderer stuff 
				 */
					front = soc.getFrontReadings(sonars);
					left = soc.getLeftReadings(sonars);
					right = soc.getRightReadings(sonars);
					right = (right < vision)? right/vision*speed : speed;
					front = (front < vision /2)? speed-(speed*front/(vision/2)):0;
					front = (right < left) ? -front/2 : front/2;
					left = (left < vision)? left/vision*speed : speed;
//					System.out.println("right: "+right+" left: "+left+" front: "+front);
					wc.setWheelVel(right + front, left-front);
					wc.updateWheels(robot, bc.isBumped(robot));
								
					try {
						Thread.sleep(100);
					} catch(InterruptedException iex) {
						System.out.println("\"Both\" sleep interrupted");
					}
				}
				outRawSonar.close();
				outFiltSonar.close();
				outFollowTracker.close();
			} catch (IOException e){
				e.printStackTrace();
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

