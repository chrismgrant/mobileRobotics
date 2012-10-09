package edu.cmu.ri.mrpl.control;

import java.awt.*;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.*;
import java.util.ArrayList;

import edu.cmu.ri.mrpl.Command;
import edu.cmu.ri.mrpl.CommandSequence;
import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.gui.PointsConsole;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.maze.MazeGraphics;


/**
 * Handles command sequencing
 * @author WangHeli
 *
 */
public class CommandController {
	WheelController wc;
	SonarController soc;
	BumperController bc;
	
	TrackerController trc;
	BearingController bac;
	BehaviorController bhc;
	
	VisualizeController vc;
    VisualizeController.MazeViewer mv;
	
	Robot robot;
    PointsConsole pointsConsole;

    private static final boolean DEBUGFLAG = true;
	private ExecuteTask exe;
	private final Command nullCommand = new Command();
	private Command active;
	private CommandSequence executeQueue;
    private boolean useVisualization;

    boolean debugFlag;
    private MazeGraphics mg;

    PrintWriter outRobo;
    PrintWriter outRoboEncode;
    PrintWriter outRoboMaze;
    PrintWriter outFollowTracker;

    /**
	 * Initializes a new CommandController
	 */
    public CommandController(Robot r, PointsConsole pc) {
        debugFlag = false;
        executeQueue = new CommandSequence();
        active = new Command();
        robot = r;

        wc = new WheelController();
        soc = new SonarController();
        bc = new BumperController();

        trc = new TrackerController(Convert.getRobotPose(robot) ,"in.maze");
        bac = new BearingController(trc.getMazeInit(), Convert.getRobotPose(r));
        bhc = new BehaviorController();

        robot = r;

        useVisualization = (pc == null)?false:true;
        if (useVisualization){
            vc = new VisualizeController();
            mg = new MazeGraphics(trc.getMaze());
            mv = vc.new MazeViewer(mg);
            pointsConsole = pc;
        }

        if (DEBUGFLAG) {
            try {
                FileWriter outFileRobo = new FileWriter("TrackRobo");
                FileWriter outFileRoboE = new FileWriter("TrackRoboEn");
                FileWriter outFileRoboM = new FileWriter("TrackRoboMa");
                FileWriter outFileFollowTracker = new FileWriter("TrackFollowData");

                outRobo = new PrintWriter(outFileRobo);
                outRoboEncode = new PrintWriter(outFileRoboE);
                outRoboMaze = new PrintWriter(outFileRoboM);
                outFollowTracker = new PrintWriter(outFileFollowTracker);
            } catch (IOException e) {}
        }
        exe = new ExecuteTask(this, robot, nullCommand, bac.getPose());

    }
	public CommandController(Robot r){
        this(r,null);
	}

	/**
	 * Gets filtered sonar readings for sonar console
	 * @return array of sonar readings
	 */
	public double[] getSonars(){
		return soc.getSonarReadings();
	}
	/**
	 * Gets next command in execution stack, or returns null command if empty
	 * @return Next command to execute
	 */
	Command getNext(){
		bhc.clearIntegrals();
		if (executeQueue.isEmpty()){
			active = nullCommand;
			return active;
		} else {
			active = executeQueue.remove(0);
			return active;
		}
	}
	/**
	 * Gets current command
	 * @return Active command
	 */
	Command.Type getActiveCommandType(){
		return active.type;
	}
	/**
	 * Gets the argument of the current command
	 * @return Argument for current command
	 */
	Command.Argument getActiveCommandArg(){
		return active.argument;
	}
	/**
	 * Enqueues command to executeQueue
	 * @param c New command to add
	 */
	public synchronized void addCommand(Command c){
		executeQueue.add(c);
	}
	public synchronized void pushCommand(Command c){
		executeQueue.add(0, c);
	}
	public synchronized void addCommandFromFile(java.lang.String inputFile){
		try {
			executeQueue.readFile(inputFile);
			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	/**
	 * Updates all controllers.
	 * To be called before accessing controller information
	 * @param sonars array of raw sonar values
	 */
	public synchronized void updateControllers(double[] sonars){
        double lastDistance = 0;
		soc.updateSonars(sonars);
		wc.updateWheels(robot, bc.isBumped(robot));

        lastDistance = bac.updateMazePoseByBearing(Convert.getRobotPose(robot));
        trc.addTrackersFromSonar(lastDistance, soc.getSonarReadings());
        trc.updateTrackers(bac.getDeltaPose());

        if (useVisualization){
            ArrayList<MazeGraphics.ContRobot> robots = new ArrayList<MazeGraphics.ContRobot>(2);
            robots.add(null);
            robots.add(null);
            if (lastDistance == 0) {
                bac.updateMazePoseBySonar(trc.getMazeCorrection(bac.getMazePose()));
            }
            vc.updateRobotPos(pointsConsole, bac.getMazePose());
            vc.addPoints(pointsConsole, trc.getFilteredTrackerRPos());
            vc.updateVisualizer(pointsConsole, robot);

            robots.set(0, new MazeGraphics.ContRobot(Convert.meterToMazeUnit(bac.getMazePose()), Color.GREEN));
            robots.set(1, new MazeGraphics.ContRobot(Convert.meterToMazeUnit(
                    Convert.inverseMultiply(bac.getInitMazePose(), Convert.getRobotPose(robot))), Color.RED));
            mg.setContRobots(robots);
            mv.recreateMazeGraphics(mg);
        }

        if (DEBUGFLAG) {
            for (RealPoint2D p : trc.getAllTrackerRPos(BearingController.getRPose(robot))) {
                outFollowTracker.print(p.toString()+";");
            }
            outFollowTracker.println();
            outRobo.println(BearingController.getRPose(robot).toString());
            outRoboEncode.println(Convert.inverseMultiply(bac.getInitMazePose(), Convert.getRobotPose(robot)).toString());
            outRoboMaze.println(bac.getMazePose().toString());
        }

	}
	/**
	 * Creates new execution thread to complete pending command.
	 */
	public synchronized void execute(){
//		System.out.println("Execute method" + active.type);
		if (!exe.t.isAlive()){
			getNext();
			exe = new ExecuteTask(this, robot, active, bac.getRPoseWithError(robot));
		}
	}
	public synchronized void haltThread() {
		System.out.println("Halting...");
		exe.halt();
		System.out.println("Halt sent");
	}
	
	
}
