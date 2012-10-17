package edu.cmu.ri.mrpl.control;

import java.awt.*;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.*;
import java.util.ArrayList;

import edu.cmu.ri.mrpl.Command;
import edu.cmu.ri.mrpl.CommandSequence;
import edu.cmu.ri.mrpl.Path;
import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.gui.PointsConsole;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import edu.cmu.ri.mrpl.maze.MazeGraphics;
import edu.cmu.ri.mrpl.maze.MazeState;


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
    MotionPlanController mpc;
	
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
    PrintWriter outTrackMaze;
    PrintWriter outTrackRob;
    PrintWriter outTrackFMaze;
    PrintWriter outTrackFRob;

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

        trc = new TrackerController(Convert.getRobotPose(robot) ,"in2.maze");
        bac = new BearingController(trc.getMazeInit(), Convert.getRobotPose(r));
        bhc = new BehaviorController();
        mpc = new MotionPlanController(trc.getMaze());

        if (true) {
            Path executePath = mpc.searchForPath(trc.getMazeInit());

            Command.PathArgument pArg = new Command.PathArgument(executePath);
            addCommand(new Command(Command.Type.FOLLOWPATH, pArg));
        }



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
                FileWriter outFileTrackMaze = new FileWriter("TrackOutMaze.txt");
                FileWriter outFileTrackRob = new FileWriter("TrackOutRob.txt");
                FileWriter outFileTrackFMaze = new FileWriter("TrackOutFMaze.txt");
                FileWriter outFileTrackFRob = new FileWriter("TrackOutFRob.txt");

                outRobo = new PrintWriter(outFileRobo);
                outRoboEncode = new PrintWriter(outFileRoboE);
                outRoboMaze = new PrintWriter(outFileRoboM);
                outFollowTracker = new PrintWriter(outFileFollowTracker);
                outTrackMaze = new PrintWriter(outFileTrackMaze);
                outTrackRob = new PrintWriter(outFileTrackRob);
                outTrackFMaze = new PrintWriter(outFileTrackFMaze);
                outTrackFRob = new PrintWriter(outFileTrackFRob);
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
        trc.updateTrackers(bac.getMazePose());

        if (useVisualization){
            ArrayList<MazeGraphics.ContRobot> robots = new ArrayList<MazeGraphics.ContRobot>(2);
            robots.add(null);
            robots.add(null);
            vc.updateRobotPos(pointsConsole, bac.getMazePose());
            if (lastDistance == 0) {
                if (DEBUGFLAG) {
                    outTrackMaze.print("Gradient input: {");
                    for (RealPoint2D p : trc.getAllTrackerWPos(bac.getMazePose())){
                        outTrackMaze.print("{"+p.getX()+","+p.getY()+"},");
                    }
                    outTrackMaze.println("}");
                    outTrackRob.print("Gradient input: {");
                    for (RealPoint2D p : trc.getAllTrackerWPos(Convert.getRobotPose(robot))){
                        outTrackRob.print("{"+p.getX()+","+p.getY()+"},");
                    }
                    outTrackRob.println("}");
                }
                bac.updateMazePoseBySonar(trc.getMazeCorrection(bac.getMazePose()));
                if (DEBUGFLAG) {
                    outTrackFMaze.print("Gradient input: {");
                    for (RealPoint2D p : trc.getFilteredTrackerWPos(bac.getMazePose())){
                        outTrackFMaze.print("{"+p.getX()+","+p.getY()+"},");
                    }
                    outTrackFMaze.println("}");
                    outTrackFRob.print("Gradient input: {");
                    for (RealPoint2D p : trc.getFilteredTrackerWPos(Convert.getRobotPose(robot))){
                        outTrackFRob.print("{"+p.getX()+","+p.getY()+"},");
                    }
                    outTrackFRob.println("}");
                }
                vc.addPoints(pointsConsole, trc.getFilteredTrackerRPos());
            }
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
            outTrackMaze.print("{");
            for (RealPoint2D p : trc.getAllTrackerWPos(bac.getMazePose())){
                outTrackMaze.print("{"+p.getX()+","+p.getY()+"},");
            }
            outTrackMaze.println("}");
            outTrackRob.print("{");
            for (RealPoint2D p : trc.getAllTrackerWPos(Convert.getRobotPose(robot))){
                outTrackRob.print("{"+p.getX()+","+p.getY()+"},");
            }
            outTrackRob.println("}");
        }

	}
	/**
	 * Creates new execution thread to complete pending command.
	 */
	public synchronized void execute(){
//		System.out.println("Execute method" + active.type);
		if (!exe.t.isAlive()){
			getNext();
			exe = new ExecuteTask(this, robot, active, bac.getMazePose());
		}
	}
	public synchronized void haltThread() {
		System.out.println("Halting...");
        if (DEBUGFLAG) {
            outRobo.close();
            outRoboEncode.close();
            outRoboMaze.close();
            outFollowTracker.close();
            outTrackMaze.close();
            outTrackRob.close();
            outTrackFMaze.close();
            outTrackFRob.close();
        }
		exe.halt();
		System.out.println("Halt sent");
	}
	
	
}
