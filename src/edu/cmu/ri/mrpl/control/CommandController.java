package edu.cmu.ri.mrpl.control;

import java.awt.*;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.*;
import java.util.ArrayList;

import edu.cmu.ri.mrpl.*;
import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.game.GameClient;
import edu.cmu.ri.mrpl.gui.PointsConsole;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.maze.MazeGraphics;
import edu.cmu.ri.mrpl.maze.MazePos;
import edu.cmu.ri.mrpl.maze.MazeState;
import edu.cmu.ri.mrpl.maze.MazeWorld;


/**
 * Handles command sequencing
 * @author WangHeli
 *
 */
public class CommandController {
	WheelController wc;
	SonarController soc;
	BumperController bc;
    CameraController cac;
	
	TrackerController trc;
	BearingController bac;
	BehaviorController bhc;
    MotionPlanController mpc;
    CommClientController cmc;
    GameClient gc;


    VisualizeController vc;
    VisualizeController.MazeViewer mv;
	
	Robot robot;
    PointsConsole pointsConsole;

    //Debug Flag prints out all debug detail
    static final boolean DEBUG_FLAG = true;
    //Path Searching enables continuous searching between robot pose and goal states
    private static final boolean PATH_SEARCH_FLAG = true;
    //Game Flag enables updating of goal states with respect to game
    private static final boolean GAME_FLAG = true;

	private ExecuteTask exe;
	private final Command nullCommand = new Command();
	private Command active;
	private CommandSequence executeQueue;
    private boolean useVisualization;
    private boolean gameActive;
    boolean holdingGold;

    private MazeGraphics mg;

    PrintWriter outRobo;
    PrintWriter outRoboEncode;
    PrintWriter outRoboMaze;
    PrintWriter outFollowTracker;
    PrintWriter outTrackMaze;
    PrintWriter outTrackRob;
    PrintWriter outTrackFMaze;
    PrintWriter outTrackFRob;
    PrintWriter outTrackWMaze;

    /**
	 * Initializes a new CommandController
	 */
    public CommandController(PointsConsole pc) {
        executeQueue = new CommandSequence();
        active = new Command();
        gameActive = true;
        holdingGold = false;

        wc = new WheelController();
        soc = new SonarController();
        bc = new BumperController();
        bhc = new BehaviorController();
        useVisualization = (pc == null)?false:true;
        pointsConsole = pc;
        cmc = new CommClientController();
        gc = new GameClient();


        exe = new ExecuteTask(this);


        if (DEBUG_FLAG) {
            try {
                FileWriter outFileRobo = new FileWriter("TrackRobo");
                FileWriter outFileRoboE = new FileWriter("TrackRoboEn");
                FileWriter outFileRoboM = new FileWriter("TrackRoboMa");
                FileWriter outFileFollowTracker = new FileWriter("TrackFollowData");
                FileWriter outFileTrackMaze = new FileWriter("TrackOutMaze.txt");
                FileWriter outFileTrackRob = new FileWriter("TrackOutRob.txt");
                FileWriter outFileTrackFMaze = new FileWriter("TrackOutFMaze.txt");
                FileWriter outFileTrackFRob = new FileWriter("TrackOutFRob.txt");
                FileWriter outFileTrackWMaze = new FileWriter("TrackOutWMaze.txt");

                outRobo = new PrintWriter(outFileRobo);
                outRoboEncode = new PrintWriter(outFileRoboE);
                outRoboMaze = new PrintWriter(outFileRoboM);
                outFollowTracker = new PrintWriter(outFileFollowTracker);
                outTrackMaze = new PrintWriter(outFileTrackMaze);
                outTrackRob = new PrintWriter(outFileTrackRob);
                outTrackFMaze = new PrintWriter(outFileTrackFMaze);
                outTrackFRob = new PrintWriter(outFileTrackFRob);
                outTrackWMaze = new PrintWriter(outFileTrackWMaze);
            } catch (IOException e) {

            }
        }

    }

	public CommandController(){
        this(null);
	}

    public void playSound(String id){
        exe.speak(id);
    }
    public void initCam() {
        cac = new CameraController();
    }
    public void initRobot(Robot r) {
        robot = r;
        exe.setRobot(r);
    }
    public void initSonars(){
    }

    public void initComm(String robotID) {
        cmc.connect();
//        gc.connectToServer("Combat Styrofoam", robotID, robotID + "other");
    }
    public void initMaze(String mazeFile) {
        trc = new TrackerController(Convert.getRobotPose(robot), mazeFile);
        bac = new BearingController(trc.getMazeInit(), Convert.getRobotPose(robot));
        mpc = new MotionPlanController(trc.getMaze());
        exe.setupTask(nullCommand, bac.getMazePose());
        exe.initPose();

        if (robot instanceof SimRobot) {
            ((SimRobot) robot).setPosition(bac.getInitMazePose().getX(),bac.getInitMazePose().getY(),
                    bac.getInitMazePose().getRotateTheta());
            bac.setInitMazePose(trc.getMazeInit(),Convert.getRobotPose(robot));
        }

        if (DEBUG_FLAG) {
            System.out.printf("Robot position: %s\nRobot maze position: %s\n",
                    Convert.getRobotPose(robot).toString(),bac.getMazePose());
        }
        if (useVisualization){
            vc = new VisualizeController();
            mg = new MazeGraphics(trc.getMaze());
            mv = vc.new MazeViewer(mg);
        }
        if (GAME_FLAG) {
            trc.targetGold();
        }
        if (PATH_SEARCH_FLAG) {
            //Search
            searchNextPath(trc.getMazeInit());

        }
    }


	/**
	 * Gets filtered sonar readings for sonar console
	 * @return array of sonar readings
	 */
	public double[] getSonars(){
		return soc.getSonarReadings();
	}

    /**
     * Searches the mazeWorld for closest goal, and adds commands to executeQueue.
     * @return first Command added to executeQueue, nullCommand if no path found
     */
    Command searchNextPath(MazeState searchInitState) {
        if (DEBUG_FLAG) {
            System.out.print("Recalculating path to goal.\n");
        }
        parseMessages();
        Path executePath = mpc.searchForPath(searchInitState);
        cmc.sendMsg(mpc.getClaimedTarget());
        if (executePath.size() > 1) {
            Command.PathArgument pArg = new Command.PathArgument(executePath);
            Command.AngleArgument aArg = new Command.AngleArgument(
                    Convert.WRTRobot(bac.getMazePose(),executePath.get(0)).getTh());

            Command first = new Command(Command.Type.TURNTO, aArg, false);
            clearCommands();
            active = first;
            addCommand(new Command(Command.Type.FOLLOWPATH, pArg));
            if (GAME_FLAG) {
                if (holdingGold) {
                    addCommand(new Command(Command.Type.DROPGOLD));
                } else {
                    addCommand(new Command(Command.Type.PICKGOLD));
                }
            }
            return first;
        } else {
            return nullCommand;
        }
    }
	/**
	 * Gets next command in execution stack, or returns null command if empty
	 * @return Next command to execute
	 */
	Command getNext(){
		bhc.clearIntegrals();
		if (executeQueue.isEmpty()){
            if (gameActive) {
                active = searchNextPath(Convert.RealPoseToMazeState(bac.getMazePose()));
            } else {
                active = nullCommand;
            }
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
    synchronized void clearCommands(){
        executeQueue.clear();
    }
	/**
	 * Enqueues command to executeQueue
	 * @param c New command to add
	 */
	public synchronized void addCommand(Command c){
		executeQueue.add(c);
	}
	synchronized void pushCommand(Command c){
		executeQueue.add(0, c);
	}
	public synchronized void addCommandFromFile(java.lang.String inputFile){
		try {
			executeQueue.readFile(inputFile);
			
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	/**
	 * Updates all controllers.
	 * To be called before accessing controller information
	 * @param sonars array of raw sonar values
	 */
	synchronized void updateControllers(double[] sonars){
        double lastDistance = 0;
        boolean wallChanged;
		soc.updateSonars(sonars);
		wc.updateWheels(robot, bc.isBumped(robot));

        lastDistance = bac.updateMazePoseByBearing(Convert.getRobotPose(robot));
        trc.addTrackersFromSonar(bac.getMazePose(),lastDistance, soc.getSonarReadings());
        wallChanged = trc.updateTrackers(bac.getMazePose());
        cmc.sendMsg(bac.getMazePose().getPosition(),mpc.getPathList());
        if (useVisualization){
            ArrayList<MazeGraphics.ContRobot> robots = new ArrayList<MazeGraphics.ContRobot>(2);
            robots.add(null);
            robots.add(null);
            vc.updateRobotPos(pointsConsole, bac.getMazePose());
            if (lastDistance == 0) {
                if (DEBUG_FLAG) {
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
                if (DEBUG_FLAG) {
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
            }


            robots.set(0, new MazeGraphics.ContRobot(Convert.meterToMazeUnit(bac.getMazePose()), Color.GREEN));
            robots.set(1, new MazeGraphics.ContRobot(Convert.meterToMazeUnit(Convert.multiply(bac.getInitPose(),
                    Convert.inverseMultiply(bac.getInitMazePose(), Convert.getRobotPose(robot)))), Color.RED));
            mg.setContRobots(robots);
            mv.recreateMazeGraphics(mg);

//            if (wallChanged && PATH_SEARCH_FLAG) {
//                // Stop execution and re-search path.
//                exe.stop();
//                Toolkit.getDefaultToolkit().beep();
//                exe.speak("Researching...");
//                exe.setupTask(searchNextPath(Convert.RealPoseToMazeState(bac.getMazePose())), bac.getMazePose());
//                System.out.printf("%s\n",executeQueue.toString());
//            }
            vc.addPoints(pointsConsole, trc.getNewTrackerRPos());

            vc.updateVisualizer(pointsConsole, robot);

        }
        // If gold drops in transit, go to next gold
        //TODO add counter to stagger gold check
//        if (holdingGold && !cac.holdingGold()) {
//            holdingGold = false;
//            exe.stop();
//            Toolkit.getDefaultToolkit().beep();
//            exe.speak("Dropped gold...");
//            trc.targetGold();
//            exe.setupTask(searchNextPath(Convert.RealPoseToMazeState(bac.getMazePose())), bac.getMazePose());
//            System.out.printf("%s\n",executeQueue.toString());
//        }

        if (DEBUG_FLAG) {
            for (RealPoint2D p : trc.getAllTrackerRPos()) {
                outFollowTracker.print(p.toString()+";");
            }
            outFollowTracker.println();
            outRobo.println(BearingController.getRPose(robot).toString());
            outRoboEncode.println(Convert.inverseMultiply(bac.getInitMazePose(),
                    Convert.getRobotPose(robot)).toString());
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
        parseMessages();
	}

    private void parseMessages() {
        String message;
        String[] commands, args;
        try {
            do{
                message = cmc.comm.getIncomingMessage();
                if (DEBUG_FLAG) {
                    System.out.println("Incoming: \""+message+"\"");
                }
                if (message != null) {
                    commands = message.replace("\n","").replace("\r","").split(";");
                    for (String command : commands) {
                        args = command.split(" ");
                        if (args[0] == "Loc:") {
                            RealPoint2D partnerLoc = new RealPoint2D(Double.parseDouble(args[1]),
                                    Double.parseDouble(args[2]));
                            trc.parseRobotPose(partnerLoc);
                        }
                        if (args[0] == "Path:") {
                            ArrayList<MazePos> otherPath = new ArrayList<MazePos>();
                            for (int i = 1; i < args.length; i++){
                                String[] loc = args[i].split(",");
                                otherPath.add(new MazePos(Integer.valueOf(loc[0]),
                                        Integer.valueOf(loc[1])));
                            }
                            System.out.printf("Path blacklist: %s\n",otherPath.toString());
                            mpc.setBlockedList(otherPath);
                            if (mpc.pathCollide()) {
                                exe.setupTask(searchNextPath(Convert.RealPoseToMazeState(bac.getMazePose())), bac.getMazePose());
//                System.out.printf("%s\n",executeQueue.toString());
                            }
                            if (active.type == Command.Type.NULL) {

                            }
                        }
                        if (args[0] == "Rsc:") {
                            switch (Integer.valueOf(args[3])) {
                                case 0: {
                                    trc.removeRsc(new MazeState(Integer.valueOf(args[1]),
                                            Integer.valueOf(args[2]),
                                            MazeWorld.Direction.East));
                                    break;
                                }
                                case 1: {
                                    trc.removeRsc(new MazeState(Integer.valueOf(args[1]),
                                            Integer.valueOf(args[2]),
                                            MazeWorld.Direction.North));
                                    break;
                                }
                                case 2: {
                                    trc.removeRsc(new MazeState(Integer.valueOf(args[1]),
                                            Integer.valueOf(args[2]),
                                            MazeWorld.Direction.West));
                                    break;
                                }
                                case 3: {
                                    trc.removeRsc(new MazeState(Integer.valueOf(args[1]),
                                            Integer.valueOf(args[2]),
                                            MazeWorld.Direction.South));
                                    break;
                                }
                            }
                        }

                    }
                }
            } while (message != null);
        } catch (CommClient.CommException e) {
            System.err.println("Comm Exception: " + e.getMessage());

        }
    }

    /**
	 * Creates new execution thread to complete pending command.
	 */
	public synchronized void step(){
//		System.out.println("Execute method" + active.type);
        boolean statusComplete = exe.step();
        if (statusComplete) {
            trc.removeGoal(Convert.RealPoseToMazeState(bac.getMazePose()));
            if (GAME_FLAG) {
//                holdingGold = cac.holdingGold();
                if (holdingGold) {
                    trc.removeGold(Convert.RealPoseToMazeState(bac.getMazePose()));
                    cmc.sendMsg(Convert.RealPoseToMazeState(bac.getMazePose()));
                    trc.targetDrop();
                } else {
                    trc.removeDrop(Convert.RealPoseToMazeState(bac.getMazePose()));
                    cmc.sendMsg(Convert.RealPoseToMazeState(bac.getMazePose()));
                    if (trc.isGoldRemaining()) {
                        trc.targetGold();
                    } else {
                        System.out.printf("No gold remaining.\n");
                        gameActive = false;
                    }
                }
            }
            exe.setupTask(getNext(), bac.getMazePose());
        }
	}
	public synchronized void closeDebug() {
		System.out.println("Halting...");
        if (DEBUG_FLAG) {
            outRobo.close();
            outRoboEncode.close();
            outRoboMaze.close();
            outFollowTracker.close();
            outTrackMaze.close();
            outTrackRob.close();
            outTrackFMaze.close();
            outTrackFRob.close();
            outTrackWMaze.close();
        }
		System.out.println("Halted");
	}
	public synchronized void bumpRight(){
		wc.setALVel(1, 0);
		wc.updateWheels(robot, false);
		wc.setALVel(0, 0);
		wc.updateWheels(robot, false);
	}
	public synchronized void bumpLeft(){
		wc.setALVel(-1, 0);
		wc.updateWheels(robot, false);
		wc.setALVel(0, 0);
		wc.updateWheels(robot, false);
	}
}
