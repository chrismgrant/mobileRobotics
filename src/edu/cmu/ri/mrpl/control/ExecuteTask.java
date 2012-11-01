package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Command;
import edu.cmu.ri.mrpl.Command.AngleArgument;
import edu.cmu.ri.mrpl.Command.LengthArgument;
import edu.cmu.ri.mrpl.Command.PathArgument;
import edu.cmu.ri.mrpl.Command.PoseArgument;
import edu.cmu.ri.mrpl.Path;
import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.gui.PointsConsole;
import edu.cmu.ri.mrpl.kinematics2D.Angle;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;

import java.awt.geom.Point2D;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;


/**
 * Execution thread that controls robot with command
 * @author WangHeli
 *
 */
public class ExecuteTask{

	private static final double DIST_THRESHOLD = .003;
	private static final double ANG_THRESHOLD = .009;
	private static final int SPEECH_PREC = 3;
    private static final int INITIAL_SONARS = 50;

    private static enum ArgType {DISTANCE, ANGLE};

	private Robot robot;
	private Command active;
	private boolean taskComplete;
	private RealPose2D initPose;
	private CommandController parent;
	private double currentError;
	private String speech;
    private int pathIndex;

    //Arguments
	private boolean isContinuous;
	private Angle angArg;
	private int intArg;
	private double dblArg;
	private Path pthArg;
	private RealPoint2D pntArg;
	private RealPose2D pseArg;
	private int event;

	
	
	/**
	 * Initializes execution thread
     * @param r robot
     * @param c command
     * @param p initial Pose at execution time
     */
	public ExecuteTask(CommandController parentController, Robot r, Command c, RealPose2D p){
        robot = r;
        parent = parentController;
        setupTask(c,p);

	}

    void setupTask(Command command, RealPose2D robotPose) {
        stop();
        initPose = robotPose;
        active = command;
        isContinuous = command.isContinuous;
        taskComplete = false;
        pathIndex = 1;


        switch (active.type){
            case FOLLOWPATH: {
                PathArgument arg = (PathArgument)(active.argument);
                pthArg = parent.bhc.refinePath(parent.bac.getMazePose(),arg.path);
                System.out.println("next:\n"+ pthArg.toString());
                isContinuous = true;
                speech = "Following path.";
                break;
            }
            case POSETO: {
                PoseArgument arg = (PoseArgument)(active.argument);
                pseArg = new RealPose2D(arg.pose);
                speech = "Poseing";
                break;
            }
            case TURNTO: {
                AngleArgument arg = (AngleArgument)(active.argument);
                angArg = new Angle(arg.angle);
                speech = "Turn " + filterSpeech(angArg.angleValue(),SPEECH_PREC) + " rad";
                break;
            }
            case GOTO: {
                LengthArgument arg = (LengthArgument)(active.argument);
                dblArg = arg.d;
                speech = "Move " + filterSpeech(dblArg,SPEECH_PREC) + " m";
                break;
            }
            case WAIT: {
                LengthArgument arg = (LengthArgument)(active.argument);
                dblArg = arg.d;
                speech = "Wait " + filterSpeech(dblArg,SPEECH_PREC) + " s";
                break;
            }
            case PAUSE:{
                speech = "Pausing until keyboard press";
                break;
            }
            case NULL:
            default: {
                speech = "";
                break;
            }
        }
        speak(speech);

    }

    void speak(String in) {
        if (!in.isEmpty()) {
            new SpeechController(this,in);
        }
    }
	private double filterSpeech(double in, int decimals){
		int a = (int)Math.pow(10,decimals);
		double b = Math.pow(10.0, decimals);
		return (long)(in * a)/b;
	}
	/**
	 * Checks if error value is in threshold
	 * @param error error value
	 * @param argtype DISTANCE if distance, ANGLE otherwise
	 * @return whether error is within threshold
	 */
	private boolean isInThreshold(double error, ArgType argtype){
		double threshold = (argtype == ArgType.DISTANCE)?DIST_THRESHOLD:ANG_THRESHOLD;
		return Math.abs(error) < ((isContinuous) ? 100*threshold:threshold) ;
	}
	/**
	 * Stops the robot
	 */
	synchronized void stop(){
		parent.wc.setALVel(0, 0);
		parent.wc.updateWheels(robot,parent.bc.isBumped(robot));
	}

    /**
     * Initialize the pose of the robot, getting updates on maze position and wall placement
     */
    synchronized void initPose() {
        double[] sonars = new double[16];
        boolean debugFlag = parent.debugFlag;
        if (active.type == Command.Type.FOLLOWPATH) {
            for (int init = 0; init < 16; init++) {
                robot.updateState();
                robot.getSonars(sonars);
                parent.soc.updateSonars(sonars);
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    System.out.println("\"init warmup\" sleep interrupted");
                }
            }
        }
        for (int init = 0; init < INITIAL_SONARS; init++) {
            robot.updateState();
            robot.getSonars(sonars);
            parent.soc.updateSonars(sonars);
            parent.trc.forceAddTrackersFromSonar(parent.bac.getMazePose(),parent.soc.getSonarReadings());
            parent.trc.updateTrackers(parent.bac.getMazePose());
            try {
                Thread.sleep(50);
            } catch(InterruptedException iex) {
                System.out.println("\"init exec\" sleep interrupted");
            }
        }
        if (debugFlag) {
            PrintWriter outTrackInit = null;
            try {
                FileWriter outFileTrackInit = new FileWriter("TrackOutInit.txt");
                outTrackInit = new PrintWriter(outFileTrackInit);
            } catch (IOException e) {}
            outTrackInit.print("Gradient input: {");
            for (RealPoint2D p : parent.trc.getAllTrackerWPos(parent.bac.getMazePose())){
                outTrackInit.print("{"+p.getX()+","+p.getY()+"},");
            }
            outTrackInit.println("}");
            outTrackInit.close();
        }
        parent.bac.updateMazePoseBySonar(parent.trc.getMazeCorrection(parent.bac.getMazePose()));
    }
	/**
	 * Steps through execution.
	 * Should only call accessor functions of parent controllers,
	 * with exception of Wheel controller and updateControllers of parent.
	 * 
	 * Step returns true when condition for specified command type is fulfilled
	 */
	public synchronized boolean step(){
		//Pre-loop initialization
		double[] sonars = new double[16];
		boolean flag0 = false;

        robot.updateState();
        robot.getSonars(sonars);
        parent.updateControllers(sonars);

        System.out.println("RobotMazePose:"+parent.bac.getMazePose());
        //Loop VM
        switch (active.type){
            case FOLLOWPATH:{//Targets are relative to world
                RealPose2D targetWRTRob = null;
                RealPose2D currentTarget = pthArg.get(pathIndex);
    //                System.out.println("TargetPose:"+currentTarget);
                RealPose2D currentPose = parent.bac.getMazePose();
                RealPoint2D closePoint = parent.bhc.getClosestPoint(pthArg, currentPose.getPosition(), pathIndex);
                currentError = currentTarget.getPosition().distance(currentPose.getPosition());
    //                System.out.println("Error: "+currentError);
                if (isInThreshold(currentError, ArgType.DISTANCE)){
                    if (pathIndex == pthArg.size()-1){//If last target achieved
                        taskComplete = true;
                        double delta = parent.bac.getMazePose().getTh() - BearingController.getRDirection(robot);
                        currentError = Angle.normalize(pthArg.get(pathIndex).getTh() - parent.bac.getMazePose().getTh());
                        while (!isInThreshold(currentError, ArgType.ANGLE)) {
                            //TODO fix
                            currentError = Angle.normalize(pthArg.get(pathIndex).getTh() -
                                    (delta + BearingController.getRDirection(robot)));
                            parent.wc.setALVel(parent.bhc.turnTo(currentError), 0);
                            parent.wc.updateWheels(robot,parent.bc.isBumped(robot));
                            try {
                                Thread.sleep(20);
                            } catch (InterruptedException e) {
                                System.out.println("\"FollowPath turning\" interrupted");
                            }
                        }
                        stop();
                        speak("E" + filterSpeech(currentError,SPEECH_PREC) + " rad");

                        //Calculate error
                        double ex, ey;
                        ex = currentPose.getX() - currentTarget.getX();
                        ey = currentPose.getY() - currentTarget.getY();
                        parent.bac.updateError(ex,ey,currentError);
                    } else {
                        pathIndex++;
                        currentTarget = pthArg.get(pathIndex);
                        targetWRTRob = Convert.WRTRobot(currentPose, currentTarget);
                        double[] speed = parent.bhc.shadowPoint(targetWRTRob.getPosition());
                        parent.wc.setALVel(speed[0], speed[1]);
                        isContinuous = (pathIndex >= pthArg.size()-1)?false:true;
                    }
                } else if(closePoint.distance(currentPose.getPosition()) > .7){//Move toward closest point on path
                    targetWRTRob = Convert.WRTRobot(currentPose, new RealPose2D(closePoint,0.0));
                    double[] speed = parent.bhc.shadowPoint(targetWRTRob.getPosition());
                    parent.wc.setALVel(speed[0], speed[1]);
                } else { //Move to next way point
                    targetWRTRob = Convert.WRTRobot(currentPose, currentTarget);
                    double[] speed = parent.bhc.shadowPoint(targetWRTRob.getPosition());
                    parent.wc.setALVel(speed[0], speed[1]);
                }
                parent.wc.updateWheels(robot,parent.bc.isBumped(robot));
                break;
            }
            case POSETO:{//New target poses are relative to last target pose
                RealPose2D targetWRTRob = null;
                targetWRTRob = RealPose2D.multiply(parent.bac.getRPoseWithError(robot).inverse(),initPose);
                targetWRTRob = RealPose2D.multiply(targetWRTRob, pseArg);
                currentError = targetWRTRob.getPosition().distance(0,0);
                System.out.println(currentError);
                if (flag0 || isInThreshold(currentError, ArgType.DISTANCE)){
                    // Begin rotate subtask
                    flag0 = true;
                    currentError = Angle.normalize(pseArg.getRotateTheta() + initPose.getTh() - BearingController.getRDirection(robot));
                    System.out.println(currentError);
                    if (isInThreshold(currentError, ArgType.ANGLE)){
                        taskComplete = true;
                        stop();
                        speak("E" + filterSpeech(currentError,SPEECH_PREC) + " rad");

                        //Calculate error
                        double ex, ey;
                        RealPose2D targetWRTWorld = null;
                        targetWRTWorld = RealPose2D.multiply(BearingController.getRPose(robot),targetWRTRob);
                        ex = BearingController.getRX(robot) - targetWRTWorld.getX();
                        ey = BearingController.getRY(robot) - targetWRTWorld.getY();
                        parent.bac.updateError(ex,ey,currentError);
                    } else {
                        //logic
                        parent.wc.setALVel(parent.bhc.turnTo(currentError), 0);
                    }
                } else {
                    double[] speed = parent.bhc.shadowPoint(targetWRTRob.getPosition(), false, Double.POSITIVE_INFINITY, 0);
                    parent.wc.setCLVel(speed[0], speed[1]);
                }
                parent.wc.updateWheels(robot,parent.bc.isBumped(robot));
                break;
            }
            case TURNTO:{
                currentError = Angle.normalize(angArg.angleValue() + initPose.getTh() - BearingController.getRDirection(robot));
                System.out.println(currentError);
                if (isInThreshold(currentError, ArgType.ANGLE)){
                    taskComplete = true;
                    stop();
                    speak("E" + filterSpeech(currentError,SPEECH_PREC) + " rad");

                    parent.bac.updateError(0, 0, currentError);
                }else{
                    //logic
                    parent.wc.setALVel(parent.bhc.turnTo(currentError), 0);
                }
                parent.wc.updateWheels(robot,parent.bc.isBumped(robot));
                break;
            }
            case GOTO:{
                Point2D result = null;
                result = initPose.inverse().transform(BearingController.getRPose(robot).getPosition(), result);
                currentError = dblArg - result.getX();
                System.out.println(currentError);
                if (isInThreshold(currentError, ArgType.DISTANCE)){
                    taskComplete = true;
                    stop();
                    speak("E" + filterSpeech(currentError,SPEECH_PREC) + " m");

                    double dir = BearingController.getRDirection(robot);
                    parent.bac.updateError(currentError*Math.cos(dir), currentError*Math.sin(dir), 0);
                } else {
                    //logic
                    parent.wc.setALVel(0,parent.bhc.moveForward(currentError));
                }
                parent.wc.updateWheels(robot,parent.bc.isBumped(robot));
                break;
            }
            case PAUSE:{
                try {
                    System.in.read();//TODO switch to GUI input using wait lock
                } catch (IOException e) {
                    e.printStackTrace();
                }
                taskComplete = true;
                break;
            }
            case WAIT:{
                try {
                    Thread.sleep((long) dblArg * 1000);
                    taskComplete = true;
                } catch (InterruptedException e){
                    e.printStackTrace();
                }
                break;
            }
            case NULL:
            default:{
                taskComplete = true;
                break;
            }
        }
        return taskComplete;
    }
}
