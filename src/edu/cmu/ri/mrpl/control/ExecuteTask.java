package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Command;
import edu.cmu.ri.mrpl.Command.AngleArgument;
import edu.cmu.ri.mrpl.Command.LengthArgument;
import edu.cmu.ri.mrpl.Command.PathArgument;
import edu.cmu.ri.mrpl.Command.PoseArgument;
import edu.cmu.ri.mrpl.Path;
import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.kinematics2D.Angle;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;

import java.awt.geom.Point2D;
import java.io.IOException;


/**
 * Execution thread that controls robot with command
 * @author WangHeli
 *
 */
public class ExecuteTask implements Runnable{

	private static final double DIST_THRESHOLD = .001;
	private static final double ANG_THRESHOLD = .009;
	private static final int SPEECH_PREC = 3;
	private static enum ArgType {DISTANCE, ANGLE};

	private Robot robot;
	Thread t;
	private Command active;
	private boolean taskComplete, running;
	private RealPose2D initPose;
	private CommandController parent;
	private double currentError;
	private String speech;
	private SpeechController st;
	
	
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
		t = new Thread(this, "ExecuteTask");
		active = c;
		robot = r;
		taskComplete = false;
		running = true;
		initPose = p;
		parent = parentController;
		isContinuous = c.isContinuous;
		
		switch (active.type){
		case FOLLOWPATH: {
			PathArgument arg = (PathArgument)(active.argument);
			pthArg = arg.path;
			speech = "Path to " + "";
			break;
		}
		case POSETO: {
			PoseArgument arg = (PoseArgument)(active.argument);
			pseArg = new RealPose2D(arg.pose);
			speech = "Pose to " + pseArg.toString();
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
		st = new SpeechController(this,speech);
		try {
			Thread.sleep(3000);
		} catch (InterruptedException e) {}
		t.start();
	}
	
	private double filterSpeech(double in, int decimals){
		int a = (int)Math.pow(10,decimals);
		double b = Math.pow(10.0, decimals);
		return (long)(in * a)/b;
	}
	/**
	 * Halts execution of thread
	 */
	void halt(){
		running = false;
		stop();
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
	private void stop(){
		parent.wc.setALVel(0, 0);
		parent.wc.updateWheels(robot,parent.bc.isBumped(robot));
	}
	/**
	 * Runs the thread. Used by Thread class
	 * Should only call accessor functions of parent controllers,
	 * with exception of Wheel controller.  
	 * 
	 * Run is terminated when condition for specified command type is fulfilled 
	 */
	public synchronized void run(){
		//Pre-loop initialization
		double[] sonars = new double[16];
		boolean flag0 = false;
		
		while (running && !taskComplete){
			// Loop header
			robot.updateState();
			robot.getSonars(sonars);
			parent.updateControllers(sonars);
			
			//Loop VM
			switch (active.type){
			case FOLLOWPATH:{//Targets are relative to world
				
				RealPose2D currentTarget = pthArg.get(0);
				currentError = currentTarget.getPosition().distance(BearingController.getRPose(robot).getPosition());
				if (isInThreshold(currentError, ArgType.DISTANCE)){
					if (pthArg.size()==1){//If last target achieved
						taskComplete = true;
						stop();
						st = new SpeechController(this,"E" + filterSpeech(currentError,SPEECH_PREC) + " rad"); 
						try {
							Thread.sleep(3000);
						} catch (InterruptedException e) {}
						
						//Calculate error
						double ex, ey;
						ex = BearingController.getRX(robot) - currentTarget.getX();
						ey = BearingController.getRY(robot) - currentTarget.getY();
						parent.bac.updateError(ex,ey,currentError);
					} else {//If intermediary step achieved
						pthArg.remove(0);
						isContinuous = (pthArg.size()<=1)?false:true;
					}
				} else {//Move toward closest point on path
					//TODO complete
				}
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
						st = new SpeechController(this,"E" + filterSpeech(currentError,SPEECH_PREC) + " rad"); 
						try {
							Thread.sleep(3000);
						} catch (InterruptedException e) {}
						
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
					st = new SpeechController(this,"E" + filterSpeech(currentError,SPEECH_PREC) + " rad"); 
					try {
						Thread.sleep(3000);
					} catch (InterruptedException e) {}
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
					new SpeechController(this,"E" + filterSpeech(currentError,SPEECH_PREC) + " m"); 
					try {
						Thread.sleep(3000);
					} catch (InterruptedException e) {}
					double dir = BearingController.getRDirection(robot);
					parent.bac.updateError(currentError*Math.cos(dir), currentError*Math.sin(dir), 0);
				}else{
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
			try {
				Thread.sleep(20);
			} catch(InterruptedException iex) {
				System.out.println("\"Both\" sleep interrupted");
			}
		}
		System.out.println("Exec halted");
	}	
}
