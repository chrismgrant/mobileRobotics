package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Command;
import edu.cmu.ri.mrpl.Path;
import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.Speech;
import edu.cmu.ri.mrpl.kinematics2D.Angle;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import java.io.IOException;


/**
 * Execution thread that controls robot with command
 * @author WangHeli
 *
 */
public class ExecuteTask implements Runnable{

	private static final double THRESHOLD = .001;
	private static final int SPEECH_PREC = 3;

	private Robot robot;
	Thread t;
	private Command active;
	private boolean taskComplete, running;
	private RealPose2D initPose;
	private CommandController parent;
	private Speech hal;
	private double currentError;
	
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
		hal = new Speech();
		
		switch (active.type){
		case TURNTO: {
			angArg = new Angle(Double.valueOf(active.argument.serialize()));
			hal.speak("Turning " + filterSpeech(angArg.angleValue(),SPEECH_PREC) + " radians");
			break;
		}
		case GOTO: {
			dblArg = Double.valueOf(active.argument.serialize());
			hal.speak("Moving " + filterSpeech(dblArg,SPEECH_PREC) + " meters forward");
			break;
		}
		case WAIT: {
			dblArg = Double.valueOf(active.argument.serialize());
			hal.speak("Waiting " + filterSpeech(dblArg,SPEECH_PREC) + " seconds");
			break;
		}
		case PAUSE:{
			hal.speak("Pausing until keyboard press");
			break;
		}
		case NULL:
		default: {
			break;
		}
		}
		t.start();
	}
	
	private double filterSpeech(double in, int decimals){
		int a = (int)Math.pow(10,decimals);
		double b = Math.pow(10.0, decimals);
		return (long)(in * a)/b;
	}
	void halt(){
		running = false;
	}
	/**
	 * Runs the thread. Used by Thread class
	 * Should only call accessor functions of parent controllers,
	 * with exception of Wheel controller.  
	 * 
	 * Run is terminated when condition for specified command type is fulfilled 
	 */
	public void run(){//TODO coordinate controller classes
		while (running && !taskComplete){
			switch (active.type){
			case TURNTO:{
				currentError = angArg.angleValue() + initPose.getTh() - parent.bac.getDirection();
				System.out.println(currentError);
				if (Math.abs(currentError) < ((isContinuous) ? 3*THRESHOLD:THRESHOLD)){
					taskComplete = true;
					parent.wc.setALVel(0, 0);
					hal.speak("Error " + filterSpeech(currentError,SPEECH_PREC+1) + " radians"); 
				}else{
					//logic
					parent.wc.setALVel(parent.bhc.turnTo(currentError), 0);
				}
				parent.wc.updateWheels(robot,parent.bc.isBumped(robot));
				break;
			}
			case GOTO:{
				Angle a = new Angle(parent.bac.getDirection()); 
				Angle turn = new Angle(90);
				double slope = Math.atan(a.add(turn));
				double robX = parent.bac.getX();
				double robY = parent.bac.getY();
				double tgtX = initPose.getX();
				double tgtY = initPose.getY();
				boolean forward = (tgtY > slope*tgtX - slope*robX + robY);
				double dist = initPose.getPosition().distance(parent.bac.getPosition());
				currentError = dblArg - ((forward)?dist:-dist);
				System.out.println(currentError);
				if (Math.abs(currentError) < ((isContinuous) ? 3*THRESHOLD:THRESHOLD)){
					taskComplete = true;
					parent.wc.setALVel(0, 0);
					hal.speak("Error " + filterSpeech(currentError,SPEECH_PREC+1) + " meters"); 
				}else{
					//logic
					parent.wc.setALVel(0,parent.bhc.moveForward(currentError));
				}				
				parent.wc.updateWheels(robot,parent.bc.isBumped(robot));
				break;
			}
			case PAUSE:{
				/*if (event!=0){ //TODO check for keyboard input
					taskComplete = true;
					event = 0;
				}*/
				try {
					System.in.read();
				} catch (IOException e) {
					// TODO Auto-generated catch block
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
//			try {
//				Thread.sleep(25);
//			} catch(InterruptedException iex) {
//				System.out.println("\"Both\" sleep interrupted");
//			}
		}
		
	}	
}
