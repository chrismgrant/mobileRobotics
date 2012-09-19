package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Command;
import edu.cmu.ri.mrpl.Path;
import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.Speech;
import edu.cmu.ri.mrpl.Task;
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

	private static final double THRESHOLD = .001;
	private static final int SPEECH_PREC = 3;

	private Robot robot;
	Thread t;
	private Command active;
	private boolean taskComplete, running;
	private RealPose2D initPose;
	private CommandController parent;
//	private Speech hal;
	private double currentError;
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
//		hal = new Speech();//TODO move speech to own thread
		
		switch (active.type){
		case TURNTO: {
			angArg = new Angle(Double.valueOf(active.argument.serialize()));
//			hal.speak("Turning " + filterSpeech(angArg.angleValue(),SPEECH_PREC) + " radians");
			st = new SpeechController(this,"Turn " + filterSpeech(angArg.angleValue(),SPEECH_PREC) + " rad");
			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
//			while(st.t.isAlive()){
//				
//			}
//			synchronized(this){
////				while(st.t.isAlive()){
//					try {
//						this.wait();
//					} catch (InterruptedException e) {
//						// TODO Auto-generated catch block
////						e.printStackTrace();
//					}
////				}
//			}
//			try {
//				st.wait();
//			} catch (InterruptedException e) {
//				// TODO Auto-generated catch block
//				//e.printStackTrace();
//			}
			break;
		}
		case GOTO: {
			dblArg = Double.valueOf(active.argument.serialize());
//			hal.speak("Moving " + filterSpeech(dblArg,SPEECH_PREC) + " meters forward");
			st = new SpeechController(this,"Move " + filterSpeech(dblArg,SPEECH_PREC) + " m");
			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			break;
		}
		case WAIT: {
			dblArg = Double.valueOf(active.argument.serialize());
//			hal.speak("Waiting " + filterSpeech(dblArg,SPEECH_PREC) + " seconds");
			st = new SpeechController(this,"Wait " + filterSpeech(dblArg,SPEECH_PREC) + " s");
			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			break;
		}
		case PAUSE:{
//			hal.speak("Pausing until keyboard press");
			st = new SpeechController(this,"Pausing until keyboard press");
			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
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
		parent.wc.setALVel(0, 0);
	}
	/**
	 * Runs the thread. Used by Thread class
	 * Should only call accessor functions of parent controllers,
	 * with exception of Wheel controller.  
	 * 
	 * Run is terminated when condition for specified command type is fulfilled 
	 */
	public synchronized void run(){
		while (running && !taskComplete){
			switch (active.type){
			case TURNTO:{ 
				currentError = Angle.normalize(angArg.angleValue() + initPose.getTh() - parent.bac.getDirection());
				System.out.println("Exe: currentE "+currentError+" target:"+angArg.angleValue()+" start:"+initPose.getTh()+" parent:"+parent.bac.getDirection());
				if (Math.abs(currentError) < ((isContinuous) ? 9*THRESHOLD:THRESHOLD*3)){
					taskComplete = true;
					parent.wc.setALVel(0, 0);
//					hal.speak("Error " + filterSpeech(currentError,SPEECH_PREC+1) + " radians"); 
					st = new SpeechController(this,"E" + filterSpeech(currentError,SPEECH_PREC) + " rad"); 
					try {
						Thread.sleep(3000);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}else{
					//logic
					parent.wc.setALVel(parent.bhc.turnTo(currentError), 0);
				}
				parent.wc.updateWheels(robot,parent.bc.isBumped(robot));
				break;
			}
			case GOTO:{ 
				Point2D result = null;
				result = initPose.inverse().transform(parent.bac.getPosition(), result);
				currentError = dblArg - result.getX();
				System.out.println("Exe: currentE "+currentError+" target:"+dblArg+" res:"+result.getX()+" start:"+initPose.toString()+" parent:"+parent.bac.toString());
				if (Math.abs(currentError) < ((isContinuous) ? 3*THRESHOLD:THRESHOLD)){
					taskComplete = true;
					parent.wc.setALVel(0, 0);
//					hal.speak("Error " + filterSpeech(currentError,SPEECH_PREC+1) + " meters"); 
					new SpeechController(this,"E" + filterSpeech(currentError,SPEECH_PREC) + " m"); 
					try {
						Thread.sleep(3000);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
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
			try {
				Thread.sleep(50);
//				Thread.sleep(25);
			} catch(InterruptedException iex) {
				System.out.println("\"Both\" sleep interrupted");
			}
		}
		System.out.println("Halted");
		//robot = null;
		
	}	
}
