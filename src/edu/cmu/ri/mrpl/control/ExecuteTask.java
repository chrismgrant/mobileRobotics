package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Command;
import edu.cmu.ri.mrpl.Path;
import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.kinematics2D.Angle;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;

/**
 * Execution thread that controls robot with command
 * @author WangHeli
 *
 */
public class ExecuteTask implements Runnable{

	private static final double THRESHOLD = .01;

	private Robot robot;
	Thread t;
	private Command active;
	private boolean taskComplete;
	private RealPose2D initPose;
	private CommandController parent;
	
	//Arguments
	private Angle angArg;
	private int intArg;
	private double dblArg;
	private Path pthArg;
	private RealPoint2D pntArg;
	private RealPose2D pseArg;
 
	
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
		initPose = p;
		parent = parentController;
		
		switch (active.type){
		case TURNTO: {
			angArg = new Angle(Double.valueOf(active.argument.serialize()));
			break;
		}
		case GOTO: 
		case WAIT: {
			dblArg = Double.valueOf(active.argument.serialize());
			break;
		}
		case PAUSE:
		case NULL:
		default: {
			break;
		}
		}
		t.start();
	}
	
	/**
	 * Runs the thread. Used by Thread class
	 * Should only call accessor functions of parent controllers,
	 * with exception of Wheel controller.  
	 * 
	 * Run is terminated when condition for specified command type is fulfilled 
	 */
	public void run(){//TODO coordinate controller classes
		while (!taskComplete){
			switch (active.type){
			case TURNTO:{
				
				if ((angArg.angleValue() + initPose.getTh() - parent.bac.getDirection()) < THRESHOLD){
					taskComplete = true;
				}
				
				
				parent.wc.setCLVel(1, 0);
				parent.wc.updateWheels(robot,parent.bc.isBumped(robot));
				break;
			}
			case GOTO:{
				if ((dblArg -initPose.getPosition().distance(parent.bac.getPosition())) < THRESHOLD){
					taskComplete = true;
				}
				parent.wc.setCLVel(0,1);
				parent.wc.updateWheels(robot,parent.bc.isBumped(robot));
				break;
			}
			case PAUSE:{
				if (false){ //TODO check for keyboard input
					taskComplete = true;
				}
			}
			case WAIT:{
				try {
					Thread.sleep(Long.valueOf(Double.toString(Double.valueOf(active.argument.toString()) * 1000)));
					taskComplete = true;
				} catch (InterruptedException e){
					e.printStackTrace();
				}
				break;
			}
			case NULL:{
				
			}
			default:{
				break;
			}
			}
		}
	}
}