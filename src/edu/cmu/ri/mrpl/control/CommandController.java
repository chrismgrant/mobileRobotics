package edu.cmu.ri.mrpl.control;

import java.io.IOException;
import java.lang.*;
import edu.cmu.ri.mrpl.Command;
import edu.cmu.ri.mrpl.CommandSequence;
import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import edu.cmu.ri.mrpl.lab.Lab3;


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
	
	Robot robot;
	
	private ExecuteTask exe;
	private final Command nullCommand = new Command();
	private Command active;
	private CommandSequence executeQueue;
	
	/**
	 * Initializes a new CommandController
	 */
	public CommandController(){
		executeQueue = new CommandSequence();
		active = new Command();
		
		wc = new WheelController();
		soc = new SonarController();
		bc = new BumperController();
		
		trc = new TrackerController();
		bac = new BearingController();
		bhc = new BehaviorController();
		vc = new VisualizeController();
	}
	/**
	 * Passes robot object to CommandController. Must be called in run thread before while loop
	 * @param r robot
	 */
	public synchronized void syncRobot(Robot r){
		robot = r;
		exe = new ExecuteTask(this, robot, nullCommand, bac.getPose());
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
		soc.updateSonars(sonars);
		wc.updateWheels(robot, bc.isBumped(robot));
		bac.updateBearing(WheelController.getRobLVel(robot), WheelController.getRobAVel(robot));
		trc.addTrackersFromSonar(soc.getSonarReadings());
		trc.updateTrackers( bac.getDeltaPose());
		//TODO add VC update

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
