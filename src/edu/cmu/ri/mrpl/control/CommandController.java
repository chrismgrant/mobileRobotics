package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Command;
import edu.cmu.ri.mrpl.CommandSequence;
import edu.cmu.ri.mrpl.Robot;
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
	SpeechController spc;
	
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
		spc = new SpeechController();
	}
	/**
	 * Passes robot object to CommandController. Must be called in run thread before while loop
	 * @param r robot
	 */
	public void syncRobot(Robot r){
		robot = r;
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
	public void addCommand(Command c){
		executeQueue.add(c);
	}
	/**
	 * Updates all controllers.
	 * To be called before accessing controller information
	 * @param sonars array of raw sonar values
	 */
	public void updateControllers(double[] sonars){
		soc.updateSonars(sonars);
		bac.updateBearing(wc.getRobLVel(robot), wc.getRobAVel(robot));
		trc.addTrackersFromSonar(soc.getSonarReadings(), bac.getPose());
		trc.updateTrackers();
		//TODO add VC update

	}
	/**
	 * Creates new execution thread to complete pending command.
	 */
	public void execute(){
		if (exe != null && active.type != Command.Type.NULL && !exe.t.isAlive()){
			new ExecuteTask(this, robot, active, bac.getPose());
		}
		
	}
	
	
}
