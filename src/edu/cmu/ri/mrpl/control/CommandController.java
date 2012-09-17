package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Command;
import edu.cmu.ri.mrpl.CommandSequence;


/**
 * Handles command sequencing
 * @author WangHeli
 *
 */
public class CommandController {

	ExecuteTask exe;
	final Command nullCommand = new Command();
	Command active;
	CommandSequence executeQueue;
	
	/**
	 * Initializes a new CommandController
	 */
	public CommandController(){
		executeQueue = new CommandSequence();
		active = new Command();
	}
	/**
	 * Gets next command in execution stack, or returns null command if empty
	 * @return Next command to execute
	 */
	public Command getNext(){
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
	public Command.Type getActiveCommand(){
		return active.type;
	}
	/**
	 * Gets the argument of the current command
	 * @return Argument for current command
	 */
	public Command.Argument getActiveCommandArg(){
		return active.argument;
	}
	/**
	 * Enqueues command to executeQueue
	 * @param c New command to add
	 */
	public void addCommand(Command c){
		executeQueue.add(c);
	}
	
	public void execute(){
		if (exe != null && active.type != Command.Type.NULL && !exe.t.isAlive()){
			new ExecuteTask();
		}
	}
	
	
}
