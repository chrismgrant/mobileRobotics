package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Command;
import edu.cmu.ri.mrpl.Robot;


public class ExecuteTask implements Runnable{

	Robot robot;
	Thread t;
	Command active;
	boolean taskComplete;
	public ExecuteTask(Robot r,Command c){
		t = new Thread(this, "ExecuteTask");
		active = c;
		robot = r;
		taskComplete = false;
		t.start();
	}
	
	
	public void run(){
		while (!taskComplete){
			switch (active.type){
			case TURNTO:{
				
				break;
			}
			case GOTO:{
				
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
