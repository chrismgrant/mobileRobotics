package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Speech;

public class SpeechController implements Runnable{

	private String speech;
	private Speech hal;
	Thread t;
	Object pt;
	
	public SpeechController(Object parent,String input){
		pt = parent;
		speech = input;
		t = new Thread(this, "soundDriver");
		hal = new Speech();
		t.start();
	}
	
	public synchronized void run(){
		hal.speak(speech);
	}
}
