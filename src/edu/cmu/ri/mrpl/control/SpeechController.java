package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Speech;


public class SpeechController implements Runnable {
	private String speech;
	Thread t;
	private Speech hal;
	public SpeechController(java.lang.String input){
		speech = input;
		t = new Thread(this, "SpeechDriver");
		hal = new Speech();
		t.start();
	}
	@Override
	public void run() {
		// TODO Auto-generated method stub
		hal.speak(speech);
	}

}
