package edu.cmu.ri.mrpl.example;

/*
 * This file contains examples of how to play audio files and use a simple
 * text-to-speech system.
 */
import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.DataLine;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.SourceDataLine;
import javax.sound.sampled.UnsupportedAudioFileException;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Mixer;

import java.io.*;
import edu.cmu.ri.mrpl.*;

class SoundExample{
	/*
	 * Takes in a file name and plays it.
	 * This method will not work with all audio files, give it a try with
	 * the audio files you want to use and see if it works.
	 */
	public static void playSound(String filename){
		try {
			// Open an input stream  to the audio file.
			File soundFile = new File(filename);

			// Create an AudioInputStream object from the input stream.
			//AudioStream as = new AudioStream(in);
			AudioInputStream audioInputFile = AudioSystem.getAudioInputStream(soundFile);
			AudioInputStream ais = AudioSystem.getAudioInputStream(AudioFormat.Encoding.PCM_SIGNED, audioInputFile);

			AudioFormat format = ais.getFormat();
			SourceDataLine auline = getSourceDataLine(format);


			// Start reading bytes from the file and playing them.
			int bufSize = 8200;
			byte[] data = new byte[bufSize];
			int bytesRead;
			while (( bytesRead=ais.read(data,0,data.length)) != -1){
				auline.write(data, 0, bytesRead);
			}
			
			auline.drain();
			auline.stop();
			auline.close();
		} catch (LineUnavailableException e) {
			e.printStackTrace();
		} catch (UnsupportedAudioFileException uafe) {
			uafe.printStackTrace();
		} catch (IOException ioe) {
			ioe.printStackTrace();
		} catch (Exception e){
			e.printStackTrace();
		}	
	}

	/**
	 * This class is a wrapper around FreeTTS.  It simplifies
	 * your user experience.  Only make one instance of it for
	 * all consumers!!  To do otherwise will result in memory 
	 * leaks or worse.
	 */
	static Speech speech = new Speech();
	
	/**
	 * This function demonstrates how to speak some text.
	 */
	public static void sayText(String text){
		speech.speak(text);
	}
	
	public static void main(String[] argv){
		playSound("/home/furlong/projects/ta-16-362/src/distribution/illah.wav");
//		sayText("This is a test.  Goodbye.");
	}	
	
	
	private static SourceDataLine getSourceDataLine(AudioFormat format) throws LineUnavailableException {
		DataLine.Info info = new DataLine.Info(SourceDataLine.class, format);
		for (Mixer.Info mi : AudioSystem.getMixerInfo()) {
			SourceDataLine dataline = null;
				
			try{
				Mixer mixer = AudioSystem.getMixer(mi);
				dataline = (SourceDataLine)mixer.getLine(info);
				dataline.open(format);
				dataline.start();
				return dataline;
			} catch (IllegalArgumentException iae) {
				// suppress exception and try next data line.
			}
			if (dataline != null){
				try {
					dataline.close();
				}
				catch (Exception e) {}
			}
		}
		return null;
	}
}
