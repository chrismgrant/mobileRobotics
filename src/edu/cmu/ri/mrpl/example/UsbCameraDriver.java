package edu.cmu.ri.mrpl.example;

import edu.cmu.ri.mrpl.usbCamera.*;
import javax.swing.*;
import java.awt.*;

/* This is a sample driver class for the UsbCamera Class */
public class UsbCameraDriver extends JFrame {
	private static final long serialVersionUID = 100L;

	public static void main(String args[]) {
		UsbCameraDriver go = new UsbCameraDriver();
		if(go == null);
	}

	private UsbCamera cam;

	public UsbCameraDriver() {
		super();
		
		/* Get our camera instance */
		cam = UsbCamera.getInstance();
		PicCanvas canvas = new PicCanvas();
		getContentPane().add(canvas, BorderLayout.CENTER);
		this.setSize(UsbCamera.XSIZE, UsbCamera.YSIZE);

		/* Display the window */
		this.setVisible(true);

		/* Now Loop */
		while(true) {
			/* Take a picture */
			cam.snap();
			canvas.setImage(cam.getImage());
			canvas.setRect(25, 25, 10, 10);

			for(int i=0; i<UsbCamera.XSIZE; i++) {
				for(int j=0;j<UsbCamera.YSIZE; j++) {
					try {
						cam.getRawPixel(i,j);
					} catch (Exception e) {
						System.out.println(e+" at " + i + "," + j);
					}
				}
			}

			/* Print out a random pixel */
			int pixel[] = cam.getPixel(30, 30);

			System.out.println(" r:"+pixel[UsbCamera.RED]+
					" g:"+pixel[UsbCamera.GREEN]+
					" b:"+pixel[UsbCamera.BLUE]);

			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {}

			canvas.unsetRect();

			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {}
		}
	}
}
