package edu.cmu.ri.mrpl.control;

import java.awt.BorderLayout;

import javax.swing.JFrame;

import edu.cmu.ri.mrpl.usbCamera.PicCanvas;
import edu.cmu.ri.mrpl.usbCamera.UsbCamera;

/**
 * Created with IntelliJ IDEA.
 * User: WangHeli
 * Date: 11/4/12
 * Time: 5:41 PM
 * To change this template use File | Settings | File Templates.
 */
public class CameraController extends JFrame {
	private static final long serialVersionUID = 100L;

	private UsbCamera cam;
	private PicCanvas canvas;
	
    public CameraController() {
        //TODO initialize webcam here
		cam = UsbCamera.getInstance();
		canvas = new PicCanvas();
		getContentPane().add(canvas, BorderLayout.CENTER);
		this.setSize(UsbCamera.XSIZE, UsbCamera.YSIZE);

		/* Display the window */
		this.setVisible(true);

    }

    public void update(){
    	cam.snap();
		canvas.setImage(cam.getImage());

    }
    
    public void printColors(int x, int y, int width, int height){
    	canvas.setRect(x, y, width, height);
    	for(int i=x; i<x+width; i++) {
			for(int j=y;j<y+height; j++) {
				try {
					cam.getRawPixel(i,j);
				} catch (Exception e) {
					System.out.println(e+" at " + i + "," + j);
				}
				int pixel[] = cam.getPixel(i, j);
				System.out.print(" r:"+pixel[UsbCamera.RED]+
						"g:"+pixel[UsbCamera.GREEN]+
						"b:"+pixel[UsbCamera.BLUE]+" ; ");
			}
			System.out.println();
		}
    }
    /**
     * Checks the camera to see if robot is holding gold
     * @return whether robot is holding gold
     */
    boolean holdingGold() {
        //TODO see if robot is holding gold.
        return false;
    }

    /**
     * Checks the camera to see if gold is at target in front of robot
     * @return whether gold is in front of robot
     */
    boolean isGoldVisible() {
        //TODO see if robot sees gold in front.
        return false;
    }
    
}
