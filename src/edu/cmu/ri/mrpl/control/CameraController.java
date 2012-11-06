package edu.cmu.ri.mrpl.control;

import java.awt.BorderLayout;

import javax.swing.*;
import java.awt.*;
import edu.cmu.ri.mrpl.usbCamera.PicCanvas;
import edu.cmu.ri.mrpl.usbCamera.UsbCamera;

/**
 * Created with IntelliJ IDEA.
 * User: WangHeli
 * Date: 11/4/12
 * Time: 5:41 PM
 * To change this template use File | Settings | File Templates.
 */
public class CameraController {

	
    public class CameraDriver extends JFrame {
        //TODO initialize webcam here
    	private static final long serialVersionUID = 100L;

    	public UsbCamera cam;
    	public PicCanvas canvas;
    	
    	public CameraDriver(){
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
        
        public boolean oneColor(int x, int y, int width, int height){
        	canvas.setRect(x, y, width, height);
        	int r =0 ,g =0 ,b =0;
        	int hits =0;
        	
        	for(int i=x; i<x+width; i++) {
    			for(int j=y;j<y+height; j++) {
    				try {
    					cam.getRawPixel(i,j);
    				} catch (Exception e) {
    					System.out.println(e+" at " + i + "," + j);
    				}
    				int pixel[] = cam.getPixel(i, j);
//    				System.out.print(" r:"+pixel[UsbCamera.RED]+
//    						"g:"+pixel[UsbCamera.GREEN]+
//    						"b:"+pixel[UsbCamera.BLUE]+" ; ");
    				if(j==y && i==x){
    					r = pixel[UsbCamera.RED];
    					g = pixel[UsbCamera.GREEN];
    					b = pixel[UsbCamera.BLUE];
    				}
    				else{
    					if(Math.abs(r-pixel[UsbCamera.RED])< 30){
    						r = (r+pixel[UsbCamera.RED])/2;
    					} else{
    						hits++;
    					}
    					if(Math.abs(g-pixel[UsbCamera.GREEN])< 30){
    						g = (g+pixel[UsbCamera.GREEN])/2;
    					} else{
    						hits++;
    					}
    					if(Math.abs(b-pixel[UsbCamera.BLUE])< 30){
    						b = (b+pixel[UsbCamera.BLUE])/2;
    					} else{
    						hits++;
    					}
    				}
    				if( hits > 30){
    					return false;
    				}
    				
    			}
    			System.out.println();
    		}
        	return true;
        }
        
    }
    CameraDriver racerX; 
    public CameraController(){
    	racerX = new CameraDriver();
    	
    }
    
    /**
     * Checks the camera to see if robot is holding gold
     * @return whether robot is holding gold
     */
    boolean holdingGold() {
        //TODO see if robot is holding gold.
    	//camera 320x240
    	racerX.update();
//    	racerX.printColors(90, 80, 5, 5);
        return racerX.oneColor(80,60,160,120);
    }

    /**
     * Checks the camera to see if gold is at target in front of robot
     * @return whether gold is in front of robot
     */
    boolean isGoldVisible() {
        //TODO see if robot sees gold in front.
        racerX.update();
//    	racerX.printColors(90, 80, 5, 5);
        return racerX.oneColor(80,60,160,120);
    }
    
}
