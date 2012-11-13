package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.CommClient;
import edu.cmu.ri.mrpl.CommClient.CommException;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;

import java.util.*;

/* This is a simple example of how to use the CommClient for 16x62.  
 * See the webpage for a full description.
 */

public class CommClientController {
	public CommClient comm;
	String myName;
	String Partner[];
	
	public CommClientController(){
		comm = new CommClient("gs5038.sp.cs.cmu.edu");
		myName = "Me";
        Partner = new String[1];
		Partner[0] = "You";
		
	}
	
	public boolean connect(){
		try{
			//CommClient will throw an exception if it does not succeed
			comm.connectToFriends(myName, Partner);
		}
		catch(CommException e) {
			System.err.println("Comm Exception: " + e);
			//All errors except missing-friends are not handled here.
			if(!e.getMessage().startsWith("missing-friends")){
				System.err.println("Giving up");
				return false;
			}

			//Wait until all friends connect.
			boolean friendsReady = false;
			while(!friendsReady){
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e1) {
					e1.printStackTrace();
				}
				try {
					comm.getIncomingMessage();
				} catch (CommException e1) {
					System.err.println("Comm Exception: " + e1);
					if(e1.getMessage().startsWith("friends-ready"))
						friendsReady = true;
					else{
						//Again, anything except freinds-ready is not handled.
						System.err.println("Giving up");
						return false;
					}
				}
			}
			return true;
		}
		return false;
	}
	
	
	public RealPoint2D swapLoc(RealPoint2D myLoc){
		String message = "Loc:"+myLoc.x+","+myLoc.y;
		String recived;
		String cord[];
		RealPoint2D partnerLoc = new RealPoint2D(-1,-1);
		boolean swap = false;
		while(!swap){
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e1) {
				e1.printStackTrace();
			}
			try {
				comm.getIncomingMessage();
			} catch (CommException e1) {
				System.err.println("Comm Exception: " + e1);
				if(e1.getMessage().startsWith("Loc:")){
					swap = true;
					recived = e1.getMessage().substring(4);
					cord = recived.split(",");
					partnerLoc = new RealPoint2D(Double.parseDouble(cord[0]),Double.parseDouble(cord[1]));
				}else{
					System.err.println("Giving up");
					return new RealPoint2D(-1,-1);
				}
			}
		comm.send(Partner[0], message);
		}
		return partnerLoc;
	}
}
