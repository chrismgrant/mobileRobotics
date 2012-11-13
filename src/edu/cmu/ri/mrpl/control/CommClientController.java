package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.CommClient;
import edu.cmu.ri.mrpl.CommClient.CommException;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.maze.MazePos;

import java.util.*;

/* This is a simple example of how to use the CommClient for 16x62.  
 * See the webpage for a full description.
 */

public class CommClientController {
	public CommClient comm;
	String myName;
	String Partner[];
	
	public CommClientController(){
		comm = new CommClient("128.237.203.134");
        Partner = new String[1];
        myName = "WD-40";
        Partner[0] = "DuctTape";

//        myName = "DuctTape";
//        Partner[0] = "WD-40";
		
	}
	
	public boolean connect(){
		try{
			//CommClient will throw an exception if it does not succeed
			comm.connectToFriends(myName, Partner);
		}
		catch(CommException e) {
			System.err.println("Comm Exception: " + e.getMessage());
			//All errors except missing-friends are not handled here.
			if(!e.getMessage().startsWith("missing-friends")){
				System.err.println("Giving up");
				return false;
			}
            System.out.print("waiting...\n");
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
	
	public void sendMsg(RealPoint2D myLoc) {
        String message = "Loc:"+myLoc.x+","+myLoc.y;
        comm.send(Partner[0], message);
    }
	public RealPoint2D swapLoc(RealPoint2D myLoc){
        String message = "Loc:"+myLoc.x+","+myLoc.y;
		String received;
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
					received = e1.getMessage().substring(4);
					cord = received.split(",");
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
	
	public void sendMazePath(ArrayList<MazePos> path){
		String message = "Path:";
		for(int i = 0; i < path.toArray().length; i++){
			message.concat();
		}
	}
	public ArrayList<MazePos> getMazePath(){
		String received;
		String cord[];
		ArrayList<MazePos> list = new ArrayList<MazePos>();
		try {
			comm.getIncomingMessage();
		} catch (CommException e1) {
			System.err.println("Comm Exception: " + e1);
			if(e1.getMessage().startsWith("Path:")){
				received = e1.getMessage().substring(5);
				String poses[] = received.split(";");
				for(int i = 0; i < poses.length; i++){
					cord = received.split(",");
					MazePos pos = new MazePos(Integer.parseInt(cord[0]),Integer.parseInt(cord[1]));
					list.add(pos);
				}
			}
		}
		return list;
	}
}
