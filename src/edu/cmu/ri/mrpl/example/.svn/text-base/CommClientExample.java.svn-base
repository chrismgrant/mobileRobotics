package edu.cmu.ri.mrpl.example;

import edu.cmu.ri.mrpl.CommClient;
import edu.cmu.ri.mrpl.CommClient.CommException;

import java.util.*;

/* This is a simple example of how to use the CommClient for 16x62.  
 * See the webpage for a full description.
 */

public class CommClientExample {
	public static void main(String[] args) {
		CommClient cc = new CommClient("gs5038.sp.cs.cmu.edu");
		
		// Get friends 
		if(args.length != 2){
			System.out.println("Usage - java CommClientExample name1 name2");
			System.exit(-1);
		}
		String myName = args[0];
		String myFriends[] = {args[1]};

		try{
			//CommClient will throw an exception if it does not succeed
			cc.connectToFriends(myName, myFriends);
		}
		catch(CommException e) {
			System.err.println("Comm Exception: " + e);
			//All errors except missing-friends are not handled here.
			if(!e.getMessage().startsWith("missing-friends")){
				System.err.println("Giving up");
				return;
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
					cc.getIncomingMessage();
				} catch (CommException e1) {
					System.err.println("Comm Exception: " + e1);
					if(e1.getMessage().startsWith("friends-ready"))
						friendsReady = true;
					else{
						//Again, anything except freinds-ready is not handled.
						System.err.println("Giving up");
						return;
					}
				}
			}
		}

		for(int i=0; i<10; i++){
			String message = "Hello " + myFriends[0] + ", it is " + new Date(System.currentTimeMillis()) + ".  Message " + (i+1) + "/10";
			cc.send(myFriends[0], message);
			
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e1) {
				e1.printStackTrace();
			}
			
			do{
				try {
					message = cc.getIncomingMessage();
				} catch (CommException e) {
					e.printStackTrace();
					break;
				}
			}while(message == null);
			System.out.println(message);
		}		
	}
}
