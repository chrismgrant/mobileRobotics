package edu.cmu.ri.mrpl.game;

import edu.cmu.ri.mrpl.CommClient;
import edu.cmu.ri.mrpl.CommClient.CommException;
import edu.cmu.ri.mrpl.kinematics2D.*;
import edu.cmu.ri.mrpl.maze.*;
import java.util.*;
import java.io.*;


import java.util.Calendar;

public class GameServer {
	protected static MazeWorld mw;
	protected static GameDisplay disp;
	protected static boolean flipped;
	protected static RealPose2D maze_origin;
    
   
    public static String[] findRobotNamesFromLog(String logFile)
    {
	String[] myFriends = {"",""};
	int numFoundFriends = 0;
			BufferedReader input = null;
		try {
		    input =  new BufferedReader(new FileReader(new File(logFile)));
		}
		catch (IOException e) {
		    System.err.println("Cannot open specified log file");
		    System.err.println(e);
		    System.exit(-1);	
		}
		String line = null;
		do 
		    {
			try {
			    line = input.readLine();
			}
			catch (IOException e) {
			    System.err.println("Error reading log file");
			    System.err.println(e);
			    System.exit(-1);	
			}
			if (line == null) 
			    continue;
			//split line into message and timestamp
			String[] tokens = line.split(":");
						
			//build up friends if necessary
			if (numFoundFriends < 2)
			    {
				if (tokens[1].contains("POSE") || 
				    tokens[1].contains("GOAL"))
				    {
					String[] toks = tokens[1].split(" ");
					if (numFoundFriends != 1 ||
					    (!myFriends[0].equals(toks[1])))
					    {
						myFriends[numFoundFriends] = toks[1];
						numFoundFriends++;
					    }
				    }
			    }
			else
			    {
				try {
				    input.close(); }
				catch (IOException e) {
				    System.err.println("Cannot close specified log file");
				    System.err.println(e);
				    System.exit(-1);	
				}
				return myFriends;
			    }
		    }
		while(line != null);
		return myFriends;
    }
	














    protected static void parseMessageAndUpdateGraphics(String lastMessage,String [] myFriends)
    {

	String[] tokens = lastMessage.split(" ");
	if (lastMessage.contains("POSE"))
	    {
		int robotNumber = -1;
		if (tokens[1].equals(myFriends[0]))
		    robotNumber = 0;
		else if (tokens[1].equals(myFriends[1]))
		    robotNumber = 1;
		else System.out.println("UNKNOWN ROBOT NUMBER " + tokens[1]);

		RealPose2D p = new RealPose2D(Double.parseDouble(tokens[2]),
					      Double.parseDouble(tokens[3]),
					      Double.parseDouble(tokens[4]));
		System.out.println("Received Pose From Robot " + robotNumber + " " + p.getX() + " " + p.getY() + " " + p.getTh());

		p = RealPose2D.multiply(maze_origin, p);

		disp.setPose(robotNumber,p);
	    }
	

	else if (lastMessage.contains("GOAL"))
	    {

		int robotNumber = -1;
		if (tokens[1].equals(myFriends[0]))
		    robotNumber = 0;
		else if (tokens[1].equals(myFriends[1]))
		    robotNumber = 1;
		else System.out.println("UNKNOWN ROBOT NUMBER " + tokens[1]);

		RealPose2D p = new RealPose2D(Double.parseDouble(tokens[2]),
					      Double.parseDouble(tokens[3]),
					      Double.parseDouble(tokens[4]));

		System.out.println("Received Goal From Robot " + robotNumber + " " + p.getX() + " " + p.getY() + " " + p.getTh());

		p = RealPose2D.multiply(maze_origin, p);

		disp.setGoal(robotNumber,p);

	    }
	

	else if (lastMessage.contains("ADDWALL"))
	    {

		int mazeX = Integer.parseInt(tokens[1]);
		int mazeY = Integer.parseInt(tokens[2]);
		MazeWorld.Direction mazeDir = MazeWorld.Direction.valueOf(tokens[3]);

		System.out.println("Received ADDWALL " + mazeX + " " + mazeY + " " + mazeDir);

		if (flipped) {
			mazeX = mw.getWidth() - 1 - mazeX;
			mazeY = mw.getHeight() - 1 - mazeY;
			mazeDir = mazeDir.rear();
		}

		disp.addWall(mazeX, mazeY, mazeDir);
	    }
	
	else if (lastMessage.contains("RMWALL"))
	    {
		int mazeX = Integer.parseInt(tokens[1]);
		int mazeY = Integer.parseInt(tokens[2]);
    MazeWorld.Direction mazeDir = MazeWorld.Direction.valueOf(tokens[3]);

		System.out.println("Received RMWALL "  + mazeX + " " + mazeY + " " + mazeDir);

		if (flipped) {
			mazeX = mw.getWidth() - 1 - mazeX;
			mazeY = mw.getHeight() - 1 - mazeY;
			mazeDir = mazeDir.rear();
		}

		disp.removeWall(mazeX, mazeY, mazeDir);
	    }

	else if (lastMessage.contains("ADDGOLD"))
	    {
		int mazeX = Integer.parseInt(tokens[1]);
		int mazeY = Integer.parseInt(tokens[2]);
    MazeWorld.Direction mazeDir = MazeWorld.Direction.valueOf(tokens[3]);

		System.out.println("Received ADDGOLD " + mazeX + " " + mazeY + " " + mazeDir);

		if (flipped) {
			mazeX = mw.getWidth() - 1 - mazeX;
			mazeY = mw.getHeight() - 1 - mazeY;
			mazeDir = mazeDir.rear();
		}

		disp.addGold(mazeX, mazeY, mazeDir);
	    }
	else if (lastMessage.contains("RMGOLD"))
	    {
		int mazeX = Integer.parseInt(tokens[1]);
		int mazeY = Integer.parseInt(tokens[2]);
    MazeWorld.Direction mazeDir = MazeWorld.Direction.valueOf(tokens[3]);

		System.out.println("Received RMGOLD " + mazeX + " " + mazeY + " " + mazeDir);

		if (flipped) {
			mazeX = mw.getWidth() - 1 - mazeX;
			mazeY = mw.getHeight() - 1 - mazeY;
			mazeDir = mazeDir.rear();
		}

		disp.removeGold(mazeX, mazeY, mazeDir);
	    }

	else if (lastMessage.contains("FILLDROP"))
	    {
		int mazeX = Integer.parseInt(tokens[1]);
		int mazeY = Integer.parseInt(tokens[2]);
    MazeWorld.Direction mazeDir = MazeWorld.Direction.valueOf(tokens[3]);

		System.out.println("Received FILLDROP " + mazeX + " " + mazeY + " " + mazeDir);

		if (flipped) {
			mazeX = mw.getWidth() - 1 - mazeX;
			mazeY = mw.getHeight() - 1 - mazeY;
			mazeDir = mazeDir.rear();
		}

		disp.fillDrop(mazeX, mazeY, mazeDir);
	    }

	else if (lastMessage.contains("EMPTYDROP"))
	    {
		int mazeX = Integer.parseInt(tokens[1]);
		int mazeY = Integer.parseInt(tokens[2]);
    MazeWorld.Direction mazeDir = MazeWorld.Direction.valueOf(tokens[3]);

		System.out.println("Received EMPTYDROP " + mazeX + " " + mazeY + " " + mazeDir);

		if (flipped) {
			mazeX = mw.getWidth() - 1 - mazeX;
			mazeY = mw.getHeight() - 1 - mazeY;
			mazeDir = mazeDir.rear();
		}

		disp.emptyDrop(mazeX, mazeY, mazeDir);
	    }

	else 
	    System.out.println ("UNRECOGNIZED MESSAGE: " + lastMessage);
    }


    public static void main (String args[] )
    {
    	flipped = false;

    	if (args.length >= 1 && args[0].equals("-flip")) {
		String[] args2 = new String[args.length-1];
		System.arraycopy(args, 1, args2, 0, args.length-1);
		args = args2;
		flipped = true;
	}

	if(args.length != 4 && args.length != 5 && args.length != 2 )
	    {
		System.out.println("Usage: java GameServer mazeFile TeamName RobotName1 RobotName2 (logfile)");
		System.out.println("This will run the server, and log to logFile if provided\n");
		System.out.println("java GameServer mazeFile logFile");
		System.out.println("This will play back a logfile");
		
		System.exit(-1);
	    }

	mw = null;
	try {
    mw = new MazeWorld(new File(args[0]));
	}
	catch (IOException e) {
		System.err.println("Cannot open specified mazeworld file");
		System.err.println(e);
		System.exit(-1);	
	}

	if (flipped) {
		maze_origin = new RealPose2D(mw.getWidth()-1, mw.getHeight()-1, Math.PI);
		Set<MazeState> old_drops = mw.getDrops();
		Set<MazeState> new_drops = new HashSet<MazeState>();

		for (MazeState ms: old_drops)
			new_drops.add(new MazeState(mw.getWidth() - 1 - ms.x(), mw.getHeight() - 1 - ms.y(), ms.dir().rear()));

		mw.removeAllDrops();

		for (MazeState ms: new_drops)
			mw.addDrop(ms);
	}
	else
		maze_origin = new RealPose2D();

	if (args.length == 2)
	    {


		String[] myFriends = findRobotNamesFromLog(args[1]);
		String teamName = "Log Playback";
		System.out.println("Parsed Robots Names From File:  " + myFriends[0] + "  and  " + myFriends[1]);

		disp = new GameDisplay(mw, teamName, myFriends);
		BufferedReader input = null;
		try {
		    input =  new BufferedReader(new FileReader(new File(args[1])));
		}
		catch (IOException e) {
		    System.err.println("Cannot open specified log file");
		    System.err.println(e);
		    System.exit(-1);	
		}
		String line = null;
		double simStartTime = -1;
		double realStartTime = Calendar.getInstance().getTimeInMillis()/1000.0;
		do 
		    {
			try {
			    line = input.readLine();
			}
			catch (IOException e) {
			    System.err.println("Error reading log file");
			    System.err.println(e);
			    System.exit(-1);	
			}
			if (line == null) 
			    continue;
			//split line into message and timestamp
			String[] tokens = line.split(":");
			double timestamp = Double.parseDouble(tokens[0]) / 1000.0;
			if (simStartTime < 0)
			    simStartTime = timestamp;

			
			
			//sleep if necessary
			double currentRealTime =  Calendar.getInstance().getTimeInMillis()/1000.0 - realStartTime;
			double desiredSimTime = timestamp - simStartTime;
		
			if (desiredSimTime > currentRealTime)
			    {
				try {
				    Thread.sleep(new Double((desiredSimTime - currentRealTime)*1000.0).intValue());
				} catch (InterruptedException e1) {
				    e1.printStackTrace();
				
				}
			    }
			parseMessageAndUpdateGraphics(tokens[1],
						      myFriends);
				    
		    }
		while (line != null);
		try {
		    input.close();
		}
		catch (IOException e) {
		    System.err.println("Cannot close specified log file");
		    System.err.println(e);
		    System.exit(-1);	
		}


	    }
			  



     
	if (args.length >= 4)
	    {

		String teamName = args[1];
		String myName = "GameServerForTeam" + teamName;
		String [] myFriends = {"GC4" + args[2],"GC4" + args[3]};
		String [] myFriends_disp = {args[2],args[3]};


		//initialize comm client
		CommClient cc = new CommClient("gs5038.sp.cs.cmu.edu");
		try{
		    
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
		

		boolean logging = false;
		FileWriter outFile;
		PrintWriter out = null;
		
		
		if (args.length == 5)
		    {
			logging = true;
			try {
			    outFile = new FileWriter(args[4]);  
			    out = new PrintWriter(outFile);
			}
			catch (Exception e)
			    {
				System.out.println("Cannot open " + args[4] + " for writing");
				System.exit(-1);
			    }
		    }
	
    disp = new GameDisplay(mw, teamName, myFriends_disp);
		
		//now loop forever, listening to messages
		while(true)
		    {
			String lastMessage = null;
			do{
			    try {
				lastMessage = cc.getIncomingMessage();
			    } catch (CommException e) {
				e.printStackTrace();
				break;
			    }
			}while(lastMessage == null);
			//System.out.println(lastMessage);
			if (lastMessage != null)
			    {
				if (logging)
				    {
					Calendar now = Calendar.getInstance();
					out.println(now.getTimeInMillis()
						    + ":" +  lastMessage);
					out.flush();
				    }
				parseMessageAndUpdateGraphics(lastMessage,
							      myFriends);
			    }
		    }
		
		    
			
	    }// if running live
	       
	
    }
}
