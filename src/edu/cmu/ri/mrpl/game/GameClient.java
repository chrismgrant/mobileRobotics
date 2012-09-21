package edu.cmu.ri.mrpl.game;


import edu.cmu.ri.mrpl.CommClient;
import edu.cmu.ri.mrpl.CommClient.CommException;
import edu.cmu.ri.mrpl.kinematics2D.*;
import edu.cmu.ri.mrpl.maze.*;


/*This class encapsulates communication with the GameServer
 */

public class GameClient {

    CommClient cc;
    boolean connectedToServer = false;
    String m_myName;
    String m_serverName;

    /** Send the pose to the gameServer
     * @param pose 2-D continuous pose in same convention as used by MazeGraphics */
    public void setPose(RealPose2D pose)
    {
	if (!connectedToServer)
	    return;
	String poseString = "POSE " + m_myName + " "  + pose.getX() + " " + pose.getY() + " " +pose.getTh();
	cc.send(m_serverName,poseString);
	
    }

    /** send a goal to the gameServer
     * @param pose 2-D continuous pose in same convention as used by MazeGraphics */
    public void setGoal(RealPose2D pose)
    {
	if (!connectedToServer)
	    return;
	String goalString = "GOAL " + m_myName + " "  + pose.getX() + " " + pose.getY() + " " + pose.getTh();
	cc.send(m_serverName,goalString);
    }

    /** indicate a new wall to the gameServer */
    public void addWall(int mazeX,int mazeY,MazeWorld.Direction dir)
    {
	if (!connectedToServer)
	    return;
	String wallString = "ADDWALL "   + mazeX + " " + mazeY + " " + dir;
	cc.send(m_serverName,wallString);
    }

    /** indicate a removed wall to the gameServer */
    public void removeWall(int mazeX,int mazeY,MazeWorld.Direction dir)
    {
	if (!connectedToServer)
	    return;
	String wallString = "RMWALL "   + mazeX + " " + mazeY + " " + dir;
	cc.send(m_serverName,wallString);
    }

    /** indicate a new (replaced?) gold to the gameServer */
    public void addGold(int mazeX,int mazeY,MazeWorld.Direction dir)
    {
	if (!connectedToServer)
	    return;
	String goldString = "ADDGOLD "   + mazeX + " " + mazeY + " " + dir;
	cc.send(m_serverName,goldString);
    }

    /** indicate a removed gold to the gameServer */
    public void removeGold(int mazeX,int mazeY,MazeWorld.Direction dir)
    {
	if (!connectedToServer)
	    return;
	String goldString = "RMGOLD "   + mazeX + " " + mazeY + " " + dir;
	cc.send(m_serverName,goldString);
    }

    /** indicate an emptied drop (verified drop failure?) to the gameServer */
    public void emptyDrop(int mazeX,int mazeY,MazeWorld.Direction dir)
    {
	if (!connectedToServer)
	    return;
	String dropString = "EMPTYDROP "   + mazeX + " " + mazeY + " " + dir;
	cc.send(m_serverName,dropString);
    }

    /** indicate a filled drop to the gameServer */
    public void fillDrop(int mazeX,int mazeY,MazeWorld.Direction dir)
    {
	if (!connectedToServer)
	    return;
	String dropString = "FILLDROP "   + mazeX + " " + mazeY + " " + dir;
	cc.send(m_serverName,dropString);
    }


    public boolean connectToServer(String myTeam,String myName, String myOtherName)
    {
	cc = new CommClient("gs5038.sp.cs.cmu.edu");
	connectedToServer = false;
	m_myName = "GC4" + myName;
	m_serverName = "GameServerForTeam" + myTeam;
	
	String myFriends[] = { m_serverName, "GC4" + myOtherName};

//	System.out.printf("GameClient: myname = \"%s\", friend1 = \"%s\", friend2 = \"%s\"\n", m_myName, myFriends[0], myFriends[1]);

	//try to connect to server
	try {
	    cc.connectToFriends(m_myName,myFriends);
	}
	catch(CommException e)
	    {

	       if (e.getMessage().startsWith("missing-friends"))
		   {
		       System.err.println("GameClient Unable to find friend: " + e.getMessage());
		       return false;
		   }
	       
	       //Wait until all friends connect.
	       boolean friendsReady = false;
	       while(!friendsReady)
		   {
		       try {
		       Thread.sleep(1000);
		       } catch (Exception e2) {}
		       try{
			   cc.getIncomingMessage();
		       }
		       catch (CommException e1) {
			   if(e1.getMessage().startsWith("friends-ready"))
			       friendsReady = true;
			   else
			       System.err.println("GameClient Giving up");
			   return false;
		       }
		   }
	    }

	connectedToServer = true;
	return true;
    }


}
	
