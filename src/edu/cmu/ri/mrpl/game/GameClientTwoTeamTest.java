package edu.cmu.ri.mrpl.game;

import edu.cmu.ri.mrpl.kinematics2D.*;


public class GameClientTwoTeamTest {

    public static void main(String[] args)
    {
	GameClientTwoTeam gc = new GameClientTwoTeam();
	if(args.length != 3){
		System.out.println("Usage - java GameClientTest matchName name1 name2");
		System.exit(-1);
	}
	//robots on your team
	boolean connected =gc.connectToServer(args[0],args[1],args[2]);
//	RealPose2D p = new RealPose2D(0.0,0.0,0.0);
//	RealPose2D goalPose = new RealPose2D(0.0,0.0,1.0);
	if (!connected)
	    System.out.println("Not actually connected, but still calling update functions");

	RealPose2D pose = new RealPose2D();
	RealPose2D goal = new RealPose2D();

	while (true) {
		gc.setPose(pose);
		gc.setGoal(goal);
		try {
		    Thread.sleep(100);
		} catch (Exception e) {}
		pose.setPose(0, 1, 0);
		goal.setPose(0, 2, 0);
	}

/*
	//if (connected)
	    {
		for (int i=0;i<20;i++)
		    {
			try {
			    Thread.sleep(1000);
			} catch (Exception e) {}
			gc.setGoal(goalPose);
			gc.setPose(p);
			gc.addWall(1,1,MazeWorld.Direction.West);
			gc.removeWall(2,2,MazeWorld.Direction.East);
			//gc.fillDrop(0,0,MazeWorld.Direction.West);
			//gc.emptyDrop(0,1,MazeWorld.Direction.East);
			
			gc.addGold(1,1,MazeWorld.Direction.East);
			p.setPose(p.getX()+0.1,p.getY(),p.getTh());
			System.out.println("SentPose\n");
		    }
	    }
*/	
	    

    }
}

