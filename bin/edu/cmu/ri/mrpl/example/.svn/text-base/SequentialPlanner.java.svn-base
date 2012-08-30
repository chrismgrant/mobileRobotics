package edu.cmu.ri.mrpl.example;

import java.io.IOException;
import java.util.*;
import javax.swing.*;

import edu.cmu.ri.mrpl.maze.*;
import edu.cmu.ri.mrpl.maze.MazeWorld.Action;

/** Here's a depth-first itterative deepening sequential planner. It ignores percepts, but tries to find a
   solution to a MazeWorld problem using a sequence of actions, despite
   any robot position uncertainty.
   No optimizations (purning, cycle busting, etc) are included. */
public class SequentialPlanner {   

  private final MazeWorld maze;
  private final int numActions;

  /** Creates a sequential planner with the given MazeWorld. */
  public SequentialPlanner(MazeWorld maze) {
    this.maze = maze;
    numActions = maze.numActions();
  }

  /** Recurseive function which computes the a plan of at most maxDepth steps.*/
  private List<Action> seqPlanHelper(Set<MazeState> curStates,
      int curDepth, int maxDepth) {

    if (maze.allAtGoal(curStates))
      return new ArrayList<Action>(curDepth); /* Done! null plan, param = presize List (for performance) */

    if (curDepth == maxDepth)
      return null; /* failure */

    for (int i=0; i < numActions; i++) {
      List<Action> returnPlan = seqPlanHelper(maze.acts(curStates, Action.values()[i]), curDepth+1, maxDepth);
      if (returnPlan != null) {
	returnPlan.add(Action.values()[i]);
	return returnPlan;
      }
    } /* end for */
    return null;  
  }

  /** This is the top-level function you should call 
     this does depth-first iterative deepening.  It will return a list of actions if it succeds or null if it fails */
  public List<Action> plan(int maxDepth) {

    for (int i=0; i <= maxDepth; i++) {
      System.out.println("Searching depth = " + i);

      List<Action> returnPlan = seqPlanHelper(maze.getInits(), 0, i);
      if (returnPlan != null) {
    	  Collections.reverse(returnPlan);    	  
    	  return returnPlan;
      }
    } /* end for */
    return null;
  }

  /** This program will create a maze and attempt to find a sequential plan to solve it.
   * It will then display the solution using MazeGraphics.
   * It takes two optional arguments (if one argument is provided the second is required).
   *   MazeFile.maze maxDepth
   */
  public static void main(String[] args) {
    MazeWorld world = null;
    int maxDepth = 0;

    if(args.length == 0){
        //If no arguments are provided use a default maze.

        /* this is just an empty 2x2 maze with one horizontal wall (parallel to the
           X-axis) between the bottom left node and the top left node. 
           Solution is of length 8 (11010101) LLGLGLGL
        */
    	world = new MazeWorld(2, 2);

    	world.addWall(0, 0, MazeWorld.Direction.North);

    	world.addInit(0, 0, MazeWorld.Direction.West);
    	world.addInit(1, 0, MazeWorld.Direction.West);

    	world.addGoal(0, 1, MazeWorld.Direction.South);
    	world.addGoal(0, 1, MazeWorld.Direction.East);
    	maxDepth = 9;
    }
    else if(args.length == 2){
    	//If arguments are provided load the maze.
    	try {
			world = new MazeWorld(args[0]);
		} catch (IOException e) {
			e.printStackTrace();
			System.exit(-1);
		}
    	maxDepth = Integer.valueOf(args[1]);
    }
    else{
    	System.err.println("Usage - java SequentialPlanner [mazeFile maxDepth]\n");
    	System.err.println("Both arguments are optional, but if one is used the other is required.\n");
    	System.exit(-1);
    }

    //Run the sequential planner.
    SequentialPlanner testPlanner = new SequentialPlanner(world);
    List<Action> thePlan;
    thePlan = testPlanner.plan(maxDepth);

    //Display solution if found.
    if (thePlan == null)
      System.out.println("failed to find plan in " + maxDepth + " steps!");
    else {
      System.out.println("the following plan was found:");
      int planSize = thePlan.size();
      
      //Get a copy of the initial positions, for display purposes.
	  Set<MazeState> ss = new LinkedHashSet<MazeState>(world.getInits());
	  world.removeAllInits(); //Just for display
	  
	  //Create a MazeGraphics object and install it in a JFrame.
	  MazeGraphics mg = new MazeGraphics(world);
	  JFrame jf = new JFrame();
	  jf.add(mg);
	  jf.pack();
	  jf.setVisible(true);

      for (int i=0; i < planSize; i++){
    	  System.out.println("Action " + i + ": " + thePlan.get(i));    	  

    	  //For each step of the plan transform the state set (initially the inits) by the action.
    	  ss = world.acts(ss, thePlan.get(i));
    	  
    	  //Next update the mazeGraphics and repaint.
    	  mg.setMazeStateSet(ss);
    	  mg.repaint();

    	  //Sleep so we can watch the display.
    	  try {
    		  Thread.sleep(1000);
    	  } catch (InterruptedException e) {
    		  e.printStackTrace();
    	  }
      }
    }
    
    System.exit(0);
  }
}

