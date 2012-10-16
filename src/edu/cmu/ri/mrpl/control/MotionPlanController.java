package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Path;
import edu.cmu.ri.mrpl.maze.MazePos;
import edu.cmu.ri.mrpl.maze.MazeState;
import edu.cmu.ri.mrpl.maze.MazeWorld;

import java.util.*;

/**
 * Created with IntelliJ IDEA.
 * User: WangHeli
 * Date: 10/11/12
 * Time: 2:46 PM
 * To change this template use File | Settings | File Templates.
 */
public class MotionPlanController {
    private Path outputPlan;
    private MazeWorld mazeWorld;

    public MotionPlanController(MazeWorld mazeWorld){
        this.mazeWorld = mazeWorld;
    }

    class MazeStateNode {
        MazeState mazeState;
        Path pathToState;
        String dirToState;
        MazeStateNode(MazeState mazeState, Path path, String directionsToState) {
            this.mazeState = mazeState;
            this.pathToState = path;
            dirToState = directionsToState;
        }

        int getDistance() {
            return pathToState.size();
        }

        MazeStateNode getNext(MazeWorld.Direction next) {
            MazeState nextMazeState;
            switch (next) {
                case North:
                    nextMazeState = new MazeState(mazeWorld, mazeState.x(),mazeState.y()+1, MazeWorld.Direction.North);
                    break;
                case East:
                    nextMazeState = new MazeState(mazeWorld, mazeState.x()+1,mazeState.y(), MazeWorld.Direction.East);
                    break;
                case South:
                    nextMazeState = new MazeState(mazeWorld, mazeState.x(),mazeState.y()-1, MazeWorld.Direction.South);
                    break;
                case West:
                    nextMazeState = new MazeState(mazeWorld, mazeState.x()-1,mazeState.y(), MazeWorld.Direction.West);
                    break;
                default:
                    nextMazeState = null;
            }
            Path nextPath = (Path)pathToState.clone();
            nextPath.add(Convert.MazeStateToRealPose(mazeState));

            String nextAction = "";
            if (mazeState.dir().left() == next) {
                nextAction = "L ";
            }
            if (mazeState.dir().right() == next) {
                nextAction = "R ";
            }
            if (mazeState.dir() == next) {
                nextAction = "G ";
            }
            String nextDirToState = dirToState.concat(nextAction);

            return new MazeStateNode(nextMazeState,nextPath,nextDirToState);
        }
    }

    /**
     *
     * @return
     */
    public Path searchForPath() {
        return searchForPath((MazeState)(mazeWorld.getInits().toArray())[0]);
    }

    /**
     *
     * @param initState
     * @return
     */
    public Path searchForPath(MazeState initState) {
        Set<MazePos> visitedPositions = new HashSet<MazePos>();
        Stack<MazeStateNode> nextNodes = new Stack<MazeStateNode>();
        nextNodes.push(new MazeStateNode(initState,new Path(),""));
        ArrayList <MazeStateNode> neighborsSet = new ArrayList<MazeStateNode>();
        Path resultPath = new Path();
        while (!nextNodes.empty()) {
        	MazeStateNode currentNode = nextNodes.pop();
        	if (!(mazeWorld.act(currentNode.mazeState, MazeWorld.Action.GTNN)).equals((currentNode.mazeState))){
        		neighborsSet.add(currentNode.getNext(currentNode.mazeState.dir()));
        	}
        	if (!(mazeWorld.act(currentNode.mazeState, MazeWorld.Action.TurnLeft)).equals((currentNode.mazeState))){
        		neighborsSet.add(currentNode.getNext(currentNode.mazeState.dir().left()));
        	}
        	if (!(mazeWorld.act(currentNode.mazeState, MazeWorld.Action.TurnRight)).equals((currentNode.mazeState))){
        		neighborsSet.add(currentNode.getNext(currentNode.mazeState.dir().right()));
        	}

        	for (int i = 0; i < neighborsSet.size(); i++){
        		 if (!visitedPositions.contains(neighborsSet.get(i).mazeState.pos())){
                 	visitedPositions.add(neighborsSet.get(i).mazeState.pos());
                 	if (mazeWorld.atGoal(neighborsSet.get(i).mazeState)){
                 		resultPath = neighborsSet.get(i).pathToState;
                 		System.out.println(neighborsSet.get(i).dirToState);
                 		return resultPath;
                 	}
                 	else{
                 		nextNodes.add(neighborsSet.get(i));
                 	}
                 }
        	}
        	neighborsSet.clear();
        }
        return resultPath;
    }

}
