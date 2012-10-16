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
        Set<MazeState> nextStates = new HashSet<MazeState>();
        nextNodes.push(new MazeStateNode(initState,new Path(),""));
        Set <MazeState> neighborsSet = new HashSet<MazeState>();
        while (!nextNodes.empty()) {
        	MazeStateNode currentNode = nextNodes.pop();
            if (!visitedPositions.contains(currentNode.mazeState.pos())){
            	visitedPositions.add(currentNode.mazeState.pos());
            	nextStates.add(currentNode.mazeState);
            	//need something indicative of a goal state to conduct check
            }
        }
        
        return null;
    }

}
