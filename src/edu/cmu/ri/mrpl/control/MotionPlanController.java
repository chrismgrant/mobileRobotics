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
        this.mazeWorld = new MazeWorld(mazeWorld);
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
        Queue<MazeStateNode> nextNodes = new LinkedList<MazeStateNode>();
        nextNodes.offer(new MazeStateNode(initState,new Path(),""));
        ArrayList <MazeStateNode> neighborsSet = new ArrayList<MazeStateNode>();
        Path resultPath = new Path();
        while (!nextNodes.isEmpty()) {
        	MazeStateNode currentNode = nextNodes.remove();
        	if (!(mazeWorld.act(currentNode.mazeState, MazeWorld.Action.GTNN)).equals((currentNode.mazeState))){
        		neighborsSet.add(currentNode.getNext(currentNode.mazeState.dir()));
        	}
        	MazeState turn = mazeWorld.act(currentNode.mazeState, MazeWorld.Action.TurnLeft);
        	if (!(mazeWorld.act(turn, MazeWorld.Action.GTNN)).equals((turn))){
        		neighborsSet.add(currentNode.getNext(currentNode.mazeState.dir().left()));
        	}
        	turn = mazeWorld.act(currentNode.mazeState, MazeWorld.Action.TurnRight);

        	if (!(mazeWorld.act(turn, MazeWorld.Action.GTNN)).equals((turn))){
        		neighborsSet.add(currentNode.getNext(currentNode.mazeState.dir().right()));
        	}

        	for (int i = 0; i < neighborsSet.size(); i++){
                if (!visitedPositions.contains(neighborsSet.get(i).mazeState.pos())){
                    visitedPositions.add(neighborsSet.get(i).mazeState.pos());
                    MazeState front, left, right, rear;
                    front = neighborsSet.get(i).mazeState;
                    left = new MazeState(front.x(),front.y(),front.dir().left());
                    right = new MazeState(front.x(), front.y(), front.dir().right());
                    rear = new MazeState(front.x(), front.y(), front.dir().rear());

                    if (mazeWorld.atGoal(front) || mazeWorld.atGoal(left) || mazeWorld.atGoal(right) || mazeWorld.atGoal(rear)){
                        resultPath = neighborsSet.get(i).pathToState;
                        System.out.println("Found: "+neighborsSet.get(i).dirToState);
                        MazeState goalState = null;
                        if (mazeWorld.atGoal(front)) goalState = front;
                        if (mazeWorld.atGoal(rear)) goalState = rear;
                        if (mazeWorld.atGoal(left)) goalState = left;
                        if (mazeWorld.atGoal(right)) goalState = right;
                        resultPath.add(Convert.MazeStateToRealPose(goalState));
                        mazeWorld.removeGoal(front);
                        mazeWorld.removeGoal(rear);
                        mazeWorld.removeGoal(left);
                        mazeWorld.removeGoal(right);
                        resultPath.addAll(searchForPath(front));
                        return resultPath;

                    } else {
                        nextNodes.add(neighborsSet.get(i));
                    }
                }
        	}
        	neighborsSet.clear();
        }
        System.out.println("NoPathFound");
        return resultPath;
    }

}
