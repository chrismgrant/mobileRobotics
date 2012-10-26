package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Path;
import edu.cmu.ri.mrpl.maze.MazePos;
import edu.cmu.ri.mrpl.maze.MazeState;
import edu.cmu.ri.mrpl.maze.MazeWorld;

import java.util.*;
import java.util.concurrent.Callable;


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

    class MazeStateNode implements Comparable<MazeStateNode>{
        MazeState mazeState;
        Path pathToState;
        String dirToState;
        int totalCost;
        MazeStateNode(MazeState mazeState, Path path, String directionsToState) {
            this.mazeState = mazeState;
            this.pathToState = path;
            dirToState = directionsToState;
            totalCost = getDistance() + getHeuristic(mazeState);
        }

        int getDistance() {
            return dirToState.length();
        }

        public int compareTo(MazeStateNode other) {
            return Integer.valueOf(totalCost).compareTo(Integer.valueOf(other.totalCost));
        }

        MazeStateNode getNext(MazeWorld.Direction next) {
            MazeState nextMazeState;
            switch (next) {
                case North:
                    nextMazeState = new MazeState(mazeWorld, mazeState.x(),mazeState.y(), MazeWorld.Direction.North);
                    break;
                case East:
                    nextMazeState = new MazeState(mazeWorld, mazeState.x(),mazeState.y(), MazeWorld.Direction.East);
                    break;
                case South:
                    nextMazeState = new MazeState(mazeWorld, mazeState.x(),mazeState.y(), MazeWorld.Direction.South);
                    break;
                case West:
                    nextMazeState = new MazeState(mazeWorld, mazeState.x(),mazeState.y(), MazeWorld.Direction.West);
                    break;
                default:
                    nextMazeState = null;
            }
            Path nextPath = (Path)pathToState.clone();

            String nextAction = "";
            if (mazeState.dir().left() == next) {
                nextAction = "L";
            }
            if (mazeState.dir().right() == next) {
                nextAction = "R";
            }
            if (mazeState.dir() == next) {
                nextAction = "G";
                nextPath.add(Convert.MazeStateToRealPose(mazeState));
                switch (next) {
                    case North:
                        nextMazeState = new MazeState(mazeWorld, nextMazeState.x(),nextMazeState.y()+1, MazeWorld.Direction.North);
                        break;
                    case East:
                        nextMazeState = new MazeState(mazeWorld, nextMazeState.x()+1,nextMazeState.y(), MazeWorld.Direction.East);
                        break;
                    case South:
                        nextMazeState = new MazeState(mazeWorld, nextMazeState.x(),nextMazeState.y()-1, MazeWorld.Direction.South);
                        break;
                    case West:
                        nextMazeState = new MazeState(mazeWorld, nextMazeState.x()-1,nextMazeState.y(), MazeWorld.Direction.West);
                        break;
                    default:
                        nextMazeState = null;
                }
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
    private int getHeuristic(MazeState state) {
        int distance, minDistance = Integer.MAX_VALUE;
        MazeState closestGoal = state;
        for (MazeState goal : mazeWorld.getGoals()) {
            distance = Math.abs(goal.x() - state.x()) + Math.abs(goal.y() - state.y());
            if (distance < minDistance) {
                minDistance = distance;
                closestGoal = goal;
            }
        }
        return minDistance + getTurnDistance(state, closestGoal);
    }
    private int getTurnDistance(MazeState state, MazeState other) {
        if (state.dir() == other.dir()) return 0;
        if (state.dir() == other.dir().rear()) return 2;
        return 1;
    }
    /**
     *
     * @param initState
     * @return
     */
    public Path searchForPath(MazeState initState) {
        //TODO overhaul for A*
        int debugCount = 0;

        Set<MazeState> visitedPositions = new HashSet<MazeState>();
        Queue<MazeStateNode> nextNodes = new LinkedList<MazeStateNode>();
        nextNodes.offer(new MazeStateNode(initState,new Path(),""));
        ArrayList <MazeStateNode> neighborsSet = new ArrayList<MazeStateNode>();
        Path resultPath = new Path();
        while (!nextNodes.isEmpty()) {
        	MazeStateNode currentNode = nextNodes.remove();
            debugCount++;
        	if (!(mazeWorld.act(currentNode.mazeState, MazeWorld.Action.GTNN)).equals((currentNode.mazeState))){
        		neighborsSet.add(currentNode.getNext(currentNode.mazeState.dir()));
        	}
            neighborsSet.add(currentNode.getNext(currentNode.mazeState.dir().left()));
            neighborsSet.add(currentNode.getNext(currentNode.mazeState.dir().right()));

        	for (int i = 0; i < neighborsSet.size(); i++){
                if (!visitedPositions.contains(neighborsSet.get(i).mazeState)){
                    visitedPositions.add(neighborsSet.get(i).mazeState);
                    MazeState front, left, right, rear;
                    front = neighborsSet.get(i).mazeState;
                    left = new MazeState(front.x(),front.y(),front.dir().left());
                    right = new MazeState(front.x(), front.y(), front.dir().right());
                    rear = new MazeState(front.x(), front.y(), front.dir().rear());

                    if (mazeWorld.atGoal(front)){
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
//                        resultPath.addAll(searchForPath(front));
                        System.out.printf("Expanded %d nodes.\n", debugCount);
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

    public Path searchForAStarPath(MazeState initState) {
        int debugCount = 0;

        Set<MazeState> visitedPositions = new HashSet<MazeState>();
        PriorityQueue<MazeStateNode> nextNodes = new PriorityQueue<MazeStateNode>();
        nextNodes.offer(new MazeStateNode(initState,new Path(),""));
        ArrayList <MazeStateNode> neighborsSet = new ArrayList<MazeStateNode>();
        Path resultPath = new Path();
        while (!nextNodes.isEmpty()) {
            MazeStateNode currentNode = nextNodes.poll();
            debugCount++;
            if (!(mazeWorld.act(currentNode.mazeState, MazeWorld.Action.GTNN)).equals((currentNode.mazeState))){
                neighborsSet.add(currentNode.getNext(currentNode.mazeState.dir()));
            }
            neighborsSet.add(currentNode.getNext(currentNode.mazeState.dir().left()));
            neighborsSet.add(currentNode.getNext(currentNode.mazeState.dir().right()));

            for (int i = 0; i < neighborsSet.size(); i++){
                if (!visitedPositions.contains(neighborsSet.get(i).mazeState)){
                    visitedPositions.add(neighborsSet.get(i).mazeState);
                    MazeState front, left, right, rear;
                    front = neighborsSet.get(i).mazeState;
                    left = new MazeState(front.x(),front.y(),front.dir().left());
                    right = new MazeState(front.x(), front.y(), front.dir().right());
                    rear = new MazeState(front.x(), front.y(), front.dir().rear());

                    if (mazeWorld.atGoal(front)){
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
//                        resultPath.addAll(searchForPath(front));
                        System.out.printf("Expanded %d nodes.\n", debugCount);
                        return resultPath;

                    } else {
                        nextNodes.offer(neighborsSet.get(i));
                    }
                }
            }
            neighborsSet.clear();
        }
        System.out.println("NoPathFound");
        return resultPath;
    }

}
