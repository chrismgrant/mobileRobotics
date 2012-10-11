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
        MazeStateNode(MazeState mazeState, Path path) {
            this.mazeState = mazeState;
            this.pathToState = path;
        }

        int getDistance() {
            return pathToState.size();
        }
    }

    List<MazeState> getNeighbors(MazeState mazeState) {
        List<MazeState> neighbors = new ArrayList<MazeState>();
        //TODO complete

        return neighbors;
    }

    public Path searchForPath() {
        return searchForPath((MazeState)(mazeWorld.getInits().toArray())[0]);
    }
    public Path searchForPath(MazeState initState) {
        Set<MazePos> visitedPositions = new HashSet<MazePos>();
        Stack<MazeStateNode> nextNodes = new Stack<MazeStateNode>();

        while (!nextNodes.empty()) {
            //TODO complete
        }
        return null;
    }

}
