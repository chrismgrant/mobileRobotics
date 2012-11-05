package edu.cmu.ri.mrpl.control;

/**
 * Created with IntelliJ IDEA.
 * User: WangHeli
 * Date: 11/4/12
 * Time: 5:41 PM
 * To change this template use File | Settings | File Templates.
 */
public class CameraController {

    public CameraController() {
        //TODO initialize webcam here

    }

    /**
     * Checks the camera to see if robot is holding gold
     * @return whether robot is holding gold
     */
    boolean holdingGold() {
        //TODO see if robot is holding gold.
        return false;
    }

    /**
     * Checks the camera to see if gold is at target in front of robot
     * @return whether gold is in front of robot
     */
    boolean isGoldVisible() {
        //TODO see if robot sees gold in front.
        return false;
    }
}
