package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.RobotModel;
import edu.cmu.ri.mrpl.gui.PointsConsole;
import edu.cmu.ri.mrpl.kinematics2D.Angle;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import edu.cmu.ri.mrpl.kinematics2D.Units;
import edu.cmu.ri.mrpl.maze.MazeGraphics;
import edu.cmu.ri.mrpl.maze.MazeGraphicsSwing;
import edu.cmu.ri.mrpl.maze.MazeWorld;
import fj.data.List;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

public class VisualizeController {

	public VisualizeController(){
		
	}
	/**
	 * Updates the robot's position in the world
	 * @param pc PointsConsole
	 * @param robotPose Pose of robot in world
	 */
	public synchronized void updateRobotPos(PointsConsole pc,RealPose2D robotPose){
		pc.setReference(robotPose);
	}
	/**
	 * Adds a point to pointConsole
	 * @param pc PointsConsole
	 * @param pointToRobot Point relative to Robot
	 */
	public void addPoint(PointsConsole pc, RealPoint2D pointToRobot){
		pc.addPoint(new RealPose2D(pointToRobot.getX(),pointToRobot.getY(),0.0));
	}
	public void addPoints(PointsConsole pc, List<RealPoint2D> pointsToRobot){
		for (RealPoint2D point : pointsToRobot.toCollection()){
			addPoint(pc, point);
		}
	}
	// PMF: Putting points in Robot Frame for PointsConsole
	
//	for( int i = 0; i < sonars.length; ++i){
//		Angle a = new Angle(22.5*i*Units.degToRad);
//		RealPose2D sonar = new RealPose2D(sonars[i],0.0,0.0);
//		RealPose2D sonarToRobot = new RealPose2D(0.19*Math.cos(a.angleValue()),0.19*Math.sin(a.angleValue()),a.angleValue());
//		pc.addPoint(RealPose2D.multiply(sonarToRobot, sonar));
//	}
	// PMF: End using PointsConsole 
	public void updateVisualizer(PointsConsole pc, Robot r){
		pc.drawAll(r, RobotModel.ROBOT_RADIUS);
	}

    /*
     * MazeWorld code for the 16x62 class taught at the Robotics Institute of
     * Carnegie Mellon University
     *
     * written from scratch by Martin Stolle in 2005 for the Fall 2005 class
     * revised in the Summer of 2006 for proper packaging
     *
     * inspired by code started by Illah Nourbakhsh and used for many years.
     */

    /** Viewer for .mazes that can be read into MazeWorld.  This is a stand alone
     * class.  Use it as a program or as inspiration/demo code.  Note that it uses
     * a MazeGraphicsSwing to wrap around a MazeGraphics class to make it
     * compatible with Swing.
     *
     * This program has two optional arguments.  The first is a maze to load.
     * The second is an image to create from the maze.  If the second argument is
     * provided the file will be created and the program will exit.  Supported image
     * formats: jpg, png, bmp, probably others.
     */
    public class MazeViewer extends JFrame {

        private static final long serialVersionUID = 1L;
        protected MazeWorld mw;
        protected MazeGraphicsSwing mg;

        protected JPanel mazeContainer;
        protected JTextField statusField;

        protected JFileChooser fileChooser;

        public MazeViewer(MazeGraphics mzg) {

            this.setTitle("Maze Viewer");
            this.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);

            JMenuItem loadMI = new JMenuItem("load...");
            JMenuItem quitMI = new JMenuItem("quit");

            JMenu fileMenu = new JMenu("File");
            fileMenu.add(loadMI);
            fileMenu.addSeparator();
            fileMenu.add(quitMI);

            JMenuBar menuBar = new JMenuBar();
            menuBar.add(fileMenu);

            this.setJMenuBar(menuBar);

            this.getContentPane().setLayout(new BorderLayout(10,10));

            mazeContainer = new JPanel();
            this.getContentPane().add(mazeContainer, BorderLayout.CENTER);

            statusField = new JTextField("Welcome to the MazeViewer v1.02");
            statusField.setEditable(false);
            this.getContentPane().add(statusField, BorderLayout.SOUTH);

            this.pack();
            this.setVisible(true);

//            mw = new MazeWorld(2,2);
            recreateMazeGraphics(mzg);

            fileChooser = new JFileChooser();

            javax.swing.filechooser.FileFilter mazeFilter = new javax.swing.filechooser.FileFilter() {
                public boolean accept(File file) {
                    if (file.isDirectory())
                        return true;

                    return file.getName().endsWith(".maze");
                }

                public String getDescription() {
                    return "Maze files (*.maze)";
                }
            };


            fileChooser.addChoosableFileFilter(mazeFilter);
            fileChooser.setFileFilter(mazeFilter);

            // attach listeners
            this.addWindowListener(new WindowAdapter() {
                public void windowClosing(WindowEvent e) {
                    System.exit(0);
                }
            });

            quitMI.addActionListener(new ActionListener() {
                public void actionPerformed(ActionEvent e) {
                    System.exit(0);
                }
            });

        }

        public MazeViewer() {
            this(null);
        }

        public void recreateMazeGraphics(MazeGraphics mzg) {

            mazeContainer.removeAll();
            mg = new MazeGraphicsSwing(mzg);
            mazeContainer.add(mg);

            mazeContainer.validate();
            this.validate(); // so it computes the preferred size
            this.setSize(this.getPreferredSize());
            this.validate(); // so it redraws after resizing
        }

    }

}
