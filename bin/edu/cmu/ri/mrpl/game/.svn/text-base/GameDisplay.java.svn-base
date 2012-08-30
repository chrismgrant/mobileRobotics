package edu.cmu.ri.mrpl.game;


import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.io.*;
import java.util.*;

import edu.cmu.ri.mrpl.maze.*;
import edu.cmu.ri.mrpl.kinematics2D.*;

public class GameDisplay extends JFrame
{
  private static final long serialVersionUID = 1L;

  protected MazeWorld mw;
  protected MazeWorld orig_mw;
  protected MazeGraphics mg;

  protected JTextField statusField;

  protected String team_name;
  protected String[] robot_names;

  protected RobotData robotsData[];
  protected java.util.List<MazeGraphics.ContRobot> robotPoses;

  protected static final Color robotColorOptions[] = {
    Color.RED,
    Color.GREEN,
    Color.BLUE,
    Color.ORANGE,
    Color.MAGENTA,
    Color.CYAN
  };

  protected class RobotData
  {
    public Color pos_color;
    public Color goal_color;

    public RealPose2D pose;
    public RealPose2D goal_pose;
  };

  public GameDisplay(MazeWorld world, String team_name, String[] robot_names)
  {
    if (world == null) {
      System.err.println("Must specify valid mazeworld for GameDispaly");
      System.exit(-1);
    }
    orig_mw = new MazeWorld(world);
    mw = new MazeWorld(world);

    this.team_name = team_name;
    if (this.team_name == null)
      this.team_name = "(unspecified)";

    this.robot_names = robot_names;

    for (int i = 0; i < robot_names.length; i++) {
      if (this.robot_names[i] == null)
        this.robot_names[i] = "(unspecified)";
    }

    robotsData = new RobotData[robot_names.length];
    robotPoses = Collections.synchronizedList(new ArrayList<MazeGraphics.ContRobot>());

    for (int i = 0; i < robot_names.length; i++) {
      robotsData[i] = new RobotData();
      robotsData[i].pos_color = robotColorOptions[i%robotColorOptions.length];
      robotsData[i].goal_color = new Color(robotsData[i].pos_color.getRed()/255.0f, robotsData[i].pos_color.getGreen()/255.0f, robotsData[i].pos_color.getBlue()/255.0f, 0.3f);
      robotsData[i].pose = new RealPose2D(-1, -1, 0);
      robotPoses.add(new MazeGraphics.ContRobot(robotsData[i].pose, robotsData[i].pos_color));
      robotsData[i].goal_pose = new RealPose2D(-1, -1, 0);
      robotPoses.add(new MazeGraphics.ContRobot(robotsData[i].goal_pose, robotsData[i].goal_color));
    }

    clearMaze();

    setupWindow();
    
    mg.setContRobots(robotPoses);

    setVisible(true);

    // attach listeners
    addWindowListener(new WindowAdapter() {
            public void windowClosing(WindowEvent e) {
                    System.exit(0);
            }
    });
  }

  public void addWall(int cell_x, int cell_y, MazeWorld.Direction dir)
  {
    synchronized(this) {
      if (!mw.isValid(cell_x, cell_y)) {
        System.err.printf("GameDisplay: addWall: invalid cell (%d,%d) specified\n", cell_x, cell_y);
	return;
      }
      mw.addWall(cell_x, cell_y, dir);
      repaint();
    }
  }

  public void removeWall(int cell_x, int cell_y, MazeWorld.Direction dir)
  {
    synchronized(this) {
      if (!mw.isValid(cell_x, cell_y)) {
        System.err.printf("GameDisplay: removeWall: invalid cell (%d,%d) specified\n", cell_x, cell_y);
	return;
      }
      mw.removeWall(cell_x, cell_y, dir);
      repaint();
    }
  }

  public void addGold(int cell_x, int cell_y, MazeWorld.Direction dir)
  {
    synchronized(this) {
      if (!mw.isValid(cell_x, cell_y)) {
        System.err.printf("GameDisplay: addGold: invalid cell (%d,%d) specified\n", cell_x, cell_y);
	return;
      }
      mw.addGold(new MazeState(cell_x, cell_y, dir));
      repaint();
    }
  }

  public void removeGold(int cell_x, int cell_y, MazeWorld.Direction dir)
  {
    synchronized(this) {
      if (!mw.isValid(cell_x, cell_y)) {
        System.err.printf("GameDisplay: removeGold: invalid cell (%d,%d) specified\n", cell_x, cell_y);
	return;
      }
      mw.removeGold(new MazeState(cell_x, cell_y, dir));
      repaint();
    }
  }

  public void fillDrop(int cell_x, int cell_y, MazeWorld.Direction dir)
  {
    synchronized(this) {
      if (!mw.isValid(cell_x, cell_y)) {
        System.err.printf("GameDisplay: fillDrop: invalid cell (%d,%d) specified\n", cell_x, cell_y);
	return;
      }
      mw.addGold(new MazeState(cell_x, cell_y, dir));
      repaint();
    }
  }

  public void emptyDrop(int cell_x, int cell_y, MazeWorld.Direction dir)
  {
    synchronized(this) {
      if (!mw.isValid(cell_x, cell_y)) {
        System.err.printf("GameDisplay: emptyDrop: invalid cell (%d,%d) specified\n", cell_x, cell_y);
	return;
      }
      mw.removeGoldDropped(new MazeState(cell_x, cell_y, dir));
      repaint();
    }
  }

  public void setPose(int robot, RealPose2D pose)
  {
    if (robot < 0 || robot >= robotsData.length) {
      System.err.println("Cannot set pose for bogus robot #" + robot);
      return;
    }

    synchronized(this) {
      synchronized(robotPoses) {
        robotPoses.get(2*robot).pose.setPose(pose.getX(), pose.getY(), pose.getTh());
        repaint();
      }
    }
  }

  public void setGoal(int robot, RealPose2D pose)
  {
    if (robot < 0 || robot >= robotsData.length) {
      System.err.println("Cannot set goal for bogus robot #" + robot);
      return;
    }

    synchronized(this) {
      synchronized(robotPoses) {
        robotPoses.get(2*robot+1).pose.setPose(pose.getX(), pose.getY(), pose.getTh());
        repaint();
      }
    }
  }

  public void reset()
  {
    synchronized(this) {
      mw.copyFrom(orig_mw);
    }
  }

  protected void clearMaze()
  {
    mw.removeAllInits();
    mw.removeAllGoals();
  }

  protected void setupWindow() {
    Container pane = getContentPane();

    pane.setBackground(Color.WHITE);

    setTitle("Game Display");
    setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);

    getContentPane().setLayout(new GridBagLayout());

    GridBagConstraints c = new GridBagConstraints();
    c.weightx = 1;

    JLabel label;
    JComponent comp;

    //title
    comp = label = new JLabel("Team " + team_name);
    Font f = label.getFont();
    label.setFont(new Font(f.getName(), f.getStyle(), f.getSize()+8));
    label.setHorizontalAlignment(SwingConstants.CENTER);

//    comp.setBorder(new LineBorder(Color.BLUE));

    c.fill = GridBagConstraints.HORIZONTAL;
    c.anchor = GridBagConstraints.CENTER;
    c.gridx = 0;
    c.gridy = 0;
    c.insets = new Insets(20, 0, 20, 0);
    pane.add(comp, c);

    //robot names
    JPanel robotNamesPanel = new JPanel();
    robotNamesPanel.setBackground(Color.WHITE);
    robotNamesPanel.setLayout(new GridLayout(0, 3, 10, 0));
    for (int i = 0; i < robot_names.length; i++) {
      comp = label = new JLabel("Robot #" + (i+1) + ": ");
      label.setHorizontalAlignment(SwingConstants.RIGHT);
      robotNamesPanel.add(label);

      comp = label = new JLabel(robot_names[i]);
      comp.setForeground(robotsData[i].pos_color);
      label.setHorizontalAlignment(SwingConstants.CENTER);
      robotNamesPanel.add(label);

      comp = label = new JLabel("(goal)");
      comp.setForeground(robotsData[i].goal_color);
      label.setHorizontalAlignment(SwingConstants.LEFT);
      robotNamesPanel.add(label);
    }
    c.fill = GridBagConstraints.NONE;
    c.anchor = GridBagConstraints.CENTER;
    c.gridx = 0;
    c.gridy = 1;
    c.ipady = 0;
    c.insets = new Insets(0, 5, 0, 5);
//    robotNamesPanel.setBorder(new LineBorder(Color.BLUE));
    pane.add(robotNamesPanel, c);

    //maze world
    JPanel mgp = new JPanel();
    mgp.setLayout(new GridLayout(1,1));
    mg = new MazeGraphics(mw);
    comp = new MazeGraphicsSwing(mg);
//    comp.setAlignmentX(CENTER_ALIGNMENT);
//    comp.setBorder(new LineBorder(Color.GREEN, 1));
//    comp.validate();
//    comp.setMaximumSize(comp.getPreferredSize());
    mgp.add(comp);
//    mgp.setMaximumSize(comp.getPreferredSize());
    c.fill = GridBagConstraints.NONE;
    c.anchor = GridBagConstraints.CENTER;
    c.gridx = 0;
    c.gridy = 2;
    c.ipady = 0;
    c.insets = new Insets(10, 0, 10, 0);
//    mgp.setBorder(new LineBorder(Color.BLUE));
    pane.add(mgp, c);

//    comp = statusField = new JTextField("Started game display");
//    statusField.setEditable(false);
//   c.fill = GridBagConstraints.HORIZONTAL;
//    c.anchor = GridBagConstraints.CENTER;
//    c.gridx = 0;
//    c.gridy = 3;
//    c.ipady = 0;
//    c.insets = new Insets(0, 0, 0, 0);
//   pane.add(comp, c);

    pack();

    setPreferredSize(getPreferredSize());
    validate();

    pack();
  }

  public void paint(Graphics g)
  {
    synchronized(this) {
      super.paint(g);
    }
  }

  public void paintComponents(Graphics g)
  {
    synchronized(this) {
      super.paintComponents(g);
    }
  }

  public static void main(String args[])
  {
    SwingUtilities.invokeLater(new Runnable() {
      public void run() {
        String robots[] = {"Bot1","Bot2"};

        try {
          GameDisplay gd = new GameDisplay(new MazeWorld(new File("test.maze")), "MyTeam", robots);

          gd.setPose(0, new RealPose2D(0, 0, 3));
          gd.setGoal(0, new RealPose2D(2, 3, 1));

          gd.setPose(1, new RealPose2D(1, 0, 2));
          gd.setGoal(1, new RealPose2D(3, 1, 0));

          gd.fillDrop(0, 1, MazeWorld.Direction.West);

          Thread.sleep(300);
        }
        catch (IOException e) {
        }
        catch (InterruptedException e) {
        }
      }
    });

  }

}


