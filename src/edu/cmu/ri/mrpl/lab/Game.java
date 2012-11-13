package edu.cmu.ri.mrpl.lab;

/**
 * Created with IntelliJ IDEA.
 * User: WangHeli
 * Date: 11/11/12
 * Time: 10:09 PM
 * To change this template use File | Settings | File Templates.
 */

import edu.cmu.ri.mrpl.*;
import edu.cmu.ri.mrpl.Robot;
import edu.cmu.ri.mrpl.control.*;
import edu.cmu.ri.mrpl.gui.PointsConsole;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class Game extends JFrame implements ActionListener, TaskController {
    public static final boolean SIM = true;
    public static final String robotID = "Duct Tape";
//    public static final String robotID = "WD-40";
    public static final String knownMazeFile = "game.maze";
    public static final String realMazeFile = "game.maze";
    public boolean[] preChecks;
    public String command;
    public Robot robot;
    private SonarConsole sc;
    private SonarConsole sc2;
    public PointsConsole pc; // Added for displaying points. PMF
    private JFrame scFrame;
    private JFrame sc2Frame;

    private JButton initSoundButton;
    private JButton initCamButton;
    private JButton connectButton;
    private JButton disconnectButton;

    private JButton initSonarButton;
    private JButton initCommButton;
    private JButton initMazeButton;

    private JButton testButton;
    private JButton playButton;

    private JButton stopButton;
    private JButton quitButton;
    private JButton commandButton;

    private JTextField commandText;

    private RotateTask programTask;
    private ForwardTask sonarTask;
    private PlayTask playTask;

    private Task curTask = null;

    static final int DEFAULT_ROOM_SIZE = 4;

    CommandController cc;


    public Game() {
        super("Game");

        preChecks = new boolean[7];

        initSoundButton = new JButton("Test Sound");
        initCamButton = new JButton("Test Camera");
        connectButton = new JButton("connect");
        disconnectButton = new JButton("disconnect");
        initSonarButton = new JButton("Start sonar");
        initCommButton = new JButton("Start comms");
        initMazeButton = new JButton("Load maze");
        testButton = new JButton("Final check the styrofoam");
        playButton = new JButton("Fight with Styrofoam!");
        stopButton = new JButton(">> stop <<");
        quitButton = new JButton(">> quit <<");

        commandButton = new JButton("commandButton");
        commandText = new JTextField("commandText");

        initSoundButton.addActionListener(this);
        initCamButton.addActionListener(this);
        connectButton.addActionListener(this);
        disconnectButton.addActionListener(this);
        initSonarButton.addActionListener(this);
        initCamButton.addActionListener(this);

        testButton.addActionListener(this);
        playButton.addActionListener(this);
        stopButton.addActionListener(this);
        quitButton.addActionListener(this);

        commandButton.addActionListener(this);

        Container main = getContentPane();
        main.setLayout(new BoxLayout(main, BoxLayout.Y_AXIS));
        Box box;

        main.add(Box.createVerticalStrut(30));

        box = Box.createHorizontalBox();
        main.add(box);
//        box.add(Box.createHorizontalStrut(15));
        box.add(initSoundButton);
        box.add(Box.createHorizontalStrut(15));
        box.add(initCamButton);
//        box.add(Box.createHorizontalStrut(15));

        main.add(Box.createVerticalStrut(15));

        box = Box.createHorizontalBox();
        main.add(box);
//        box.add(Box.createHorizontalStrut(15));
        box.add(connectButton);
        box.add(Box.createHorizontalStrut(15));
        box.add(disconnectButton);
//        box.add(Box.createHorizontalStrut(15));

        main.add(Box.createVerticalStrut(15));

        box = Box.createHorizontalBox();
        main.add(box);
//        box.add(Box.createHorizontalStrut(15));
        box.add(initSonarButton);
        box.add(Box.createHorizontalStrut(15));
        box.add(initCommButton);
//        box.add(Box.createHorizontalStrut(15));

        main.add(Box.createVerticalStrut(15));

        box = Box.createHorizontalBox();
        main.add(box);
        box.add(testButton);

        main.add(Box.createVerticalStrut(30));

        box = Box.createHorizontalBox();
        main.add(box);
        box.add(playButton);

        main.add(Box.createVerticalStrut(30));

        box = Box.createHorizontalBox();
        main.add(box);
        box.add(Box.createHorizontalStrut(30));
        box.add(commandButton);
        box.add(Box.createHorizontalStrut(30));
        box.add(commandText);
        box.add(Box.createHorizontalStrut(30));

        main.add(Box.createVerticalStrut(30));

        box = Box.createHorizontalBox();
        main.add(box);
        box.add(stopButton);
        box.add(Box.createHorizontalStrut(30));
        box.add(quitButton);

        main.add(Box.createVerticalStrut(30));


        this.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
        this.addWindowListener(new WindowAdapter() {
            public void windowClosing(WindowEvent e) {
                quit();
            }
        });


        this.setLocationByPlatform(true);
        this.pack();
        this.setVisible(true);

        // construct the SonarConsole, but don't make it visible
        scFrame = new JFrame("Sonar Console");
        scFrame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
        sc = new SonarConsole();
        scFrame.add(sc);
        scFrame.pack();

        sc2Frame = new JFrame("Live Sonar Console");
        sc2Frame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
        sc2 = new SonarConsole();
        sc2Frame.add(sc2);
        sc2Frame.pack();

        // PMF: Creating a new points console for displaying robot-relative points.
        // takes arguments( top-left-x, top-left-y, window-width,window-height)
        // next set the bounds of the viewport.
        pc = new PointsConsole(768-640, 1024-640, 640, 640);
        pc.setWorldViewport(-1.0 * DEFAULT_ROOM_SIZE, -1.0 * DEFAULT_ROOM_SIZE, 1.0 * DEFAULT_ROOM_SIZE - 0.5, 1.0 * DEFAULT_ROOM_SIZE - 0.5);
        // PMF: End new code for Points console

        // PMF: Creating a frame to put the PointsConsole panel in.
        // get the panel that
        Frame f = new Frame();
        f.setTitle("Points Console");
        f.add(pc.getPanel());
        f.setSize(pc.getPanel().getSize());
        f.setVisible(true);
        // PMF: end of displaying PointsConsole

        // construct tasks
        programTask = new RotateTask(this);
        sonarTask = new ForwardTask(this);
        playTask = new PlayTask(this);


        if (SIM) {
            robot=new SimRobot();
            ((SimRobot) robot).loadMaze(realMazeFile);
        } else {
            robot=new ScoutRobot();
        }
    }

    // call from GUI thread
    public void connect() {
        try {
            robot.connect();
        } catch(IOException ioex) {
            System.err.println("couldn't connect to robot:");
            ioex.printStackTrace();
        }
    }

    // call from GUI thread
    public void disconnect() {
        try {
            robot.disconnect();
        } catch(IOException ioex) {
            System.err.println("couldn't disconnect from robot:");
            ioex.printStackTrace();
        }
    }


    // call from GUI thread
    public synchronized void stop() {
        if (curTask!=null)
            curTask.pleaseStop();
    }

    // call from GUI thread
    public void quit() {

        synchronized(this) {
            if (curTask!=null) {
                curTask.pleaseStop();

                try {
                    this.wait(1000);
                } catch(InterruptedException iex) {
                    System.err.println("interrupted while waiting for worker termination signal");
                }
            }

            if (curTask!=null) {
                System.err.println("exiting even though robot is running!");
            } else {
                disconnect();
            }
        }

        System.exit(0);
    }

    // invoked from non-GUI thread to hide/show SonarConsole
    public void showSC() {
        SwingUtilities.invokeLater(new Runnable() {
            public void run() {
                scFrame.setLocationByPlatform(true);
                scFrame.setVisible(true);
                sc2Frame.setLocationByPlatform(true);
                sc2Frame.setVisible(true);
            }
        });
    }

    // invoked from non-GUI thread to hide/show SonarConsole
    public void hideSC() {
        SwingUtilities.invokeLater(new Runnable() {
            public void run() {
                scFrame.setVisible(false);
                sc2Frame.setVisible(false);
            }
        });
    }

    public void actionPerformed(ActionEvent e) {
        Object source = e.getSource();
        if (source==initSoundButton) {
        	cc.playSound(robotID);
        	preChecks[1] = true;

        } else if (source==initCamButton) {

        } else if ( source==connectButton) {
            connect();
        } else if ( source==disconnectButton ) {
            disconnect();
        } else if (source==initSonarButton) {

        } else if (source==initCommButton) {

        } else if (source== initMazeButton) {

        } else if (source==testButton) {

        } else if ( source==stopButton ) {
            stop();
        } else if ( source==quitButton ) {
            quit();
        } else if ( source==playButton ) {
            if (ready()) {
                (new Thread(playTask)).start();
            }
        } else if ( source==commandButton) {
            command = commandText.getText();
            FileWriter commandFile;
            try {
                commandFile = new FileWriter("command.txt");
                PrintWriter outCommand = new PrintWriter(commandFile);
                outCommand.print(command);
                commandFile.close();
            } catch (IOException e1) {

                e1.printStackTrace();
            }
            preChecks[7]=true;

        }
    }

    private boolean ready() {
        for (int i = 0;;){}
    }

    public synchronized boolean canStart(Task t) {
        if (curTask!=null)
            return false;

        curTask = t;
        return true;
    }

    public synchronized void finished(Task t) {
        if (curTask!=t) {
            System.err.println("ignoring finished() from unknown task "+t+"!");
            return;
        }
        curTask=null;
        this.notifyAll();
    }

    public static void main(String[] argv) {
        javax.swing.SwingUtilities.invokeLater(new Runnable() {
            public void run() {
                new Game();
            }
        });
    }


    // here we implement the different robot controllers
    // as Tasks
    //

    class RotateTask extends Task {
        public WheelController wc;
        public SonarController soc;
        public BumperController bc;
        public TrackerController trc;
        public BearingController bac;
        public CommandController cc;

        RotateTask(TaskController tc) {
            super(tc);
//			wc = new WheelController();
//			soc = new SonarController();
//			bc = new BumperController();
//			bac = new BearingController();
        }

        public void taskRun() {
            showSC();
            robot.turnSonarsOn();

            double[] sonars = new double[16];
            double direction = 0;
            int counter = 20;
            wc.setALVel(0, 0);
            try{
                FileWriter outFileRobPos = new FileWriter("RobPosData.csv");
                FileWriter outFileCalcPos = new FileWriter("CalcPosData.csv");
                FileWriter outFileRobVel = new FileWriter("RobVelData.csv");
                FileWriter outFileCtrVel = new FileWriter("CtrVelData.csv");
                FileWriter outFileData = new FileWriter("Data.csv");

                PrintWriter outRobPos = new PrintWriter(outFileRobPos);
                PrintWriter outCalcPos = new PrintWriter(outFileCalcPos);
                PrintWriter outRobVel = new PrintWriter(outFileRobVel);
                PrintWriter outCtrVel =  new PrintWriter(outFileCtrVel);
                PrintWriter outData = new PrintWriter(outFileData);

                outData.println("Bearring, Robot, Velocity, Comand");

                double robotDistance = 0;
                while(!shouldStop()) {
                    robot.updateState();
                    robot.getSonars(sonars);
                    soc.updateSonars(sonars);
                    sc.setSonars(soc.getSonarReadings());
                    sc2.setSonars(sonars);
                    bac.updateBearing(wc.getRobLVel(robot), wc.getRobAVel(robot));
                    outCalcPos.print(bac.getX()+",");
                    outRobPos.print(bac.getRPose(robot).getX()+",");
                    outRobVel.print(wc.getRobLVel(robot)+",");
                    robotDistance = Math.hypot(robot.getPosX(),robot.getPosY());
                    outData.print(bac.getX()+","+robotDistance+","+wc.getRobLVel(robot)+",");
                    if (counter >= 0){
                        wc.setALVel(0, 1);
                        outCtrVel.print("1,");
                        outData.println("1");
                        counter--;
                    } else {
                        wc.setALVel(0, 0);
                        outCtrVel.print("0,");
                        outData.println("0");
                    }
                    wc.updateWheels(robot, bc.isBumped(robot));
                    try {
                        Thread.sleep(50);
                    } catch(InterruptedException iex) {
                        System.out.println("sample program sleep interrupted");
                    }
                }
                outRobPos.close();
                outCalcPos.close();
                outRobVel.close();
                outCtrVel.close();
                outData.close();
            } catch (IOException e){
                e.printStackTrace();
            }
            robot.turnSonarsOff();
            robot.setVel(0,0);
            hideSC();
        }

        public String toString() {
            return "sample program";
        }
    }

    class ForwardTask extends Task {
        public WheelController wc;
        public SonarController soc;
        public BumperController bc;
        public TrackerController trc;
        public BearingController bac;
        public CommandController cc;

        ForwardTask(TaskController tc) {
            super(tc);
//			wc = new WheelController();
//			soc = new SonarController();
//			bc = new BumperController();
//			bac = new BearingController();
        }

        public void taskRun() {
            showSC();
            robot.turnSonarsOn();
            wc.setALVel(0,0);
            double[] sonars = new double[16];
            double speed = 0, target = 0;

            try{
                FileWriter outFileRawSonar = new FileWriter("FwdRawSonarData");
                FileWriter outFileFiltSonar = new FileWriter("FwdFiltSonarData");
                PrintWriter outRawSonar = new PrintWriter(outFileRawSonar);
                PrintWriter outFiltSonar = new PrintWriter(outFileFiltSonar);
                while(!shouldStop()) {
                    robot.updateState();
                    robot.getSonars(sonars);
                    soc.updateSonars(sonars);
                    bac.updateBearing(wc.getLVel(), wc.getAVel());
                    speed = soc.getForwardSonarReading() - .5;
                    target = (speed > .5) ? .5 : ((Math.abs(speed)<=.03)? 0 : speed);
                    for(int i =0 ; i < 16; i ++){
                        outRawSonar.print(sonars[i]+",");
                        outRawSonar.println(";");
                    }
                    for(int i =0 ; i < 16; i ++){
                        outFiltSonar.print(soc.getSonarReadings()[i]+",");
                        outFiltSonar.println(";");
                    }
                    wc.setALVel(0, target);
                    wc.updateWheels(robot, bc.isBumped(robot));
                    sc.setSonars(soc.getSonarReadings());
                    sc2.setSonars(sonars);
                    try {
                        Thread.sleep(50);
                    } catch(InterruptedException iex) {
                        System.out.println("Sonar sleep interrupted");
                    }
                }
                outRawSonar.close();
                outFiltSonar.close();
            } catch (IOException e){
                e.printStackTrace();
            }
            robot.turnSonarsOff();
            hideSC();
        }

        public String toString() {
            return "sonar loop";
        }
    }

    class PlayTask extends Task {


        PlayTask(TaskController tc) {
            super(tc);
        }

        public void taskRun() {
//			showSC();
            robot.turnSonarsOn();
            cc = new CommandController(robot,knownMazeFile, pc);

            try{
                FileWriter outFileRawSonar = new FileWriter("TrackRawSonarData");
                FileWriter outFileFiltSonar = new FileWriter("TrackFiltSonarData");
                FileWriter outFileFollowTracker = new FileWriter("TrackFollowData");
                PrintWriter outRawSonar = new PrintWriter(outFileRawSonar);
                PrintWriter outFiltSonar = new PrintWriter(outFileFiltSonar);
                PrintWriter outFollowTracker = new PrintWriter(outFileFollowTracker);
                while(!shouldStop()) {

                    cc.step();

                    try {
                        Thread.sleep(50);

                    } catch(InterruptedException iex) {
                        System.out.println("\"Both\" sleep interrupted");
                    }
                }
                outRawSonar.close();
                outFiltSonar.close();
                outFollowTracker.close();
            } catch (IOException e){
                e.printStackTrace();
            }
            cc.closeDebug();

            robot.turnSonarsOff();
            robot.setVel(0.0f, 0.0f);
//			hideSC();
        }

        public String toString() {
            return "\"both\"";
        }


    }


    private static final long serialVersionUID = 0;
}
