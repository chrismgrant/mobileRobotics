package edu.cmu.ri.mrpl.maze;

/*

 * MazeWorld code for the 16x62 class taught at the Robotics Institute of
 * Carnegie Mellon University
 *
 * written from scratch by Martin Stolle in 2005 for the Fall 2005 class
 * revised in the Summer of 2006 for proper packaging
 *
 * inspired by code started by Illah Nourbakhsh and used for many years.
 */

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.io.*;
import java.awt.image.*;
import javax.imageio.*;

//import edu.cmu.ri.mrpl.maze.*;

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

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	protected MazeWorld mw;
	protected MazeGraphicsSwing mg;

	protected JPanel mazeContainer;
	protected JTextField statusField;

	protected JFileChooser fileChooser;

	public MazeViewer(String fileName) {

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

		mw = new MazeWorld(2,2);
		recreateMazeGraphics();

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

		loadMI.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				tryLoad();
			}
		});

		if (fileName!=null) {
			File file=new File(fileName);
			try {
				loadMaze(file);
			} catch(IOException ioex) {
				statusField.setText("maze loading failed!");
				JOptionPane.showMessageDialog(this, "Error loading "+file+": "+ioex, "Error loading file", JOptionPane.ERROR_MESSAGE);
			}
		}
	}

	public MazeViewer() {
		this(null);
	}

	protected void loadMaze(File file) throws IOException {

		MazeWorld braveNewWorld = new MazeWorld(file);
		this.mw = braveNewWorld;

		recreateMazeGraphics();

		statusField.setText("maze loaded from "+file);
	}

	protected void recreateMazeGraphics() {

		mazeContainer.removeAll();
		mg = new MazeGraphicsSwing(new MazeGraphics(mw));
		mazeContainer.add(mg);

		mazeContainer.validate();
		this.validate(); // so it computes the preferred size
		this.setSize(this.getPreferredSize());
		this.validate(); // so it redraws after resizing
	}

	public void tryLoad() {

		int returnVal = fileChooser.showOpenDialog(this);

		if(returnVal == JFileChooser.APPROVE_OPTION)  {
			File file = fileChooser.getSelectedFile();
			try {
				loadMaze(file);
			} catch(IOException ioex) {
				statusField.setText("maze loading failed!");
				JOptionPane.showMessageDialog(this, "Error loading "+file+": "+ioex, "Error loading file", JOptionPane.ERROR_MESSAGE);
			}
		}

	}
	
	// this enables us to call "java MazeViewer" from cmd line //
	public static void main(String args[]) {
		if (args.length == 1) {
			final String fileName = args[0];
			SwingUtilities.invokeLater(new Runnable() {
				public void run() {
					new MazeViewer(fileName);
				}
			});
			//Two arguments.  First is maze second is the image to create from maze.
		} else if(args.length == 2){
			try {
				MazeWorld mw = new MazeWorld(args[0]);
				MazeGraphics mg = new MazeGraphics(mw);
				
				BufferedImage image= new BufferedImage((int)mg.getPreferredSize().getWidth()+2, (int)mg.getPreferredSize().getHeight()+2, BufferedImage.TYPE_INT_RGB);
				mg.paint(image.getGraphics());
				File file = new File(args[1]);
				String extention = args[1].substring(args[1].lastIndexOf('.')+1);
				if(ImageIO.getImageReadersBySuffix(extention).hasNext())				
					ImageIO.write(image, extention, file);
				else
					System.out.println("Format \"" + extention +"\" is not supported.");
			} catch (Exception e) {
				e.printStackTrace();
			}
		} else
			SwingUtilities.invokeLater(new Runnable() {
				public void run() {
					new MazeViewer();
				}
			});
	}
}
