package edu.cmu.ri.mrpl.maze;

import java.text.*;
import java.awt.*;
import java.awt.event.*;
import java.beans.PropertyChangeListener;
import java.beans.PropertyChangeEvent;
import javax.swing.*;

import edu.cmu.ri.mrpl.maze.MazeGraphics;
import edu.cmu.ri.mrpl.maze.MazeGraphicsSwing;
import edu.cmu.ri.mrpl.maze.MazeState;
import edu.cmu.ri.mrpl.maze.MazeWorld;

import java.io.*;

/** Editor for .mazes that can be read into MazeWorld.  This is a stand alone
 * class.  Use it as a program or as inspiration/demo code.  Note that it uses
 * a MazeGraphicsSwing to wrap around a MazeGraphics class to make it
 * compatible with Swing.
 */
public class MazeEditor extends JFrame {

	protected MazeWorld mw;
    protected MazeGraphicsSwing mg;
    protected boolean changed = false;

    protected JPanel mazeContainer;
    protected JTextField statusField;

    protected QuitDialog quitDialog;
    protected ResizeDialog resizeDialog;
    protected JFileChooser fileChooser;

    protected final ButtonGroup buttonGroup;
    protected final ButtonModel wallButtonModel;
    protected final ButtonModel initsButtonModel;
    protected final ButtonModel goalsButtonModel;
    protected final ButtonModel goldsButtonModel;
    protected final ButtonModel dropsButtonModel;

    public MazeEditor(String fileName) {

      this.setTitle("Maze Editor");
      this.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);

      JMenuItem loadMI = new JMenuItem("load...");
      loadMI.setMnemonic('L');
      loadMI.setAccelerator(
	  KeyStroke.getKeyStroke(
	    java.awt.event.KeyEvent.VK_L,
	    java.awt.Event.CTRL_MASK));

      JMenuItem saveMI = new JMenuItem("save...");
      saveMI.setMnemonic('S');
      saveMI.setAccelerator(
	  KeyStroke.getKeyStroke(
	    java.awt.event.KeyEvent.VK_S,
	    java.awt.Event.CTRL_MASK));

      JMenuItem quitMI = new JMenuItem("quit");
      quitMI.setMnemonic('Q');
      quitMI.setAccelerator(
	  KeyStroke.getKeyStroke(
	    java.awt.event.KeyEvent.VK_Q,
	    java.awt.Event.CTRL_MASK));

      JMenu fileMenu = new JMenu("File");
      fileMenu.setMnemonic('F');
      fileMenu.add(loadMI);
      fileMenu.add(saveMI);
      fileMenu.addSeparator();
      fileMenu.add(quitMI);

      JMenuItem resizeMI = new JMenuItem("resize maze...");
      resizeMI.setMnemonic('R');
      resizeMI.setAccelerator(
	  KeyStroke.getKeyStroke(
	    java.awt.event.KeyEvent.VK_R,
	    java.awt.Event.CTRL_MASK));

      JMenu mazeMenu = new JMenu("Maze");
      mazeMenu.setMnemonic('M');
      mazeMenu.add(resizeMI);

      JMenuBar menuBar = new JMenuBar();
      menuBar.add(fileMenu);
      menuBar.add(mazeMenu);

      this.setJMenuBar(menuBar);

      this.getContentPane().setLayout(new BorderLayout(10,10));

      JRadioButton wallButton = new JRadioButton("walls");
      wallButtonModel = wallButton.getModel();
      JRadioButton initsButton = new JRadioButton("inits");
      initsButtonModel = initsButton.getModel();
      JRadioButton goalsButton = new JRadioButton("goals");
      goalsButtonModel = goalsButton.getModel();
      JRadioButton goldsButton = new JRadioButton("golds");
      goldsButtonModel = goldsButton.getModel();
      JRadioButton dropsButton = new JRadioButton("drops");
      dropsButtonModel = dropsButton.getModel();

      wallButton.setSelected(true);

      buttonGroup = new ButtonGroup();
      buttonGroup.add(wallButton);
      buttonGroup.add(initsButton);
      buttonGroup.add(goalsButton);
      buttonGroup.add(goldsButton);
      buttonGroup.add(dropsButton);

      JPanel buttonBar = new JPanel(new GridLayout(0,1));
      buttonBar.add(wallButton);
      buttonBar.add(initsButton);
      buttonBar.add(goalsButton);
      buttonBar.add(goldsButton);
      buttonBar.add(dropsButton);

      JPanel buttonSuper = new JPanel();
      buttonSuper.setLayout(new BorderLayout());
      buttonSuper.add(buttonBar, BorderLayout.NORTH);
      buttonSuper.add(new JLabel(), BorderLayout.CENTER);

      this.getContentPane().add(buttonSuper, BorderLayout.WEST);

      mazeContainer = new JPanel();
      this.getContentPane().add(mazeContainer, BorderLayout.CENTER);

      statusField = new JTextField("Welcome to the MazeEditor v1.11");
      statusField.setEditable(false);
      this.getContentPane().add(statusField, BorderLayout.SOUTH);

      this.pack();
      this.setVisible(true);

      mw = new MazeWorld(2,2);
      recreateMazeGraphics();

      quitDialog = new QuitDialog(this);
      quitDialog.pack();

      resizeDialog = new ResizeDialog(this);
      resizeDialog.pack();

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
	  tryQuit();
	}
      });

      quitMI.addActionListener(new ActionListener() {
	public void actionPerformed(ActionEvent e) {
	  tryQuit();
	}
      });

      loadMI.addActionListener(new ActionListener() {
	public void actionPerformed(ActionEvent e) {
	  tryLoad();
	}
      });

      saveMI.addActionListener(new ActionListener() {
	public void actionPerformed(ActionEvent e) {
	  trySave();
	}
      });

      resizeMI.addActionListener( new ActionListener() {
	public void actionPerformed(ActionEvent e) {
	  tryResize();
	}
      });

      mazeContainer.addMouseListener( new MouseAdapter() {
	public void mouseClicked(MouseEvent e) {
	  int ex = e.getX();
	  int ey = e.getY();
	  int mx = mg.getX();
	  int my = mg.getY();

	  //System.out.println(e);
	  //System.out.println(mg.getX()+" "+mg.getY());

	  if (ex>=mx && ey>=my && mx<mx+mg.getWidth() && my<my+mg.getHeight()) {
	    MazeState state = mg.pixel2MazeState(ex-mx, ey-my, .1);
	    
	    if (state==null) {
	      statusField.setText("ambiguous direction");
	    } 
	    else if (!mw.isValid(state.pos())){
	    	statusField.setText("outside maze");
	    }
	    else {

	      ButtonModel curModel = buttonGroup.getSelection();
	      if (curModel == wallButtonModel) {
		if (mw.isWall(state.x(), state.y(), state.dir())) {
		  try {
		    mw.removeWall(state.x(), state.y(), state.dir());
		    changed = true;
		    statusField.setText("removed wall "+state);
		  } catch(IllegalArgumentException iaex) {
		    statusField.setText(iaex.toString());
		  }
		} else {
		  mw.addWall(state.x(), state.y(), state.dir());
		  changed = true;
		  statusField.setText("added wall "+state);
		}
		repaint();
	      } else if (curModel == initsButtonModel) {
		if (mw.getInits().contains(state)) {
		  mw.removeInit(state);
		  changed = true;
		  statusField.setText("removed init "+state);
		} else {
		  mw.addInit(state);
		  changed = true;
		  statusField.setText("added init "+state);
		}
		repaint();
	      } else if (curModel == goalsButtonModel) {
		if (mw.getGoals().contains(state)) {
		  mw.removeGoal(state);
		  changed = true;
		  statusField.setText("removed goal "+state);
		} else {
		  mw.addGoal(state);
		  changed = true;
		  statusField.setText("added goal "+state);
		}
		repaint();
	      } else if (curModel == goldsButtonModel) {
		if (mw.getFreeGolds().contains(state)) {
		  mw.removeGold(state);
		  changed = true;
		  statusField.setText("removed gold (free) "+state);
		} else if (mw.getDropGolds().contains(state)) {
		  mw.removeGoldDropped(state);
		  changed = true;
		  statusField.setText("removed gold (dropped) "+state);
		} else {
		  mw.addGold(state);
		  changed = true;
		  statusField.setText("added gold "+state);
		}
		repaint();
	      } else if (curModel == dropsButtonModel) {
		if (mw.getDrops().contains(state)) {
		  mw.removeDrop(state);
		  changed = true;
		  statusField.setText("removed drop location "+state);
		} else {
		  mw.addDrop(state);
		  changed = true;
		  statusField.setText("added drop location "+state);
		}
		repaint();
	      }
	    }
	  } else {
	    statusField.setText("please click inside the maze");
	  }
	}
      });

      if (fileName!=null) {
	File file=new File(fileName);
	try {
	  loadMaze(file);
	  fileChooser.setSelectedFile(file);
	} catch(IOException ioex) {
	  statusField.setText("maze loading failed!");
	  JOptionPane.showMessageDialog(this, "Error loading "+file+": "+ioex, "Error loading file", JOptionPane.ERROR_MESSAGE);
	}
      }
    }

    public MazeEditor() {
      this(null);
    }

    protected boolean resizeMaze(int newWidth, int newHeight) {

      boolean perfectCopy;
      
      MazeWorld braveNewWorld = new MazeWorld(newWidth, newHeight);
      perfectCopy = braveNewWorld.copyFrom(mw);

      changed = true;
      this.mw=braveNewWorld;
      recreateMazeGraphics();

      return perfectCopy;
    }

    protected void loadMaze(File file) throws IOException {

      MazeWorld braveNewWorld = new MazeWorld(file);
      this.mw = braveNewWorld;
      changed = false;

      recreateMazeGraphics();

      this.setTitle("Maze Editor - "+file.getName());
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

    public void tryQuit() {
      if (!changed)
	System.exit(0);

      quitDialog.setVisible(true);

      if (quitDialog.confirmed())
	System.exit(0);
      else
	statusField.setText("quit aborted by user");
    }

    public void tryResize() {
      resizeDialog.setVisible(true);
      if (resizeDialog.isUserOK()) {
	if ( resizeMaze(resizeDialog.getUserWidth(), resizeDialog.getUserHeight()) )
	  statusField.setText("maze resized to "+mw.getWidth()+"x"+mw.getHeight());
	else
	  statusField.setText("discarded goal(s) and/or init(s) while resizing to "+mw.getWidth()+"x"+mw.getHeight());
      } else 
	statusField.setText("resize aborted by user");
    }

    public void tryLoad() {

      int returnVal = fileChooser.showOpenDialog(this);

      if(returnVal == JFileChooser.APPROVE_OPTION)  {

	if (changed) {
	  quitDialog.setVisible(true);

	  if (!quitDialog.confirmed()) {
	    statusField.setText("load aborted by user");
	    return;
	  }
	}

	File file = fileChooser.getSelectedFile();
	try {
	  loadMaze(file);
	} catch(IOException ioex) {
	  statusField.setText("maze loading failed!");
	  JOptionPane.showMessageDialog(this, "Error loading "+file+": "+ioex, "Error loading file", JOptionPane.ERROR_MESSAGE);
	}
      }

    }

    public void trySave() {
      int returnVal = fileChooser.showSaveDialog(this);

      if(returnVal == JFileChooser.APPROVE_OPTION) {
	File file = fileChooser.getSelectedFile();
	if (!file.getName().endsWith(".maze") && !file.getName().endsWith("."))
	  file = new File(file.getPath()+".maze");

	try {
	  PrintWriter pw = new PrintWriter(new FileOutputStream(file));
	  mw.save(pw);
	  pw.close();

	  changed = false;
	  this.setTitle("Maze Editor - "+file.getName());
	  statusField.setText("maze saved to "+file);

	} catch(IOException ioex) {
	  statusField.setText("maze saving failed!");
	  JOptionPane.showMessageDialog(this, "Error saving "+file+": "+ioex, "Error saving file", JOptionPane.ERROR_MESSAGE);
	}
      }
    }

    protected static class QuitDialog extends JDialog {

      /**
		 * 
		 */
		private static final long serialVersionUID = 1L;
	protected final JOptionPane quitPane;
     
      protected QuitDialog(Frame owner) {
	super(owner, "Confirm action", true);

	quitPane = new JOptionPane(
	    "This will discard your changes, do you want to continue?",
	    JOptionPane.WARNING_MESSAGE,
	    JOptionPane.YES_NO_OPTION
	    );

	quitPane.addPropertyChangeListener(
	    new PropertyChangeListener() {
	      public void propertyChange(PropertyChangeEvent e) {
		String prop = e.getPropertyName();

		if (isVisible() 
		  && (e.getSource() == quitPane)
		  && (prop.equals(JOptionPane.VALUE_PROPERTY))) {
		  setVisible(false);
		}
	      }
	    });

	this.setContentPane(quitPane);
      }

      public void setVisible(boolean visible) {
	if (visible)
	  quitPane.setValue(null);

	super.setVisible(visible);
      }

      public boolean confirmed() {
	Object value = quitPane.getValue();
	return (value!=null && ((Integer)value).intValue() == JOptionPane.YES_OPTION);
      }
    }

    protected class ResizeDialog extends JDialog {
      /**
		 * 
		 */
		private static final long serialVersionUID = 1L;
	protected final JTextField widthField;
      protected final JTextField heightField;

      protected int userWidth;
      protected int userHeight;

      protected boolean userOK;

      protected ResizeDialog(Frame owner) {
	super(owner, "Maze Size", true);

	widthField = new JFormattedTextField(NumberFormat.getIntegerInstance());
	heightField = new JFormattedTextField(NumberFormat.getIntegerInstance());

	JPanel mainPanel = new JPanel(new GridLayout(2,2));
	mainPanel.add(new JLabel("width:"));
	mainPanel.add(widthField);
	mainPanel.add(new JLabel("height:"));
	mainPanel.add(heightField);

	JButton okButton = new JButton("Ok");
	JButton cancelButton = new JButton("Cancle");

	Box buttonPanel = Box.createHorizontalBox();
	buttonPanel.add(Box.createGlue());
	buttonPanel.add(Box.createVerticalStrut(50));
	buttonPanel.add(Box.createHorizontalStrut(20));
	buttonPanel.add(okButton);
	buttonPanel.add(Box.createHorizontalStrut(20));
	buttonPanel.add(cancelButton);
	buttonPanel.add(Box.createVerticalStrut(30));
	buttonPanel.add(Box.createHorizontalStrut(20));
	buttonPanel.add(Box.createGlue());

	Box topPanel = Box.createHorizontalBox();
	topPanel.add(Box.createGlue());
	topPanel.add(Box.createVerticalStrut(30));
	topPanel.add(new JLabel("enter new maze size:"));
	topPanel.add(Box.createVerticalStrut(30));
	topPanel.add(Box.createGlue());

	Box leftPanel = Box.createVerticalBox();
	leftPanel.add(Box.createHorizontalStrut(20));
	Box rightPanel = Box.createVerticalBox();
	rightPanel.add(Box.createHorizontalStrut(20));

	this.getContentPane().add(leftPanel, BorderLayout.WEST);
	this.getContentPane().add(rightPanel, BorderLayout.EAST);
	this.getContentPane().add(topPanel, BorderLayout.NORTH);
	this.getContentPane().add(mainPanel, BorderLayout.CENTER);
	this.getContentPane().add(buttonPanel, BorderLayout.SOUTH);

	okButton.addActionListener(new ActionListener() {
	  public void actionPerformed(ActionEvent e) {

	    try {
	      userWidth = Integer.parseInt(widthField.getText());
	      if (userWidth<=0) {
		throw new NumberFormatException();
	      }

	    } catch(NumberFormatException nfex) {
	      JOptionPane.showMessageDialog(ResizeDialog.this, widthField.getText()+" is not a valid Number", "Invalid Number", JOptionPane.ERROR_MESSAGE);
	      return;
	    }

	    try {
	      userHeight = Integer.parseInt(heightField.getText());
	      if (userHeight<=0) {
		throw new NumberFormatException();
	      }
	    } catch(NumberFormatException nfex) {
	      JOptionPane.showMessageDialog(ResizeDialog.this, heightField.getText()+" is not a valid Number", "Invalid Number", JOptionPane.ERROR_MESSAGE);
	      return;
	    }

	    userOK = true;
	    setVisible(false);
	  }
	});

	cancelButton.addActionListener(new ActionListener() {
	  public void actionPerformed(ActionEvent e) {
	    setVisible(false);
	  }
	});
      }

      public void setVisible(boolean visible) {
	if (visible) {
	  widthField.setText(String.valueOf(mw.getWidth()));
	  heightField.setText(String.valueOf(mw.getHeight()));
	  userOK = false;
	}

	super.setVisible(visible);
      }

      public boolean isUserOK() {
	return userOK;
      }

      public int getUserWidth() {
	return userWidth;
      }

      public int getUserHeight() {
	return userHeight;
      }
    }

    // this enables us to call "java MazeEditor" from cmd line //
    public static void main(String args[]) {
      if (args.length>0) {
	final String fileName = args[0];
	SwingUtilities.invokeLater(new Runnable() {
	  public void run() {
	    new MazeEditor(fileName);
	  }
	});
      } else
	SwingUtilities.invokeLater(new Runnable() {
	  public void run() {
	    new MazeEditor();
	  }
	});
    }
    
    /**
	 * 
	 */
	private static final long serialVersionUID = 1L;
}
