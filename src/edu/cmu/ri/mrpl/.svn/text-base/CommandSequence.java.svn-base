
package edu.cmu.ri.mrpl;

import java.io.*;
import java.util.*;
import java.util.regex.*;
import edu.cmu.ri.mrpl.kinematics2D.*;

/** This class represents a sequence of simple commands.
 *
 * The file format loaded and saved is a text file containing a textual
 *  representation of a command and its required arguments, one command per
 *  line. Lines starting with %, #, or ; are  assumed to be comments and
 *  ignored.  Blank lines are also ignored.
 *  <p>
 * The format of the text files is:
 *  <pre>  [command type] [comma separated arguments] [isContinuous]</pre>
 *  <br>
 * If ommitted, <tt>isContinuous</tt> is assumed to be false.
 * <p>
 * See the Command class for the commands defined.
 * <p>
 * For a usage example, see the source to the main function.
 */

public class CommandSequence extends ArrayList<Command>
{
  static final long serialVersionUID = 1L;

  /** Create an empty command sequence
   */
  public CommandSequence()
  {
    super();
  }

  /** Append the contents of the provided file to this command sequence
   */
  public void readFile(String fname) throws IOException
  {
    System.out.printf("Loading command sequence from %s\n",fname);
    readFile(new FileInputStream(fname));
  }

  /** Append the contents of the provided input stream to this command sequence
   */
  public void readFile(InputStream input) throws IOException
  {
    readFile(new BufferedReader(new InputStreamReader(input)));
}

  /** Append the contents of the provided buffered reader to this command sequence
   */
  public void readFile(BufferedReader input) throws IOException
  {
    int linenum = 0;
    String s;

    try {
      while ((s = input.readLine()) != null) {
        linenum++;
        if (s.length() < 1 || s.charAt(0) == '%' || s.charAt(0) == '#' || s.charAt(0) == ';')
          continue;
        parseCommand(s);
      }
    }
    catch(IOException e){
      System.out.println("Error parsing input command stream");
      throw e;
    }
    catch(Exception e){
      System.out.printf("Error parsing input command stream at line %d: %s: %s\n",linenum, e.getClass().getName(), e.getMessage());
      throw new IOException(e.getMessage());
    }
  }

  private void parseCommand(String s) throws Exception
  {
    String type;
    String arg = null;
    boolean isContinuous = false;
    StringTokenizer t = new StringTokenizer(s);

    try {
      type = t.nextToken();
      if (t.hasMoreTokens())
        arg = t.nextToken();
      if (t.hasMoreTokens())
        isContinuous = Boolean.parseBoolean(t.nextToken());
      if (t.hasMoreTokens())
        System.out.println("Warning: Junk found on command line \"" + s + "\"");
      parseCommand(type, arg, isContinuous);
    }
    catch (PatternSyntaxException e) { }
    catch (NoSuchElementException e) {
      System.out.println("Command line \"" + s + "\" doesn't have enough tokens");
    }
  }

  private void parseCommand(String cmdstr, String argstr, boolean isContinuous) throws Exception
  {
    Command cmd;
//    Command.Type type;
//    Command.Argument arg;
//    String splitarg[];

    cmd = new Command(cmdstr, argstr, isContinuous);

    if (cmd.type == Command.Type.NULL) {
      throw new Exception("Unrecognized command \"" + cmdstr + "\"");
    }

    add(cmd);
  }

  /** Write out this command sequence to the specified file
   */
  public void writeFile(String fname) throws IOException
  {
    System.out.printf("Writing command sequence to %s\n",fname);
    writeFile(new FileOutputStream(fname));
  }

  /** Write out this command sequence to the specified output stream
   */
  public void writeFile(OutputStream output) throws IOException
  {
    writeFile(new BufferedWriter(new OutputStreamWriter(output)));
  }

  /** Write out this command sequence to the specified buffered writer
   */
  public void writeFile(BufferedWriter output) throws IOException
  {
    String space = " ";

    for (Command cmd: this) {
      output.write(Command.typeToString(cmd.type));
      output.write(space);
      if (cmd.argument != null)
        output.write(cmd.argument.serialize());
      if (cmd.isContinuous) {
        output.write(" TRUE");
      }
      output.write("\n");
      output.flush();
    }
  }

  /** This function provides an example usage for CommandSequence.
   */
  public static void main(String args[])
  {
    CommandSequence commands = new CommandSequence();

    try {
      commands.readFile("testcmds.txt");
    }
    catch (IOException e) {
    }

    for (Command cmd: commands) {
      System.out.println("Found command " + cmd.type.toString() + " with arg " + (cmd.argument != null ? cmd.argument.toString() : null));

      if (cmd.type == Command.Type.TURNTO) {
        Angle ang = ((Command.AngleArgument)cmd.argument).angle;
        System.out.println(" Rotating by angle: " + ang.toString());
      }
      else if (cmd.type == Command.Type.GOTO) {
        double dist = ((Command.LengthArgument)cmd.argument).d;
        System.out.println(" Moving by distance: " + dist);
      }
    }

    //wait for 30s
    commands.add(new Command(Command.Type.WAIT, new Command.LengthArgument(30.0)));

    //replace the first command with a motion to (x=3,y=4,theta=pi/2)
    commands.set(0, new Command(Command.Type.POSETO, new Command.PoseArgument(3, 4, Math.PI/2.0)));

    //go back to beginning
    commands.add(new Command(Command.Type.RESTART, null));

    try {
      commands.writeFile("testcmds_out.txt");
    }
    catch (IOException e) {
    }
  }
}

