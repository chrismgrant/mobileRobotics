
package edu.cmu.ri.mrpl;

import java.io.*;
import edu.cmu.ri.mrpl.kinematics2D.*;

/** This class is used to represent a simple action.
 *
 *  Commands are defined by a command name (type), arguments required for that
 *   type of command, and a boolean flag indicating whether the command is
 *   to be executed as part of a continuous sequence.  If this flag is false,
 *   the robot is expected to come to a complete stop after completing the
 *   command as precisely as possible.  If true, the command need only be
 *   "approximately" completed so that motion may be smoothly continued with
 *   the next command (e.g., this may represent just a waypoint used a knot
 *   in a spline-based trajectory).
 *   <p>
 *   Available command types (textual type) and comma-separated arguments:
 *   <br>
 *   <pre>
 *    TURNTO      body_relative_angle_in_radians
 *    GOTO        body_relative_distance_in_meters
 *    MOVETO      body_relative_x,body_relative_y
 *    POSETO      body_relative_x,body_relative_y,body_relative_theta
 *    FOLLOWPATH  path_text_file_name
 *    FOLLOWPATH  [x1],[y1],[theta1];...;[xN],[yN],[thetaN]
 *    TURN90      integer_number_of_90_degree_rotations 
 *    TURNTOCELL  integer_number_of_90_degree_rotations_towards_target_cell
 *    GOTOCELL    integer_number_of_cells_to_move
 *    LOG
 *    WAIT        floating_point_number_of_seconds
 *    PAUSE
 *    RESTART
 *    PICKGOLD
 *    DROPGOLD
 *   </pre>
 *   <br>
 *   In all cases, forward or counter-clockwise is positive.
 *   <p>
 *   Note that commands not taking arguments are always assumed to have a
 *    <tt>false</tt> continuous flag.
 *   <p>
 *   See the CommandSequence class for information on storing commands in files
 *    and stringing together sequences of them.
 */
public class Command
{
    /** Numeric (enum) values of command types
      */
    public enum Type {
      /** Null command */
      NULL,
      /** Turn to the specified relative angle (ccw positive, radians) */
      TURNTO,
      /** Move the specified distance along the forward axis (forward positive, meters) */
      GOTO,
      /** Move to the specified relative position (forward x, left y, meters) */
      MOVETO,
      /** Move to the specified relative pose (forward x, left y, meters, ccw positive, radians) */
      POSETO,
      /** Follow a path comprised of pose waypoints */
      FOLLOWPATH,
      /** Return to the start of the current command sequence and continue execution */
      RESTART,
      /** Turn 90 degrees the specified number of times (ccw positive) */
      TURN90,
      /** Turn approx 90 degrees the specified number of times and align with the maze grid (ccw positive) */
      TURNTOCELL,
      /** Move the specified number of cells along the forward axis and align with the maze grid (forward positive) */
      GOTOCELL,
      /** Log the current world state for offline analysis */
      LOG,
      /** Pause execution for the specified number of seconds (floating point) */
      WAIT,
      /** Wait for some form of user input before continuing */
      PAUSE,
      /** Attempt to pick up gold at this location */
      PICKGOLD,
      /** Attempt to drop off gold at this location */
      DROPGOLD
    };

    /** The command to execute
     */
    public final Type type;

    /** Arguments to the command to execute
     */
    public final Argument argument;

    /** Whether this command is part of a continuous motion
     */
    public boolean isContinuous;

    /**
     * Abstract parent class for command arguments
     */
    public abstract static class Argument {
      public String serialize() { return toString(); }
    }

    /** Argument for a command that expects an angle parameter
     */
    public static class AngleArgument extends Argument {
      public Angle angle;

      AngleArgument(Angle a) { angle = new Angle(a); }
      AngleArgument(double a) { angle = new Angle(a); }

      public String toString() { return angle.toString(); }
    }

    /** Argument for a command that expects a floating point length/duration parameter
     */
    public static class LengthArgument extends Argument {
      public double d;

      LengthArgument(double d) { this.d = d; }

      public String toString() { return String.valueOf(d); }
    }

    /** Argument for a command that expects a 2-D point parameter
     */
    public static class PointArgument extends Argument {
      public RealPoint2D point;

      PointArgument(RealPoint2D p) { point = (RealPoint2D)p.clone(); }
      PointArgument(double x, double y) { point = new RealPoint2D(x,y); }

      public String toString() { return String.valueOf(point.getX()) + "," + String.valueOf(point.getY()); }
    }

    /** Argument for a command that expects a 2-D pose parameter
     */
    public static class PoseArgument extends Argument {
      public RealPose2D pose;

      PoseArgument(RealPose2D p) { pose = p.clone(); }
      PoseArgument(double x, double y, double theta) { pose = new RealPose2D(x,y,theta); }

      public String toString() { return String.valueOf(pose.getX()) + "," + String.valueOf(pose.getY()) + "," + String.valueOf(pose.getTh()); }
    }

    /** Argument for a command that expects a discrete count parameter
     */
    public static class CountArgument extends Argument {
      public int n;

      CountArgument(int n) { this.n = n; }

      public String toString() { return String.valueOf(n); }
    }

    /** Argument for a command that expects a path parameter
     */
    public static class PathArgument extends Argument {
      public Path path;

      PathArgument(Path p) { path = (Path)p.clone(); }
      PathArgument(String s) throws IOException {
        path = new Path();
        if (s.indexOf(';') >= 0)
          path.deserialize(s);
        else
          path.readFile(s);
      }

      public String toString() { return "path waypoints:\n" + path.toString(); }
      public String serialize() { return path.serialize(); }
    }

    /** Exception thrown when a malformed command argument is passed
     */
    class MalformedArgumentException extends Exception
    {
      public MalformedArgumentException(String message)
      {
        super(message);
      }
      static final long serialVersionUID = 0L;
    }

    /** Create an empty (null) command with no arguments with the continuous flag unset
     */
    public Command()
    {
      this(Type.NULL, (Argument)null, false);
    }

    /** Create a command of the specified type with no arguments with the continuous flag unset
     */
    public Command(Type type)
    {
      this(type, (Argument)null, false);
    }

    /** Create a command of the specified type and arguments with the continuous flag unset
     */
    public Command(Type type, Argument arg)
    {
      this(type, arg, false);
    }

    /** Create a command of the specified type and arguments with the continuous flag set as specified
     */
    public Command(Type type, Argument arg, boolean isContinuous)
    {
      this.type = type;
      argument = arg;
      this.isContinuous = isContinuous;
    }

    /** Create a command of the specified type and arguments (supplied textually) with the continuous flag unset
     */
    public Command(String typestr, String argstr) throws MalformedArgumentException
    {
      this(typestr, argstr, false);
    }

    /** Create a command of the specified type and arguments (supplied textually) with the continuous flag set as specified
     */
    public Command(String typestr, String argstr, boolean isContinuous) throws MalformedArgumentException
    {
      this.type = stringToType(typestr);
      if (this.type == Type.NULL)
        throw new MalformedArgumentException("Unrecognized command type \"" + typestr + "\"");
      argument = parseArgument(argstr);
      this.isContinuous = isContinuous;
    }

    /** Returns the parsed command type from its textual representation
     */
    public static Type stringToType(String s)
    {
      if (s.equalsIgnoreCase("TURNTO"))
        return Type.TURNTO;
      else if (s.equalsIgnoreCase("GOTO"))
        return Type.GOTO;
      else if (s.equalsIgnoreCase("MOVETO"))
        return Type.MOVETO;
      else if (s.equalsIgnoreCase("POSETO"))
        return Type.POSETO;
      else if (s.equalsIgnoreCase("FOLLOWPATH"))
        return Type.FOLLOWPATH;
      else if (s.equalsIgnoreCase("RESTART"))
        return Type.RESTART;
      else if (s.equalsIgnoreCase("TURN90"))
        return Type.TURN90;
      else if (s.equalsIgnoreCase("TURNTOCELL"))
        return Type.TURNTOCELL;
      else if (s.equalsIgnoreCase("GOTOCELL"))
        return Type.GOTOCELL;
      else if (s.equalsIgnoreCase("LOG"))
        return Type.LOG;
      else if (s.equalsIgnoreCase("WAIT"))
        return Type.WAIT;
      else if (s.equalsIgnoreCase("PAUSE"))
        return Type.PAUSE;
      else if (s.equalsIgnoreCase("PICKGOLD"))
        return Type.PICKGOLD;
      else if (s.equalsIgnoreCase("DROPGOLD"))
        return Type.DROPGOLD;
      else
        return Type.NULL;
    }

    /** Returns the textual representation of a command type
     */
    public static String typeToString(Type t)
    {
      return t.toString();
    }

    public String toString()
    {
    	String s = "";
        s += "Command\n";
	s += "  type: " + typeToString(type) + "\n";
	s += "  arg: " + (argument == null ? "null" : argument.toString()) + "\n";
	return s;
    }

    private Argument parseArgument(String argstr) throws MalformedArgumentException
    {
      Argument arg;
      String[] splitarg;

      if (argstr == null)
        argstr = "";

      if (type == Type.TURNTO)
      {
        try {
          arg = new AngleArgument(Double.parseDouble(argstr));
        }
        catch (NumberFormatException e) {
          throw new MalformedArgumentException("\"" + typeToString(type) + "\" requires 1 double argument");
        }
      }
      else if (type == Type.GOTO ||
               type == Type.WAIT) {
        try {
          arg = new LengthArgument(Double.parseDouble(argstr));
        }
        catch (NumberFormatException e) {
          throw new MalformedArgumentException("\"" + typeToString(type) + "\" requires 1 double argument");
        }
      }
      else if (type == Type.MOVETO) {
        try {
          splitarg = argstr.split(",");
          if (splitarg.length != 2)
            throw new MalformedArgumentException("\"" + typeToString(type) + "\" requires 2 double arguments");

          arg = new PointArgument(Double.parseDouble(splitarg[0]), Double.parseDouble(splitarg[1]));
        }
        catch (Exception e) {
          throw new MalformedArgumentException("\"" + typeToString(type) + "\" requires 2 double arguments");
        }
      }
      else if (type == Type.POSETO) {
        try {
          splitarg = argstr.split(",");
          if (splitarg.length != 3)
            throw new MalformedArgumentException("\"" + typeToString(type) + "\" requires 3 double arguments");

          arg = new PoseArgument(Double.parseDouble(splitarg[0]), Double.parseDouble(splitarg[1]), Double.parseDouble(splitarg[2]));
        }
        catch (Exception e) {
          throw new MalformedArgumentException("\"" + typeToString(type) + "\" requires 3 double arguments");
        }
      }
      else if (type == Type.FOLLOWPATH) {
        try {
          arg = new PathArgument(argstr);
        }
        catch (Exception e) {
          throw new MalformedArgumentException("\"" + typeToString(type) + "\" requires filename of existing well-formed path file or a serialized path as its argument");
        }
      }
      else if (type == Type.TURN90 ||
               type == Type.TURNTOCELL ||
               type == Type.GOTOCELL) {
        try {
          arg = new CountArgument(Integer.parseInt(argstr));
        }
        catch (NumberFormatException e) {
          throw new MalformedArgumentException("\"" + typeToString(type) + "\" requires 1 integer argument");
        }
      }
      else if (type == Type.RESTART ||
               type == Type.LOG ||
               type == Type.PAUSE ||
               type == Type.PICKGOLD ||
               type == Type.DROPGOLD) {
        arg = null;
      }
      else {
        throw new MalformedArgumentException("Parser does not recognize command + \"" + typeToString(type) + "\"");
      }

      return arg;
    }
}

