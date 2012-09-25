package edu.cmu.ri.mrpl.control;

import java.awt.geom.Line2D;
import java.util.ArrayList;

import edu.cmu.ri.mrpl.Path;
import edu.cmu.ri.mrpl.kinematics2D.LineSegment;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import edu.cmu.ri.mrpl.kinematics2D.Vector2D;
import fj.F2;
import fj.data.List;
import fj.F;

public class BehaviorController {

	private final double MAX_SPEED = .886;
	private RealPoint2D targetPoint;
	private Path history;
	private PIDController forwardPID, turnPID;
	
	public BehaviorController(){
		setHistory(new Path());
		setTarget(new RealPoint2D(0,0)); 
		forwardPID = new PIDController(1,2.2);//was 1,1.9
		turnPID = new PIDController(1.0,.25);//was 1,.22
	}
	
	public void updateBehavior(RealPose2D rpos, List<RealPoint2D> world){
		final double vision = 1.5, speed = .6;
		history.add(rpos);
		//Look to see if something is in front of me
		//RealPoint2D closest = getClosest(world);
		for (RealPoint2D p : world.toCollection()){
			System.out.println(p.getX()+","+p.getY());
		}
		System.out.println("isupdating");
		List<RealPoint2D> frontList = filterFront(world);
		List<RealPoint2D> rightList = filterRight(world);
		List<RealPoint2D> leftList  = filterLeft(world);
		double right = getClosest(rightList).y;
		double left  = getClosest(leftList).y;
		double front = getClosest(frontList).x;
		System.out.println("Before right: "+right+" "+rightList.length()+" left: "+left+" "+leftList.length()+" front: "+front+" "+frontList.length());
		
		//setTarget(new RealPoint2D(-closest.getX(), -closest.getY()));
		right = (right < vision)? right/vision*speed : speed;
		front = (front < vision /2)? speed-(speed*front/(vision/2)):0;
		front = (right < left) ? -front/2 : front/2;
		left = (left < vision)? left/vision*speed : speed;
		System.out.println("right: "+right+" left: "+left+" front: "+front);

		/*world.filter(new F<RealPoint2D, Boolean>() {
			public Boolean f(RealPoint2D p){
				return p.x <= vision;
			}
		});*/
		//Look at list If I have been there go somewhere else

		//set destination
		setTarget(new RealPoint2D(right + front, left-front));
		
	}
	
	private RealPoint2D getClosest(List<RealPoint2D> l){
		return l.foldLeft(new F2<RealPoint2D, RealPoint2D, RealPoint2D>() {
			public RealPoint2D f(RealPoint2D out, RealPoint2D next){
				//System.out.println("next:x "+next.x+" y "+next.y+" out:x "+out.x+" y "+out.y);
				return (out.distance(0, 0) < next.distance(0, 0))?out:next;
			}
		}, new RealPoint2D(1, 1));
	}
	
	/*  
	 * Give a path(list of poses) and robo position then find the closest point.
	 */
 	private RealPoint2D getClosestPoint(Path l, RealPoint2D pos)
	{
		RealPoint2D current = new RealPoint2D(), closest = new RealPoint2D();
		Line2D path = new Line2D.Float();
		for (int i = 0; i+1 < l.size(); i++){
			path.setLine(l.get(i).getPosition(), l.get(i+1).getPosition());
			LineSegment.closestPointOnLineSegment(path, pos, current);
			if (current.distance(pos)< closest.distance(pos)){
				closest = current;
			}
		}
		if(l.size()>= 0){
		return closest;
		}
		return new RealPoint2D(0,0);
	}
	
	private RealPoint2D getMin(List<RealPoint2D> l){
		return l.foldLeft(new F2<RealPoint2D, RealPoint2D, RealPoint2D>() {
			public RealPoint2D f(RealPoint2D out, RealPoint2D next){
				return (out.distance(0, 0) > out.distance(.5, .5))?next:out;
			}
		}, new RealPoint2D(0, 3));
	}
	
	private boolean intersects(List<RealPoint2D> pointList, final RealPoint2D target, final double threshold){
		return !pointList.forall(new F<RealPoint2D, Boolean>() {
			public Boolean f(RealPoint2D p){
				return (p.distance(target) > threshold);
			}
		});
	}
	
	private List<RealPoint2D> filterFront(List<RealPoint2D> l){
		
		return l.filter(new F<RealPoint2D, Boolean>(){
			public Boolean f(RealPoint2D p){
				//return (p.getX()>0 && Math.abs(5*p.getY()) < p.getX());
				System.out.println(p.getX() +","+p.getY());
				return (p.getX() >= 0 && Math.abs(p.getY()) <= .5);
			}
		});
	}
	private List<RealPoint2D> filterRight(List<RealPoint2D> l){
		return l.filter(new F<RealPoint2D, Boolean>(){
			public Boolean f(RealPoint2D p){
				return (p.getX()>0 && p.getY() > .5 && p.getY()*2 > p.getX());
			}
		});
	}
	private List<RealPoint2D> filterLeft(List<RealPoint2D> l){
		return l.filter(new F<RealPoint2D, Boolean>(){
			public Boolean f(RealPoint2D p){
//				return (p.getX()>0 && Math.abs(5*p.getY()) < p.getX() && -p.getY()*2 > p.getX());
				return (p.getX()>0 && p.getY() > .5 && -p.getY()*2 > p.getX());
			}
		});
	}
	
	public RealPoint2D getTarget() {
		return targetPoint;
	}

	public void setTarget(RealPoint2D target) {
		this.targetPoint = target;
	}

	public Path getHistory() {
		return history;
	}
	/*
	 * Takes a path(list of poses) and refines it to a list of points with a .5 distance.
	 */
	public Path refinePath(Path l){
		Path betterList = new Path();
		ArrayList<Vector2D> newPoints = new ArrayList<Vector2D>();
		RealPose2D newPoint = new RealPose2D();
		RealPose2D nextPoint = new RealPose2D();
		RealPose2D startPoint = l.get(0);
		Line2D path = new Line2D.Float();

		betterList.add(startPoint);
		//Go through Each given point
		for(int i = 1; i < l.size(); i++ ){
			nextPoint = l.get(i);
			//Make a line from start to next point
			path.setLine(startPoint.getPosition(), nextPoint.getPosition());
			//Add points that are the right distance away
			while(LineSegment.radialPointsOnLineSegment(path, .5, startPoint.getPosition(), newPoints)){
				newPoint = new RealPose2D(newPoints.get(0).x,newPoints.get(0).y,startPoint.getTh());
				betterList.add(newPoint);
				startPoint = newPoint;	
			}
			betterList.add(nextPoint);
			startPoint = nextPoint;
		}
		return betterList;
	}
	
	public void setHistory(Path history) {
		this.history = history;
	}
	/**
	 * Gets forward linear distance to move a certain distance
	 * @param distance distance remaining
	 * @return 
	 */
	public synchronized double moveForward(double distance){
		double speed = forwardPID.getOutput(distance);
//		System.out.println(speed);
		double target = clamp(speed, MAX_SPEED);
		return target; 
	}
	public synchronized double turnTo(double angleDistance){
		double angle = turnPID.getOutput(angleDistance);
		double target = clamp(angle,MAX_SPEED*1.5);
		return target;
	}
	private double clamp(double val, double max){
		if (val > max){
			return max;
		} else if (val < -max){
			return -max;
		} else {
			return val;
		}
	}
	/**
	 * Clears integrals from PID controllers. Needed for 
	 */
	public synchronized void clearIntegrals(){
		forwardPID.clearIntegral();
		turnPID.clearIntegral();
	}
	/**
	 * Determines radius of turning for a given point. Useful for determining curvature of path
	 * @param p Target point to turn to
	 * @return radius in m of how far to turn
	 */
	public static double calculateRadiusOfTurning(RealPoint2D p){
		return (Math.pow(p.getX(), 2)+Math.pow(p.getY(),2))/(2*p.getY());
	}
	/**
	 * Shadows a point, maintaining a constant distance from the tracker
	 * @param p target point, robot-centric
	 * @param isLost whether reporting point is lost
	 * @param frontSonar distance front sonar is recording
	 * @param shadowDistance distance to maintain while shadowing, in meters. Set to zero to move to point
	 * @return array of [curvature, linearVel]
	 */
	public double[] shadowPoint(RealPoint2D p, boolean isLost, double frontSonar, double shadowDistance){
		double[] speed = {0,0};
		if (!isLost){
			double radius = calculateRadiusOfTurning(p);
			radius = (Double.isNaN(radius))?Double.POSITIVE_INFINITY:radius;
			speed[0] = 1/radius;
			double theta = Math.atan2(p.getX(),radius-p.getY());
			theta = (radius >= 0)?theta : -theta;
			double distance = radius * theta;
			distance = (Double.isNaN(distance))?p.getX():distance;
			distance = (Math.abs(theta) > 2.35)?1:distance;
			System.out.println(radius +","+theta+","+distance);
			speed[1] = moveForward(distance); 
//					WheelController.getCappedLVel(Math.min(p.getX() - shadowDistance, frontSonar),
//					WheelController.SPEED,
//					WheelController.MIN_SPEED);
//			Legacy code for baggage tracker
//				if (getLVel() < .1){	
//					pointToDirection(Math.atan2(p.getY(), p.getX()));
//				}	
		}
		return speed;
	}
}
