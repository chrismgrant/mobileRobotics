package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Path;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
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
	
	/*private RealPoint2D getClosest(List<RealPoint2D> l)
	{
		Double currentMin = Double.POSITIVE_INFINITY;
		int minIndex = 0;
		for (int i = 0; i < l.length(); i++){
			if (l.toArray().get(i).distance(0,0) < currentMin){
				currentMin = l.toArray().get(i).distance(0,0);
				minIndex= i;
			}
		}
		if(l.length()>= 0){
		return l.toArray().get(minIndex);
		}
		return new RealPoint2D(6,0);
	}*/
	
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

	public void setHistory(Path history) {
		this.history = history;
	}
	/**
	 * Gets forward linear distance to move a certain distance
	 * @param distance distance remaining
	 * @return 
	 */
	public double moveForward(double distance){
		double speed = forwardPID.getOutput(distance);
//		System.out.println(speed);
		double target = clamp(speed, MAX_SPEED);
		return target; 
	}
	public double turnTo(double angleDistance){
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
	public void clearIntegrals(){
		forwardPID.clearIntegral();
		turnPID.clearIntegral();
	}
}
