package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.Path;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import fj.F2;
import fj.data.List;
import fj.F;

public class BehaviorController {

	private RealPoint2D targetPoint;
	private Path history;
	
	public BehaviorController(){
		setHistory(new Path());
		setTarget(new RealPoint2D(0,0)); 
	}
	
	public void updateBehavior(RealPose2D rpos, List<RealPoint2D> world){
		final double vision = 1.5, speed = .6;
		history.add(rpos);
		//Look to see if something is in front of me
		//RealPoint2D closest = getClosest(world);
		
		List<RealPoint2D> frontList = filterFront(world);
		List<RealPoint2D> rightList = filterRight(world);
		List<RealPoint2D> leftList  = filterLeft(world);
		double right = getClosest(rightList).y;
		double left  = getClosest(leftList).y;
		double front = getClosest(frontList).x;
		
		//setTarget(new RealPoint2D(-closest.getX(), -closest.getY()));
		right = (right < vision)? right/vision*speed : speed;
		front = (front < vision /2)? speed-(speed*front/(vision/2)):0;
		front = (right < left) ? -front/2 : front/2;
		left = (left < vision)? left/vision*speed : speed;
		
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
				return (next.distance(0, 0) < out.distance(0, 0))?next:out;
			}
		}, new RealPoint2D(Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY));
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
				return (p.getX()>0 && Math.abs(p.getY()) < p.getX());
			}
		});
	}
	private List<RealPoint2D> filterRight(List<RealPoint2D> l){
		return l.filter(new F<RealPoint2D, Boolean>(){
			public Boolean f(RealPoint2D p){
				return (p.getX()>0 && Math.abs(p.getY()) < p.getX() && p.getY()*2 > p.getX());
			}
		});
	}
	private List<RealPoint2D> filterLeft(List<RealPoint2D> l){
		return l.filter(new F<RealPoint2D, Boolean>(){
			public Boolean f(RealPoint2D p){
				return (p.getX()>0 && Math.abs(p.getY()) < p.getX() && -p.getY()*2 > p.getX());
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
}