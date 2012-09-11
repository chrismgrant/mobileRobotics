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
		final double vision = 1.5;
		history.add(rpos);
		//Look to see if something is in front of me
		
		
		world.filter(new F<RealPoint2D, Boolean>() {
			public Boolean f(RealPoint2D p){
				return p.x <= vision;
			}
		});
		//Look at list If I have been there go somewhere else

		//set destination
		setTarget(new RealPoint2D(0,2));
		//Set wheels to achieve it.
	}
	
	private RealPoint2D getClosest(List<RealPoint2D> l){
		return l.foldLeft(new F2<RealPoint2D, RealPoint2D, RealPoint2D>() {
			public RealPoint2D f(RealPoint2D out, RealPoint2D next){
				return (next.distance(0, 0) < out.distance(0, 0))?next:out;
			}
		}, new RealPoint2D(Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY));
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
