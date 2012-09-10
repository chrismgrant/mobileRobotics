package edu.cmu.ri.mrpl.control;

import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import fj.data.List;

public class BehaviorController {

	private RealPoint2D targetPoint;
	
	public BehaviorController(){
		
		setTarget(new RealPoint2D(0,0)); 
	}
	
	public void updateBehavior(List<RealPoint2D> l){
		//Look at list find unchecked area, go there
		
		//For now just set target in front of us
		setTarget(new RealPoint2D(0,2));
	}
	
	public RealPoint2D getTarget() {
		return targetPoint;
	}

	public void setTarget(RealPoint2D target) {
		this.targetPoint = target;
	}
}
