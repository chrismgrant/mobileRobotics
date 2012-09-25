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

public class BehaviorTester {
	public BehaviorTester(){
		Path dummyPath = new Path();
		BehaviorController b = new BehaviorController();
		RealPoint2D next = new RealPoint2D(1.0, 2.0);
		RealPose2D origin = new RealPose2D(1.0, 1.0, 45);
		dummyPath.add(origin);
		b.getClosestPoint(dummyPath, next);

	}
	
}
