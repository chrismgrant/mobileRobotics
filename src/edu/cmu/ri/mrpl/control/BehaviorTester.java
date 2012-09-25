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
		RealPose2D p1 = new RealPose2D(1.0, 1.0, 45);
		RealPose2D p2 = new RealPose2D(3.0, 1.0, 45);
		RealPose2D  p3 = new RealPose2D(1.0, 1.0, 75);
		RealPose2D  p4 = new RealPose2D(1.0, 5.0, 20);
		RealPose2D  p5 = new RealPose2D(3.6, 2.0, 23.7);
		RealPose2D[] points = new RealPose2D[5];
		points[0] = p1;
		points[1] = p2;
		points[2] = p3;
		points[3] = p4;
		points[4] = p5;
		dummyPath.addAll(points);
		b.getClosestPoint(dummyPath, next);

	}
	
}
