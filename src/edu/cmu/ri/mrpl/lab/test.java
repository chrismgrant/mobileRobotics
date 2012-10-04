package edu.cmu.ri.mrpl.lab;

import edu.cmu.ri.mrpl.control.Convert;
import edu.cmu.ri.mrpl.control.Tracker;
import edu.cmu.ri.mrpl.kinematics2D.Angle;
import edu.cmu.ri.mrpl.kinematics2D.RealPoint2D;
import edu.cmu.ri.mrpl.kinematics2D.RealPose2D;
import fj.F;
import fj.F2;
import fj.data.List;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

/**
 * Created with IntelliJ IDEA.
 * User: WangHeli
 * Date: 10/4/12
 * Time: 3:45 PM
 * To change this template use File | Settings | File Templates.
 */
public class test {
    private static final double EPSILON = .001;

    private static double getPointError(final RealPose2D inputPose){
        Tracker a = new Tracker(new RealPoint2D(0,.4683));
        Tracker b = new Tracker(new RealPoint2D(.1,.2683));
        List<Tracker> filteredTrackers = List.list(a,b);
        List<Double> offsets = filteredTrackers.map(new F<Tracker, Double>() {
            public Double f(Tracker t){
                double distance, minDistance;
                Point2D holder = new Point2D.Double();
                RealPoint2D worldPoint;
                minDistance = Double.POSITIVE_INFINITY;
                worldPoint = Convert.WRTWorld(inputPose, t.getRPoint());
//                for (int i = 0; i < 4; i++){
////					distance = LineSegment.closestPointOnLineSegment(border[i], Convert.WRTWorld(inputPose, t.getRPoint()), holder);
//					if (distance < minDistance) {
//						minDistance = distance;
//					}
//				}
                distance = Math.min(Math.abs(worldPoint.getX()%.7366-.3683),Math.abs(worldPoint.getY()%.7366-.3683));

                return distance;
            }
        });
//        for (Double d : offsets){
//            System.out.print(d+",");
//        }
//        System.out.println(offsets.length());
        Double error = offsets.foldRight(new F2<Double,Double,Double>(){
            public Double f(Double a, Double b){
                return a+b;
            }
        }, 0.0);
        return Math.sqrt(error);
    }

    public static void main(String[] argv){
        RealPose2D oldMazePose = new RealPose2D(.7366,.7366,1.57);
        double dx, dy, dth;
        final double dp = 0.0001;
        dx = oldMazePose.getX()+dp;
        dy = oldMazePose.getY()+dp;
        dth = Angle.normalize(oldMazePose.getRotateTheta() + dp / Math.PI);
        double[] gradient = new double[3];
        gradient[0] = (getPointError(new RealPose2D(dx, oldMazePose.getY(),oldMazePose.getRotateTheta())) -
                getPointError(oldMazePose))/dp;
        gradient[1] = (getPointError(new RealPose2D(oldMazePose.getX(),dy,oldMazePose.getRotateTheta())) -
                getPointError(oldMazePose))/dp;
        gradient[2] = (getPointError(new RealPose2D(oldMazePose.getX(), oldMazePose.getY(),dth)) -
                getPointError(oldMazePose))/dp;
//        System.out.println(getPointError(border,oldMazePose));
        System.out.println(gradient[0]+","+gradient[1]+","+gradient[2]);

        //Traverse down gradient
        double last = Double.POSITIVE_INFINITY;
        RealPose2D nextPose = oldMazePose.clone();
        double nextError = getPointError(nextPose);
        while (last > nextError) {
            last = nextError;
            dx = -EPSILON * gradient[0]*nextPose.getX();
            dy = -EPSILON * gradient[1]*nextPose.getY();
            dth = -EPSILON * gradient[2]*nextPose.getTh();
            nextPose.add(dx, dy, dth);
            nextError = getPointError(nextPose);
        }
        nextPose.add(-dx, -dy, -dth);

        if (oldMazePose.getPosition().distance(nextPose.getPosition())< .7){
//            return nextPose;
            System.out.print(nextPose);
        } else {
            System.out.print(oldMazePose);
        }
    }
}
