package com.github.mittyrobotics.autonomous.pathfollowing.v2;

import com.github.mittyrobotics.autonomous.pathfollowing.math.*;

public class Test {
    public static void main(String... args) {
//        QuinticHermiteSpline spline = new QuinticHermiteSpline(
//                new Pose(new Point(0, 0), new Angle(0)),
//                new Pose(new Point(100, 50), new Angle(0))
//        );
        QuinticHermiteSpline spline = new QuinticHermiteSpline(
                new Pose(new Point(81, 63), new Angle(Math.PI)),
                new Pose(new Point(75, 63), new Angle(Math.PI))
        );

        for(double t = 0; t <= 1+1e-6; t += 0.02) {
            Point p = spline.get(t);
            System.out.print((t == 0 ? "[(" : "(") + p.getX() + ", " + p.getY() + (t >= 1 ? ")]" : "), "));
        }
        System.out.println();

//        System.out.println(spline.getLength());
//        System.out.println(spline.getLength(0.5, 17));

//        SwervePath path = new SwervePath(spline, 10, 10, 10, 1, 0, 0);
//
//        Pose robot = new Pose(new Point(95, 47), new Angle(0));
//        Vector linear = path.updateLinear(robot, 0.02, 10);
//        System.out.println(linear);
    }
}
