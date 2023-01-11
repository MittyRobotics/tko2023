package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;

import java.io.IOException;
import java.util.Arrays;

public class GenerateClosestPointTesting {
    public static void main(String[] args) throws IOException {
        QuinticHermiteSpline spline = new QuinticHermiteSpline(
                new Point(0, 0),
                new Angle(Math.PI / 2.),
                new Point(7, 0),
                new Angle(3*Math.PI / 2.)
        );
        SwervePath path = new SwervePath(spline, new Angle(0), new Angle(0));
        spline.getClosestPoint(new Pose(new Point(0.6, 0.4), new Angle(0)), 50, 10);
        System.out.println(spline.get(0.8));

        Point[] points = new Point[50];
        for (int i = 0; i < 50; i++) {
            double t = i/50.;
            Point p = spline.get(t);
            points[i] = p;
        }
        System.out.println("\n\n\n\n");
        System.out.println(Arrays.toString(points));
        System.out.println(spline.getClosestPoint(new Pose(new Point(6, 2), new Angle(0)), 50, 10));
        System.out.println(spline.get(spline.getClosestPoint(new Pose(new Point(6, 2), new Angle(0)), 50, 10)));
        System.out.println(path.getVectorToLookahead(new Pose(new Point(6, 2), new Angle(0)), 0.4));
    }
}
