package com.github.mittyrobotics;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.*;
import com.github.mittyrobotics.drivetrain.Pair;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import org.ejml.simple.SimpleMatrix;

public class TestOdometry {

    public static void main(String[] args) {

        boolean leftSide = true;
        boolean low = false;

        int tag_id = leftSide ? (low ? 8 : 6) : (low ? 1 : 3);
        int second_index = 1;
        int third_index = leftSide ? (low ? 0 : 2) : (low ? 2 : 0);

        Pose scoring = Odometry.scoringZones[tag_id-1][low ? 2 : 0];

        Pose starting = new Pose(Point.add(scoring.getPosition(), new Point(leftSide ? 32 : -32, 0)),
                new Angle(leftSide ? 0 : Math.PI));

        Pose firstCone = new Pose(Point.add(starting.getPosition(), new Point(leftSide ? 195 : -195, low ? 5 : -5)),
                starting.getHeading());

        Pose secondCone = new Pose(Point.add(firstCone.getPosition(), new Point(0, low ? 48 : -48)),
                new Angle(leftSide ? (low ? 1 : -1) : (low ? Math.PI - 1 : -(Math.PI - 1))));

        Pose beforeSecondCone = new Pose(Point.add(firstCone.getPosition(), new Point(leftSide ? -60 : 60, 0)),
                firstCone.getHeading());

        Pose beforeFirstCone = new Pose(Point.add(firstCone.getPosition(), new Point(leftSide ? -100 : 100, 0)),
                starting.getHeading());

        Pose beforeAutoScore = new Pose(Point.add(starting.getPosition(), new Point(leftSide ? 20 : -20, 0)),
                starting.getHeading());

        Point starting_second = Point.add(
                Odometry.scoringZones[tag_id-1][second_index].getPosition(),
                new Point(leftSide ? 32 : -32, 0));

        System.out.println(beforeAutoScore + "\n" + starting_second);

        Pose startingSecondPose = new Pose(starting_second,
                new Vector(starting_second, Point.add(beforeAutoScore.getPosition(), new Point(leftSide ? 30 : -30, 0))).getAngle());

        double scoreHeading = leftSide ? (low ? 2.75 : Math.PI) : (low ? 0 : -0.4);

        QuinticHermiteSpline spline = new QuinticHermiteSpline(beforeSecondCone, secondCone);

        for(double t = 0; t <= 1+1e-6; t += 0.02) {
            Point p = spline.get(t);
            System.out.print((t == 0 ? "[(" : "(") + p.getX() + ", " + p.getY() + (t >= 1 ? ")]" : "), "));
        }
        System.out.println();



    }
}
