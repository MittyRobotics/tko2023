package com.github.mittyrobotics;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.Pair;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import org.ejml.simple.SimpleMatrix;

public class TestOdometry {
    public static double standardize(double radians) {
        return (radians %= (Math.PI * 2)) >= 0 ? radians : (radians + 2 * Math.PI);
    }

    public static void joystick(double currentAngle, double currentDesired) {
        boolean right;
        double dist;

        if (currentDesired == Math.PI) {
            if (currentAngle < Math.PI) {
                right = true;
                dist = Math.PI - currentAngle;
            } else {
                right = false;
                dist = currentAngle - Math.PI;
            }
        } else {
            if (currentAngle > Math.PI) {
                right = true;
                dist = 2 * Math.PI - currentAngle;
            } else {
                right = false;
                dist = currentAngle;
            }
        }

        System.out.println(right + " | " + dist);
    }

    public static void score(double norm, double normDes) {
        boolean right;
        double dist;

        if (normDes < norm) {
            if (norm - normDes > Math.PI) {
                right = true;
                dist = normDes + 2 * Math.PI - norm;
            } else {
                right = false;
                dist = norm - normDes;
            }
        } else {
            if (normDes - norm > Math.PI) {
                right = false;
                dist = norm + 2 * Math.PI - normDes;
            } else {
                right = true;
                dist = normDes - norm;
            }
        }

        System.out.println(right + " | " + dist);
    }

    public static void main(String[] args) throws InterruptedException {

        double cur = Math.PI + 0.5;
        double curDes = 0;
//        System.out.println(standardize(1.5 * Math.PI));
        joystick(cur, curDes);

        score(cur, curDes);

    }
}
