package com.github.mittyrobotics;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.Pair;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import org.ejml.simple.SimpleMatrix;

public class TestOdometry {
    public static double standardize(double radians) {
        return (radians %= (Math.PI * 2)) >= 0 ? radians : (radians + 2 * Math.PI);
    }
    public static void main(String[] args) throws InterruptedException {
//        System.out.println(standardize(1.5 * Math.PI));

        System.out.println(System.currentTimeMillis() * 1000000);
    }
}
