package com.github.mittyrobotics;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.Pair;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import org.ejml.simple.SimpleMatrix;

public class TestOdometry {
    public static void main(String[] args) throws InterruptedException {
        int i = 0;
        long time = 0;
        while (i < 100) {
            Thread.sleep(10);
//            Odometry.getInstance().update(0.02, new Vector(3, 1), 0, new SimpleMatrix(
//                    new double[]{3.2*0.02*i, 1.2*0.02*i, 0}));
//            if (i == 50) Odometry.getInstance().state.print();
            SwerveSubsystem.getInstance().setPose(new Point(3.2*0.02*i, 1.2*0.02*i));
            if (i == 50) time = System.currentTimeMillis();
            i++;
        }
        System.out.println(SwerveSubsystem.getInstance().getPose());
    }
}
