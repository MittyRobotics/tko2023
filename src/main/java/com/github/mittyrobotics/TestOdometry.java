package com.github.mittyrobotics;

import com.github.mittyrobotics.autonomous.Odometry;
import org.ejml.simple.SimpleMatrix;

public class TestOdometry {
    public static void main(String[] args) throws InterruptedException {
        int i = 0;
        while (true) {
            Thread.sleep(1000);
            Odometry.getInstance().update(0.02, 3, 0, new SimpleMatrix(
                    new double[]{3*0.02*i*Math.cos(Odometry.getInstance().state.get(2)), 3*0.02*i*Math.sin(Odometry.getInstance().state.get(2)), 0}));
            Odometry.getInstance().state.print();
            i++;
        }
    }
}
