package com.github.mittyrobotics.autonomous;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import org.ejml.simple.SimpleMatrix;

public class Odometry {
    SimpleMatrix state = new SimpleMatrix(6, 1);
    SimpleMatrix covariance = new SimpleMatrix(6, 6);

    //INPUT DIMS
    SimpleMatrix kalmanGain = new SimpleMatrix(1,1);

    SimpleMatrix W;

    SimpleMatrix V;

    SimpleMatrix R;

    SimpleMatrix Q;

    SimpleMatrix H;

    public SimpleMatrix getF(double dt) {
        return new SimpleMatrix(new double[][]
                {{1, 0, dt, 0, 0, 0},
                 {0, 1, 0, dt, 0, 0},
                 {0, 0, 0, 0, 1, dt}});
    }

    public SimpleMatrix getJf() {
        return new SimpleMatrix(3, 6);
    }

    public SimpleMatrix getJh() {
        return new SimpleMatrix(3, 6);
    }

    public void stateExtrapolate(double dt) {
        //POTENTIALLY REPLACE
        Vector vel = SwerveSubsystem.getInstance().getVel();
        state.set(2, vel.getX());
        state.set(3, vel.getY());
        state = state.mult(getF(dt));
    }

    public void covarianceExtrapolate(double dt) {
        SimpleMatrix J = getJf();
        covariance = J.mult(covariance).mult(J.transpose()).plus(Q);
    }

    public void kalmanGain() {
        SimpleMatrix J = getJh();
        SimpleMatrix temp = J.mult(covariance).mult(J.transpose()).plus(R).invert();
        kalmanGain = covariance.mult(J.transpose()).mult(temp);
    }
}
