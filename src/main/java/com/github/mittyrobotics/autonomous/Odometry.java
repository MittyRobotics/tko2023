package com.github.mittyrobotics.autonomous;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import org.ejml.simple.SimpleMatrix;

public class Odometry {
    SimpleMatrix state = new SimpleMatrix(6, 1);
    SimpleMatrix covariance = new SimpleMatrix(6, 6);

    //INPUT DIMS
    SimpleMatrix kalmanGain = new SimpleMatrix(1,1);

//    SimpleMatrix W;
//
//    SimpleMatrix V;

    SimpleMatrix R;

    SimpleMatrix Q;

    SimpleMatrix H;

    public SimpleMatrix f(SimpleMatrix state, double v, double w, double dt) {
        //FIXXXXXXX
        return state.plus(new SimpleMatrix(new double[] {v * Math.cos(state.get(2)) * dt, v * Math.sin(state.get(2)), w * dt}));
//        return new SimpleMatrix(new double[][]
//                {{1, 0, dt, 0, 0, 0},
//                 {0, 1, 0, dt, 0, 0},
//                 {0, 0, 0, 0, 1, dt}});
    }

    public SimpleMatrix getJf(double v, double w, double dt) {
        return new SimpleMatrix(new double[][]
                {{1, 0, 0},
                 {0, 1, 0},
                 {-v * Math.sin(state.get(2)) * dt, v * Math.cos(state.get(2)) * dt, 1}});
    }

    public SimpleMatrix getJh() {
        return new SimpleMatrix(3, 6);
    }

    public void stateExtrapolate(double dt) {
        //REPLACE
        Vector vel = SwerveSubsystem.getInstance().getVel();
        state = f(state, vel.getMagnitude(), Gyro.getInstance().getAngularVel(), dt);
    }

    public void covarianceExtrapolate(double dt) {
        Vector vel = SwerveSubsystem.getInstance().getVel();
        SimpleMatrix J = getJf(vel.getMagnitude(), Gyro.getInstance().getAngularVel(), dt);
        covariance = J.mult(covariance).mult(J.transpose()).plus(Q);
    }

    public void kalmanGain() {
        SimpleMatrix J = getJh();
        kalmanGain = covariance.mult(J.transpose()).mult(J.mult(covariance).mult(J.transpose()).plus(R).invert());
    }

    public void stateUpdate(SimpleMatrix z) {
        state = state.plus(kalmanGain.mult(z.minus(H.mult(state))));
    }

    public void covarianceUpdate() {
        SimpleMatrix Jh = getJh();
        covariance = covariance
                .minus(kalmanGain.mult(Jh).mult(covariance))
                .minus(covariance.mult(Jh.transpose()).mult(kalmanGain.transpose()))
                .plus(kalmanGain.mult(Jh).mult(covariance).mult(Jh.transpose()).mult(kalmanGain.transpose()))
                .plus(kalmanGain.mult(R).mult(kalmanGain.transpose()));
    }

    public void update(double dt, SimpleMatrix... z) {
        stateExtrapolate(dt);
        covarianceExtrapolate(dt);
        for (int i = 0; i < z.length; i++) {
            kalmanGain();
            stateUpdate(z[i]);
            covarianceUpdate();
        }
    }
}
