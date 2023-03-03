package com.github.mittyrobotics.autonomous;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import org.ejml.simple.SimpleMatrix;

public class Odometry {
    private static Odometry instance;

    public static Odometry getInstance() {
        if (instance == null) instance = new Odometry();
        return instance;
    }

    public SimpleMatrix state = new SimpleMatrix(3, 1);
    SimpleMatrix covariance = new SimpleMatrix(3, 3);

    //INPUT DIMS
    SimpleMatrix kalmanGain = new SimpleMatrix(3,3);

//    SimpleMatrix W;
//
//    SimpleMatrix V;

    SimpleMatrix R = SimpleMatrix.identity(3);

    SimpleMatrix Q = SimpleMatrix.identity(3);

//    SimpleMatrix H;

    public SimpleMatrix f(SimpleMatrix state, double v, double w, double dt) {
        //FIXXXXXXX - done?
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
        return new SimpleMatrix(new double[][]
                {{1, 0, 0},
                 {0, 1, 0},
                 {0, 0, 1}});
    }

    public void stateExtrapolate(double dt, double v, double w) {
        //REPLACE
        state = f(state, v, w, dt);
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

    public void covarianceExtrapolate(double dt, double v, double w) {
        SimpleMatrix J = getJf(v, w, dt);
        covariance = J.mult(covariance).mult(J.transpose()).plus(Q);
    }

    public void kalmanGain() {
        SimpleMatrix J = getJh();
        kalmanGain = covariance.mult(J.transpose()).mult(J.mult(covariance).mult(J.transpose()).plus(R).invert());
    }

    public void stateUpdate(SimpleMatrix z) {
        state = state.plus(kalmanGain.mult(z.minus(state)));
    }

    public void covarianceUpdate() {
        SimpleMatrix Jh = getJh();
        covariance = covariance
                .minus(kalmanGain.mult(Jh).mult(covariance))
                .minus(covariance.mult(Jh.transpose()).mult(kalmanGain.transpose()))
                .plus(kalmanGain.mult(Jh).mult(covariance).mult(Jh.transpose()).mult(kalmanGain.transpose()))
                .plus(kalmanGain.mult(R).mult(kalmanGain.transpose()));
    }

    public void update(double dt, double v, double w, SimpleMatrix... z) {
        stateExtrapolate(dt, v, w);
        covarianceExtrapolate(dt, v, w);
        for (int i = 0; i < z.length; i++) {
            kalmanGain();
            stateUpdate(z[i]);
            covarianceUpdate();
        }
    }
}
