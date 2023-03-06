package com.github.mittyrobotics.autonomous;

import com.github.mittyrobotics.LoggerInterface;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.pivot.PivotSubsystem;
import com.github.mittyrobotics.util.Gyro;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import org.ejml.simple.SimpleMatrix;

import java.util.Arrays;

public class Odometry {
    private static Odometry instance;
    private double last_time;

    public static Odometry getInstance() {
        if (instance == null) instance = new Odometry();
        return instance;
    }

    public Odometry() {
        last_time = System.currentTimeMillis() * 1000000;
    }

    double offset = 20.873;
    Pose[][] scoringZones = {
            {
                new Pose(new Point(610.77, 42.19 + offset), new Angle(Math.PI)),
                new Pose(new Point(610.77, 42.19), new Angle(Math.PI)),
                new Pose(new Point(610.77, 42.19 - offset), new Angle(Math.PI)),
            },
            {
                new Pose(new Point(610.77, 108.19 + offset), new Angle(Math.PI)),
                new Pose(new Point(610.77, 108.19), new Angle(Math.PI)),
                new Pose(new Point(610.77, 108.19 - offset), new Angle(Math.PI)),
            },
            {
                new Pose(new Point(610.77, 147.19 + offset), new Angle(Math.PI)),
                new Pose(new Point(610.77, 147.19), new Angle(Math.PI)),
                new Pose(new Point(610.77, 147.19 - offset), new Angle(Math.PI)),
            },
            {
                new Pose(new Point(40.45, 147.19 + offset), new Angle(0)),
                new Pose(new Point(40.45, 147.19), new Angle(0)),
                new Pose(new Point(40.45, 147.19 - offset), new Angle(0)),
            },
            {
                new Pose(new Point(40.45, 108.19 + offset), new Angle(0)),
                new Pose(new Point(40.45, 108.19), new Angle(0)),
                new Pose(new Point(40.45, 108.19 - offset), new Angle(0)),
            },
            {
                new Pose(new Point(40.45, 42.19 + offset), new Angle(0)),
                new Pose(new Point(40.45, 42.19), new Angle(0)),
                new Pose(new Point(40.45, 42.19 - offset), new Angle(0)),
            },
    };
    Pose rightHP = new Pose(new Point(636.96, 265.74), new Angle(Math.PI));
    Pose leftHP = new Pose(new Point(14.25, 265.74), new Angle(0));

    SimpleMatrix state = new SimpleMatrix(3, 1);
    SimpleMatrix covariance = new SimpleMatrix(3, 3);

    SimpleMatrix kalmanGain = new SimpleMatrix(3,3);

    SimpleMatrix R = SimpleMatrix.identity(3);

    SimpleMatrix Q = SimpleMatrix.identity(3);

    private final double ERROR_MARGIN = 30;

    public void setState(double x, double y, double t) {
        state.set(0, 0, x);
        state.set(1, 0, y);
        state.set(2, 0, t);
    }

    public void update() {
        DoubleArraySubscriber sub = LoggerInterface.getInstance().getPoseSub();

        double[][] m = sub.readQueueValues();
        for(double[] measurement : m) {
            double x = measurement[0];
            double y = measurement[1];
            double theta = measurement[3];
            double time = measurement[4];
            double x_dist = measurement[5];

//            System.out.println(time);
//            System.out.println("System: " + System.currentTimeMillis() * 1000000);

            double dt = (time - last_time) / (1000000000.);

            Point lastP = SwerveSubsystem.getInstance().forwardKinematics.getPoseAtTime(last_time);
            Point curP = SwerveSubsystem.getInstance().forwardKinematics.getPoseAtTime(time);

            Angle lastA = SwerveSubsystem.getInstance().forwardKinematics.getAngleAtTime(last_time);
            Angle curA = SwerveSubsystem.getInstance().forwardKinematics.getAngleAtTime(time);

            Vector v = new Vector(Point.multiply(1/dt, Point.add(curP, Point.multiply(-1, lastP))));
            double w = (1/dt) * (curA.getRadians() - lastA.getRadians());

            try {
                stateExtrapolate(dt, v, w);
                covarianceExtrapolate(dt, v, w);

//                if (Math.sqrt((state.get(0, 0) - x) * (state.get(0, 0) - x) +
//                        (state.get(1, 0) - y) * (state.get(1, 0) - y)) > ERROR_MARGIN) {
//                    System.out.println("Bad input");
//                    continue;
//                }

                updateCovarianceR(x_dist);
                kalmanGain();
                stateUpdate(new SimpleMatrix(new double[]{x, y, theta}));
                covarianceUpdate();

                LoggerInterface.getInstance().putDesiredCamera(getIdealCamera());

                last_time = time;

//                state.transpose().print();
            } catch (Exception e) {
                System.out.println("error");
            }
        }
    }

    public double[] getPose() {
        double time = System.currentTimeMillis() * 1000000.;

        Point lastP = SwerveSubsystem.getInstance().forwardKinematics.getPoseAtTime(last_time);
        Point curP = SwerveSubsystem.getInstance().forwardKinematics.getPoseAtTime(time);

        Angle lastA = SwerveSubsystem.getInstance().forwardKinematics.getAngleAtTime(last_time);
        Angle curA = SwerveSubsystem.getInstance().forwardKinematics.getAngleAtTime(time);

        Point pos = Point.add(curP, Point.multiply(-1, lastP));
        double angle = curA.getRadians() - lastA.getRadians();

//        System.out.println(pos + " a: " + angle);

        return new double[]{pos.getX() + state.get(0, 0), pos.getY() + state.get(1, 0), angle + state.get(2, 0)};
    }

    public int getIdealCamera() {
        return 2;
    }

    public void updateCovarianceR(double x) {
        R = new SimpleMatrix(new double[][]{
                {0.00001 * x * x - 0.0007 * x + 0.015, 0, 0},
                {0, 0.00001 * x * x + 0.0006 * x, 0.001},
                {0, 0.001, 0.00001}
        });
    }

    //INPUT DIMS

//    SimpleMatrix W;
//
//    SimpleMatrix V;

//    SimpleMatrix H;

    public SimpleMatrix f(SimpleMatrix state, Vector v, double w, double dt) {
        //FIXXXXXXX - done?
//        System.out.println(v);
//        System.out.println(v.getX() * dt + "   " + v.getY() * dt + "    " + w * dt);
        return state.plus(new SimpleMatrix(new double[] {v.getX() * dt, v.getY() * dt, w * dt}));
//        return new SimpleMatrix(new double[][]
//                {{1, 0, dt, 0, 0, 0},
//                 {0, 1, 0, dt, 0, 0},
//                 {0, 0, 0, 0, 1, dt}});
    }

    public SimpleMatrix getJf(double v, double angle, double w, double dt) {
        return new SimpleMatrix(new double[][]
                {{1, 0, 0},
                 {0, 1, 0},
                 {-v * Math.sin(angle) * dt, v * Math.cos(angle) * dt, 1}});
    }

    public SimpleMatrix getJh() {
        return new SimpleMatrix(new double[][]
                {{1, 0, 0},
                 {0, 1, 0},
                 {0, 0, 1}});
    }

    public void stateExtrapolate(double dt, Vector v, double w) {
        //REPLACE
        state = f(state, v, w, dt);
    }

    public void stateExtrapolate(double dt) {
        //REPLACE
        Vector vel = SwerveSubsystem.getInstance().getVel();
        state = f(state, vel, Gyro.getInstance().getAngularVel(), dt);
    }

    public void covarianceExtrapolate(double dt) {
        Vector vel = SwerveSubsystem.getInstance().getVel();
        SimpleMatrix J = getJf(vel.getMagnitude(), vel.getAngle().getRadians(), Gyro.getInstance().getAngularVel(), dt);
        covariance = J.mult(covariance).mult(J.transpose()).plus(Q);
    }

    public void covarianceExtrapolate(double dt, Vector v, double w) {
        SimpleMatrix J = getJf(v.getMagnitude(), v.getAngle().getRadians(), w, dt);
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

    public void update(double dt, Vector v, double w, SimpleMatrix... z) {
        stateExtrapolate(dt);
        covarianceExtrapolate(dt, v, w);
        for (int i = 0; i < z.length; i++) {
            kalmanGain();
            stateUpdate(z[i]);
            covarianceUpdate();
        }
    }

//    public Pose getState() {
//        return new Pose(new Point(state.get(0), state.get(1)), new Angle(state.get(2)));
//    }

    public Pose getState() {
        return SwerveSubsystem.getInstance().getPose();
    }

    public Pose[] getClosestScoringZone() {
        int minIndex = 0;
        double min = Integer.MAX_VALUE;
        for (int i = 0; i < 6; i++) {
            if (new Vector(scoringZones[i][1].getPosition(), getState().getPosition()).getMagnitude() < min) {
                min = new Vector(scoringZones[i][1].getPosition(), getState().getPosition()).getMagnitude();
                minIndex = i;
            }
        }
        return scoringZones[minIndex];
    }
}
