package frc.robot.subsystems;

//import com.github.mittyrobotics.LoggerInterface;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.math.*;
import org.ejml.simple.SimpleMatrix;

public class PoseEstimator extends SubsystemBase {
    private Limelight limelight;
    private Swerve.ForwardKinematics odometry;

    private double last_time;
    public boolean FIELD_LEFT_SIDE = true;
    public final double FIELD_HALF_X = 325.61;
    public final double MID_TAG_Y = 108.19;
    public final double R_SCALE = 10;
    public final double Q_SCALE = 0.3;
    private Pose lastPose;

    public PoseEstimator(Limelight limelight, Swerve swerve) {
        this.limelight = limelight;
        this.odometry = swerve.getForwardKinematics();

        last_time = System.currentTimeMillis() * 1000000;
        lastPose = null;
//        LoggerInterface.getInstance().putDesiredCamera(2);
    }

    @Override
    public void periodic() {
        FIELD_LEFT_SIDE = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue);
    }

    public static final double offset = 20.873;
    public static final Pose[][] SCORING_ZONES = {
            {
                    new Pose(new Point(610.77, 42.19 + offset), new Angle(0, true)),
                    new Pose(new Point(610.77, 42.19), new Angle(0, true)),
                    new Pose(new Point(610.77, 42.19 - offset), new Angle(0, true)),
            },
            {
                    new Pose(new Point(610.77, 108.19 + offset), new Angle(0, true)),
                    new Pose(new Point(610.77, 108.19), new Angle(0, true)),
                    new Pose(new Point(610.77, 108.19 - offset), new Angle(0, true)),
            },
            {
                    new Pose(new Point(610.77, 174.19 + offset), new Angle(0, true)),
                    new Pose(new Point(610.77, 174.19), new Angle(0, true)),
                    new Pose(new Point(610.77, 174.19 - offset), new Angle(0, true)),
            },
            {},
            {},
            {
                    new Pose(new Point(40.45, 174.19 + offset), new Angle(Math.PI, true)),
                    new Pose(new Point(40.45, 174.19), new Angle(Math.PI, true)),
                    new Pose(new Point(40.45, 174.19 - offset), new Angle(Math.PI, true)),
            },
            {
                    new Pose(new Point(40.45, 108.19 + offset), new Angle(Math.PI, true)),
                    new Pose(new Point(40.45, 108.19), new Angle(Math.PI, true)),
                    new Pose(new Point(40.45, 108.19 - offset), new Angle(Math.PI, true)),
            },
            {
                    new Pose(new Point(40.45, 42.19 + offset), new Angle(Math.PI, true)),
                    new Pose(new Point(40.45, 42.19), new Angle(Math.PI, true)),
                    new Pose(new Point(40.45, 42.19 - offset), new Angle(Math.PI, true)),
            },
    };
    Pose rightHP = new Pose(new Point(636.96, 265.74), new Angle(Math.PI, true));
    Pose leftHP = new Pose(new Point(14.25, 265.74), new Angle(0, true));

    SimpleMatrix state = new SimpleMatrix(3, 1);
    SimpleMatrix covariance = new SimpleMatrix(3, 3);

    SimpleMatrix kalmanGain = new SimpleMatrix(3, 3);

    SimpleMatrix R = SimpleMatrix.identity(3);

    SimpleMatrix Q = SimpleMatrix.identity(3).scale(Q_SCALE);

    private final double ERROR_MARGIN = 100;
    private final double MIN_DISTANCE = 20;
    private final double MAX_DISTANCE = 100;

    public void setState(double x, double y, double t) {
        state.set(0, 0, x);
        state.set(1, 0, y);
        state.set(2, 0, t);
    }

    public void update() {
        updateFromLimelight();
    }

    public void updateFromLimelight() {
        Pose limelightPose = limelight.getPose();
        if (limelightPose == null) return;
        double x = limelightPose.getPosition().getX();
        double y = limelightPose.getPosition().getY();
        double theta = limelightPose.getHeading().getRadians();
        double nanoTime = System.currentTimeMillis() * 1000000 - limelight.getLatency();
        double x_dist = limelight.getXDistToTarget();

        double dt = (nanoTime - last_time) / (1000000000.);

        if (lastPose == null)
            lastPose = odometry.getPoseAtTime(last_time);
        Pose curOdometryPose = odometry.getPoseAtTime(nanoTime);

        Vector v = new Vector(Point.multiply(1 / dt, Point.add(curOdometryPose.getPosition(),
                Point.multiply(-1, lastPose.getPosition()))));
        double w = (1 / dt) * (curOdometryPose.getHeading().getRadians() - lastPose.getHeading().getRadians());

        try {
            stateExtrapolate(dt, v, w);
            covarianceExtrapolate(dt, v, w);

            updateCovarianceR(x_dist);
            kalmanGain();
            stateUpdate(new SimpleMatrix(new double[]{x, y, theta}));
            covarianceUpdate();

            last_time = nanoTime;
            lastPose = curOdometryPose;
        } catch (Exception e) {
            System.out.println("error");
        }
    }

    public double[] getOdometryExtrapolatedPose() {
        Pose curP = odometry.getLatestPose();
//        System.out.println("     - " + curP);

        if (lastPose == null) {
            last_time = System.currentTimeMillis() * 1000000;
            lastPose = curP;
        }
        Point pos = Point.add(curP.getPosition(), Point.multiply(-1, lastPose.getPosition()));
        double angle = curP.getHeading().getRadians() - lastPose.getHeading().getRadians();

        return new double[]{pos.getX() + state.get(0, 0), pos.getY() + state.get(1, 0), angle + state.get(2, 0)};
    }

    public boolean belowMiddleY() {
        return getOdometryExtrapolatedPose()[1] < MID_TAG_Y;
    }

    public void updateCovarianceR(double x) {
        R = new SimpleMatrix(new double[][]{
                {0.00001 * x * x - 0.0007 * x + 0.015, 0, 0},
                {0, 0.00001 * x * x + 0.0006 * x, 0.001},
                {0, 0.001, 0.1}
        }).scale(R_SCALE);
    }

    //INPUT DIMS

    public SimpleMatrix f(SimpleMatrix state, Vector v, double w, double dt) {
        return state.plus(new SimpleMatrix(new double[]{v.getX() * dt, v.getY() * dt, w * dt}));
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

    public void setLastPose(Pose p) {
        lastPose = p;
    }

    public void setLastTime(double t) {
        last_time = t;
    }

    public Pose getState() {
        double[] pose = getOdometryExtrapolatedPose();
        return new Pose(new Point(pose[0], pose[1]), new Angle(pose[2], true));
    }

    public Pose[] getScoringZone(int tag_id) {
        return SCORING_ZONES[tag_id - 1];
    }

    public static int getClosestScoringZone(Pose state) {
        int minIndex = 0;
        double dist = Double.POSITIVE_INFINITY;
        Point robot = state.getPosition();

        int i = 0;
        for (Pose[] p : SCORING_ZONES) {
            if (p.length == 3) {
                if (Point.getDistance(robot, p[1].getPosition()) < dist) {
                    minIndex = i;
                    dist = Point.getDistance(robot, p[1].getPosition());
                }
            }
            i++;
        }

        return minIndex + 1;
    }
}
