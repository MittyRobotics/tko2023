package com.github.mittyrobotics.drivetrain;

import com.github.mittyrobotics.util.math.*;
import com.github.mittyrobotics.util.Pair;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.github.mittyrobotics.util.math.Angle;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.math.Vector;

import java.util.ArrayList;

import static com.github.mittyrobotics.drivetrain.SwerveConstants.*;

public class SwerveSubsystem {
    private static SwerveSubsystem instance;

    public static SwerveSubsystem getInstance() {
        if (instance == null) instance = new SwerveSubsystem();
        return instance;
    }

    public final InverseKinematics inverseKinematics = new InverseKinematics();
    public final ForwardKinematics forwardKinematics = new ForwardKinematics();

    private WPI_TalonFX[] driveMotors = new WPI_TalonFX[4];
    private WPI_TalonFX[] angleMotors = new WPI_TalonFX[4];

    private double[] prevEnc = new double[4];

    public void initHardware() {
        for (int i = 0; i < 4; i++) {
            driveMotors[i] = new WPI_TalonFX(DRIVE_MOTOR_IDS[i]);
            angleMotors[i] = new WPI_TalonFX(ANGLE_MOTOR_IDS[i]);

            driveMotors[i].configFactoryDefault();
            driveMotors[i].setSelectedSensorPosition(0);
            driveMotors[i].config_kP(0, DRIVE_PID[0]);
            driveMotors[i].config_kP(0, DRIVE_PID[1]);
            driveMotors[i].config_kP(0, DRIVE_PID[2]);
            driveMotors[i].setInverted(DRIVE_INVERTED[i]);

            angleMotors[i].configFactoryDefault();
            angleMotors[i].setSelectedSensorPosition(0);
            angleMotors[i].config_kP(0, ANGLE_PID[0]);
            angleMotors[i].config_kP(0, ANGLE_PID[1]);
            angleMotors[i].config_kP(0, ANGLE_PID[2]);
            angleMotors[i].config_kF(0, ANGLE_PID[3]);
            angleMotors[i].setInverted(ANGLE_INVERTED[i]);
        }
    }

    public void calculateInputs(Vector linearVel, double angularVel) {
        inverseKinematics.calculateInputs(linearVel, angularVel);
    }

    public void applyCalculatedInputs() {
        setDriveMotors(inverseKinematics.getMagnitudes());
        setAngleMotors(inverseKinematics.getAngles());
    }

    public void setDriveMotors(double[] values) {
        for (int i = 0; i < 4; i++) {
            driveMotors[i].set(ControlMode.Velocity, values[i]);
        }
    }

    public void setAngleMotors(double[] values) {
        for (int i = 0; i < 4; i++) {
            angleMotors[i].set(ControlMode.Position, values[i]);
        }
    }

    public double getModuleAngle(int i) {
        return angleMotors[i].getSelectedSensorPosition();
    }

    public void updateForwardKinematics() {
        Vector[] modules = new Vector[4];

        for (int i = 0; i < 4; i++) {
            double cur = driveMotors[i].getSelectedSensorPosition();

//            LoggerInterface.getInstance().put("Module " + i + " field angle", angle(i) + Gyro.getInstance().getHeadingRadians());
            modules[i] = new Vector(new Angle(angle(i) + Gyro.getInstance().getHeadingRadians()), 39.37 * (cur - prevEnc[i]) / SwerveConstants.TICKS_PER_METER);
//            System.out.println(i + ": " + angle(i));

            prevEnc[i] = cur;
        }

        forwardKinematics.updateForwardKinematics(modules);
    }

    static class InverseKinematics {
        private double[] angles;
        private double[] magnitudes;

        private int length, width;

        private Vector r;

        public InverseKinematics() {

        }

        public void calculateInputs(Vector linearVel, double angularVel) {
            linearVel = new Vector(
                    new Angle(
                            linearVel.getAngle().getRadians() - Gyro.getInstance().getRadians().getRadians(), true),
                    linearVel.getMagnitude()
            );

            for (int i = 0; i < 4; i++) {
                Vector wheelVector = Vector.add(linearVel, Vector.multiply(angularVel, getR(i)));
                angles[i] = wheelVector.getAngle().getRadians();
                magnitudes[i] = wheelVector.getMagnitude();
            }
        }

        public Vector getR(int i) {
            return new Vector(r.getX() * (i == 0 || i == 3 ? -1 : 1), r.getY() * (i > 1 ? -1 : 1));
        }

        public double[] getAngles() {
            return angles;
        }

        public double[] getMagnitudes() {
            return magnitudes;
        }
    }

    public static class ForwardKinematics {
        private ArrayList<Pair> poses = new ArrayList<>();

        private Vector vel = new Vector(0, 0);
        private Angle curHeading = new Angle(0, true);


        public ForwardKinematics() {

        }

        public void updateForwardKinematics(Vector[] modules) {

            long nanoTime = System.currentTimeMillis() * 1000000;

            Point new_ = new Point(0, 0);
            for (int i = 0; i < 4; i++) new_ = Point.add(new_, new Point(modules[i]));
            new_ = Point.multiply(0.25, new_);
//            new_ = new Point(new_.getX(), new_.getY());

            curHeading = new Angle(Gyro.getInstance().getHeadingRadians(), true);
            poses.add(new Pair(nanoTime, new Pose(Point.add(poses.get(poses.size() - 1).getValue().getPosition(), new_), curHeading)));

        }

        private int search(ArrayList<Pair> array, double value, boolean greater) {
            int start = 0, end = array.size() - 1;

            int ans = -1;
            while (start <= end) {
                int mid = (start + end) / 2;

                if (greater) {
                    if (array.get(mid).getKey() < value) {
                        start = mid + 1;
                    } else {
                        ans = mid;
                        end = mid - 1;
                    }
                } else {
                    if (array.get(mid).getKey() > value) {
                        end = mid - 1;
                    } else {
                        ans = mid;
                        start = mid + 1;
                    }
                }
            }
            return ans;
        }

        public Pose getPoseAtTime(double time) {
            long tl, tr;
            Pose pl, pr;

            int left_index = search(poses, time, false);
            int right_index = search(poses, time, true);

            if (left_index == -1) return poses.get(right_index).getValue();
            if (right_index == -1) return poses.get(left_index).getValue();

            tl = poses.get(left_index).getKey();
            tr = poses.get(right_index).getKey();
            pl = poses.get(left_index).getValue();
            pr = poses.get(right_index).getValue();

            if (tr == tl) return pl;

            double a1 = pl.getHeading().getRadians();
            double a2 = pr.getHeading().getRadians();

            return new Pose(
                    Point.add(pl.getPosition(), Point.multiply((time - tl) / (tr - tl),
                            Point.add(pr.getPosition(), Point.multiply(-1, pl.getPosition())))),

                    new Angle(a1 + ((time - tl) / (tr - tl)) * (a2 - a1), true)
            );

        }

        public Pose getLatestPose() {
            if (poses.size() == 0) return new Pose(new Point(0, 0), new Angle(0, true));
            return new Pose(poses.get(poses.size() - 1).getValue().getPosition(), poses.get(poses.size() - 1).getValue().getHeading());
        }

        public long getLatestTime() {
            return poses.get(0).getKey();
        }

        public Angle getCurHeading() {
            return curHeading;
        }
    }
}
