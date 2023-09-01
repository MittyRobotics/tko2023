package com.github.mittyrobotics.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.github.mittyrobotics.drivetrain.commands.SwerveDefaultCommand;
import com.github.mittyrobotics.util.math.*;
import com.github.mittyrobotics.util.Pair;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.github.mittyrobotics.util.math.Angle;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.math.Vector;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.reduxrobotics.sensors.canandcoder.*;

import java.util.ArrayList;

import static java.lang.Math.*;

import static com.github.mittyrobotics.drivetrain.SwerveConstants.*;

public class SwerveSubsystem extends SubsystemBase {
    private static SwerveSubsystem instance;
    private boolean flipped[] = new boolean[4];

    public static SwerveSubsystem getInstance() {
        if (instance == null) instance = new SwerveSubsystem();
        return instance;
    }

    public final InverseKinematics inverseKinematics = new InverseKinematics();
    public final ForwardKinematics forwardKinematics = new ForwardKinematics();

    private WPI_TalonFX[] driveMotors = new WPI_TalonFX[4];
    private WPI_TalonFX[] angleMotors = new WPI_TalonFX[4];

    private CANandcoder[] absEncoders = new CANandcoder[4];

    private double[] prevEnc = new double[4];

    public void initHardware() {
        for (int i = 0; i < 4; i++) {
            driveMotors[i] = new WPI_TalonFX(DRIVE_MOTOR_IDS[i]);
            angleMotors[i] = new WPI_TalonFX(ANGLE_MOTOR_IDS[i]);

            absEncoders[i] = new CANandcoder(ABS_ENCODER_IDS[i]);

            driveMotors[i].configFactoryDefault();
            driveMotors[i].setSelectedSensorPosition(0);
            driveMotors[i].config_kP(0, DRIVE_PID[0]);
            driveMotors[i].config_kI(0, DRIVE_PID[1]);
            driveMotors[i].config_kD(0, DRIVE_PID[2]);
            driveMotors[i].config_kF(0, DRIVE_PID[3]);
            driveMotors[i].setInverted(DRIVE_INVERTED[i]);

            angleMotors[i].configFactoryDefault();
            angleMotors[i].setSelectedSensorPosition(-absEncoders[i].getAbsPosition()
                    * 2 * PI / TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO);
            angleMotors[i].config_kP(0, ANGLE_PID[0]);
            angleMotors[i].config_kI(0, ANGLE_PID[1]);
            angleMotors[i].config_kD(0, ANGLE_PID[2]);
            angleMotors[i].setInverted(ANGLE_INVERTED[i]);
            angleMotors[i].setNeutralMode(NeutralMode.Coast);

            setDefaultCommand(new SwerveDefaultCommand());
        }
    }

    public void calculateInputs(Vector linearVel, double angularVel) {
        inverseKinematics.calculateInputs(linearVel, angularVel);
    }

    public void applyCalculatedInputs() {
        setAngleMotors(inverseKinematics.getAngles());
        setDriveMotors(inverseKinematics.getMagnitudes());
    }

    public double[] getDesiredAngles() {
        return inverseKinematics.getAngles();
    }

    public double[] getDesiredMagnitudes() {
        return inverseKinematics.getMagnitudes();
    }

    public void setAngleMotors(double[] values) {
        for (int i = 0; i < 4; i++) {
            double currentModuleAngle = getStandardizedModuleAngle(i);
            values[i] = Angle.standardize(values[i]);

            boolean cw = (values[i] - currentModuleAngle < PI && values[i] - currentModuleAngle > 0)
                    || values[i] - currentModuleAngle < -PI;
            double dist = Angle.getRealAngleDistance(currentModuleAngle, values[i], cw);

            if (i == 0) System.out.println("C_Q: " + Angle.getQuadrant(currentModuleAngle) + " T_Q: " + Angle.getQuadrant(values[i]));

            boolean flip = dist > PI / 2;

            if (i == 0) System.out.printf("%.3f %.3f %.3f %b %b\n", currentModuleAngle, values[i], dist, cw, flip);

            //check
            flipped[i] = flip;

            values[i] = getEncoderModuleAngle(i) + (cw ? 1 : -1) * dist;

            if (flip) {
                values[i] += (cw ? -1 : 1) * PI;
            }

            if (i == 0) System.out.println("Setting to " + values[i]);
            angleMotors[i].set(ControlMode.Position, values[i] * TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO);
        }
    }

    public void setDriveMotors(double[] values) {
        for (int i = 0; i < 4; i++) {
            driveMotors[i].set(ControlMode.Velocity, (flipped[i] ? -1 : 1) * values[i] * TICKS_PER_INCH / 10);
        }
    }

    public double getRawWheelVelocity(int i) {
        return driveMotors[i].getSelectedSensorVelocity();
    }

    public double getWheelVelocityInches(int i) {
        return getRawWheelVelocity(i) / TICKS_PER_INCH * 10;
    }

    public double getWheelVelocityFeet(int i) {
        return getWheelVelocityInches(i) / 12;
    }

    public double getEncoderModuleAngle(int i) {
        return angleMotors[i].getSelectedSensorPosition() / TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO;
    }

    public double getRawPosition(int i) {
        return angleMotors[i].getSelectedSensorPosition();
    }

    public double getStandardizedModuleAngle(int i) {
        return Angle.standardize(getEncoderModuleAngle(i));
    }

    public void updateForwardKinematics() {
        Vector[] modules = new Vector[4];

        for (int i = 0; i < 4; i++) {
            double cur = driveMotors[i].getSelectedSensorPosition();

//            LoggerInterface.getInstance().put("Module " + i + " field angle", angle(i) + Gyro.getInstance().getHeadingRadians());
            modules[i] = new Vector(new Angle(getStandardizedModuleAngle(i) + Gyro.getInstance().getHeadingRadians(), true), (cur - prevEnc[i]) / TICKS_PER_INCH);
//            System.out.println(i + ": " + angle(i));

            prevEnc[i] = cur;
        }

        forwardKinematics.updateForwardKinematics(modules);
    }

    public double getAppliedOutput(boolean accessingAngleMotors, int i) {
        if (accessingAngleMotors) {
            return angleMotors[i].getMotorOutputVoltage();
        } else {
            return driveMotors[i].getMotorOutputVoltage();
        }
    }

    static class InverseKinematics {
        private double[] angles;
        private double[] magnitudes;

        private final double length = 25.68, width = 22.68;

        private Vector r;

        public InverseKinematics() {
            angles = new double[4];
            magnitudes = new double[4];

            r = new Vector(length, width);
        }

        public void calculateInputs(Vector linearVel, double angularVel) {
            linearVel = new Vector(
                    new Angle(
                            linearVel.getAngle().getRadians() - Gyro.getInstance().getHeadingRadians(), true),
                    linearVel.getMagnitude()
            );

            for (int i = 0; i < 4; i++) {
                Vector wheelVector = Vector.add(linearVel, Vector.multiply(angularVel, getAngularVector(i)));
                angles[i] = -wheelVector.getAngle().getRadians();
                magnitudes[i] = wheelVector.getMagnitude();
            }
        }

        public Vector getAngularVector(int i) {
            return new Vector(r.getY() * (i == 0 || i == 1 ? -1 : 1), r.getX() * (i == 0 || i == 3 ? 1 : -1));
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
