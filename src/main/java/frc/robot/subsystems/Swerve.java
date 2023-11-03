package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.util.math.*;
import com.reduxrobotics.sensors.canandcoder.CANandcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Objects;

import static frc.robot.Constants.SwerveConstants.*;
import static java.lang.Math.PI;

public class Swerve extends SubsystemBase {
    private Gyro gyro;

    private final InverseKinematics inverseKinematics;
    private final ForwardKinematics forwardKinematics;

    private WPI_TalonFX[] driveMotors;
    private WPI_TalonFX[] angleMotors;
    public CANandcoder[] absEncoders;

    private boolean flipped[];
    private double[] prevEnc;

    public Swerve(Gyro gyro) {
        this.gyro = gyro;

        inverseKinematics = new InverseKinematics(gyro);
        forwardKinematics = new ForwardKinematics(gyro);

        driveMotors = new WPI_TalonFX[4];
        angleMotors = new WPI_TalonFX[4];
        absEncoders = new CANandcoder[4];

        flipped = new boolean[4];
        prevEnc = new double[4];

        initHardware();
    }

    public void initHardware() {
        for (int i = 0; i < 4; i++) {
            driveMotors[i] = new WPI_TalonFX(DRIVE_MOTOR_IDS[i]);
            angleMotors[i] = new WPI_TalonFX(ANGLE_MOTOR_IDS[i]);

            absEncoders[i] = new CANandcoder(ABS_ENCODER_IDS[i]);

            driveMotors[i].configFactoryDefault();
            driveMotors[i].setSelectedSensorPosition(0);
            driveMotors[i].config_kP(0, DRIVE_PID[i][0]);
            driveMotors[i].config_kI(0, DRIVE_PID[i][1]);
            driveMotors[i].config_kD(0, DRIVE_PID[i][2]);
            driveMotors[i].config_kF(0, DRIVE_PID[i][3]);
            driveMotors[i].configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 60, 0.5));
            driveMotors[i].setInverted(DRIVE_INVERTED[i]);
            driveMotors[i].setNeutralMode(NeutralMode.Coast);
//            driveMotors[i].configOpenloopRamp(1.45);
//            driveMotors[i].configClosedloopRamp(1.45);
            angleMotors[i].configFactoryDefault();
            angleMotors[i].config_kP(0, ANGLE_PID[0]);
            angleMotors[i].config_kI(0, ANGLE_PID[1]);
            angleMotors[i].config_kD(0, ANGLE_PID[2]);
            angleMotors[i].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            angleMotors[i].setSelectedSensorPosition(absEncoders[i].getAbsPosition() * TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO * 2 * PI);
            angleMotors[i].configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 60, 0.5));
            angleMotors[i].setInverted(ANGLE_INVERTED[i]);
            angleMotors[i].setNeutralMode(NeutralMode.Brake);
        }
    }

    public ForwardKinematics getForwardKinematics() {
        return forwardKinematics;
    }

    public void setAllAngleEncodersZero() {
        for (int i = 0; i < 4; i++) {
            angleMotors[i].setSelectedSensorPosition(0);
        }
    }

    public void zeroRelativeEncoders() {
        for (int i = 0; i < 4; i++) {
            System.out.println(absEncoders[i]);
            angleMotors[i].setSelectedSensorPosition(absEncoders[i].getAbsPosition()
                    * 2 * PI * TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO);
        }
    }

    public void calculateInputs(Vector linearVel, double angularVel) {
        inverseKinematics.calculateInputs(linearVel, angularVel);
    }

    public void applyCalculatedInputs() {
        setAngleMotors(inverseKinematics.getAngles());
        setDriveMotors(inverseKinematics.getMagnitudes());
    }

    public void setAngleCoastMode() {
        for (int i = 0; i < 4; i++) {
            angleMotors[i].setNeutralMode(NeutralMode.Coast);
        }
    }

    public double[] getDesiredAngles() {
        return inverseKinematics.getAngles();
    }

    public double[] getDesiredMagnitudes() {
        return inverseKinematics.getMagnitudes();
    }

    public void setAngleMotors(double[] values, boolean allowedToFlip) {
        for (int i = 0; i < 4; i++) {
            double currentModuleAngle = getStandardizedModuleAngle(i);
            values[i] = Angle.standardize(values[i]);

            boolean cw = (values[i] - currentModuleAngle < PI && values[i] - currentModuleAngle > 0)
                    || values[i] - currentModuleAngle < -PI;
            double dist = Angle.getRealAngleDistanceSwerve(currentModuleAngle, values[i], cw);

//            if (i == 0)
//                System.out.println("C_Q: " + Angle.getQuadrant(currentModuleAngle) + " T_Q: " + Angle.getQuadrant(values[i]));

            boolean flip = dist > PI / 2;

//            if (i == 0) System.out.printf("%.3f %.3f %.3f %b %b\n", currentModuleAngle, values[i], dist, cw, flip);

            //check
            if (allowedToFlip) flipped[i] = flip;
            else flipped[i] = false;

            values[i] = getEncoderModuleAngle(i) + (cw ? 1 : -1) * dist;

            if (flip && allowedToFlip) {
                values[i] += (cw ? -1 : 1) * PI;
            }

//            if (i == 0) System.out.println("Setting to " + values[i]);
            angleMotors[i].set(ControlMode.Position, values[i] * TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO);
            System.out.print(angleMotors[i].getClosedLoopError() + ", ");
        }
        System.out.println();
    }

    public void setAngleMotors(double[] values) {
        setAngleMotors(values, true);
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
            modules[i] = new Vector(new Angle(-getStandardizedModuleAngle(i) + gyro.getHeadingRadians(), true), (cur - prevEnc[i]) / TICKS_PER_INCH);
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

    public static PIDController controller = new PIDController(4.5, 0, 0.004);

    public static double getDesiredAngularMP(double curHeading, double desiredHeading, double maxW, double maxA, double threshold) {
        maxW = 3.5;

        double norm = Angle.standardize(curHeading);
        double normDes = Angle.standardize(desiredHeading);

        boolean right;
        double dist;

        if (normDes < norm) {
            if (norm - normDes > Math.PI) {
                right = true;
                dist = normDes + 2 * Math.PI - norm;
            } else {
                right = false;
                dist = norm - normDes;
            }
        } else {
            if (normDes - norm > Math.PI) {
                right = false;
                dist = norm + 2 * Math.PI - normDes;
            } else {
                right = true;
                dist = normDes - norm;
            }
        }


        double out = controller.calculate(dist * (right ? -1 : 1));
        return Math.copySign(Math.min(maxW, Math.abs(out)), out);
    }

    public Vector getDesiredVel() {
        return inverseKinematics.linearVel;
    }

    public void setZero() {
        setDriveMotors(new double[]{0, 0, 0, 0});
    }

    public void lockWheels() {
        setAngleMotors(new double[]{0, 0, 0, 0}, false);
    }

    static class InverseKinematics {
        private Gyro gyro;

        private double[] angles;
        private double[] magnitudes;

        private final double length = 20.75, width = 17.75;

        private Vector r;

        private Vector linearVel = new Vector(0, 0);
        private double angularVel = 0;

        public InverseKinematics(Gyro gyro) {
            this.gyro = gyro;

            angles = new double[4];
            magnitudes = new double[4];

            r = new Vector(length, width);
        }

        public void calculateInputs(Vector linearVel, double angularVel) {
            this.linearVel = linearVel;
            linearVel = new Vector(
                    new Angle(
                            linearVel.getAngle().getRadians() - gyro.getHeadingRadians(), true),
                    linearVel.getMagnitude()
            );

            for (int i = 0; i < 4; i++) {
                Vector wheelVector = Vector.add(linearVel, Vector.multiply(angularVel, getAngularVector(i)));
                angles[i] = -wheelVector.getAngle().getRadians();
                magnitudes[i] = wheelVector.getMagnitude();
                System.out.print("WV " + i + ": " + wheelVector + ", ");
            }
            System.out.println();
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
        private Gyro gyro;

        private ArrayList<Pair> poses = new ArrayList<>();

        private Vector vel = new Vector(0, 0);
        private Angle curHeading = new Angle(0, true);


        public ForwardKinematics(Gyro gyro) {
            this.gyro = gyro;
        }

        public void init() {
            poses.add(new Pair(System.currentTimeMillis() * 1000000, new Pose(0, 0, 0, true)));
        }

        public void updateForwardKinematics(Vector[] modules) {

            long nanoTime = System.currentTimeMillis() * 1000000;

            Point new_ = new Point(0, 0);
            for (int i = 0; i < 4; i++) {
                Point p = new Point(modules[i].getX(), modules[i].getY());
                new_ = Point.add(new_, p);
            }
            new_ = Point.multiply(0.25, new_);
//            new_ = new Point(new_.getX(), new_.getY());

            curHeading = new Angle(gyro.getHeadingRadians(), true);
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

    /**
     * Container to ease passing around a tuple of two objects. This object provides a sensible
     * implementation of equals(), returning true if equals() is true on each of the contained
     * objects.
     */
    public static class Pair implements Serializable {
        private final Long first;
        private final Pose second;

        public Pair(Long first, Pose second) {
            this.first = first;
            this.second = second;
        }

        public long getKey() {
            return first;
        }

        public Pose getValue() {
            return second;
        }

        @Override
        public boolean equals(Object o) {
            if (!(o instanceof Pair)) {
                return false;
            }
            Pair p = (Pair) o;
            return Objects.equals(p.first, first) && Objects.equals(p.second, second);
        }

        @Override
        public int hashCode() {
            return (first.hashCode()) ^ (second == null ? 0 : second.hashCode());
        }

        @Override
        public String toString() {
            return "Pair{" + first + " " + second + "}";
        }
    }
}
