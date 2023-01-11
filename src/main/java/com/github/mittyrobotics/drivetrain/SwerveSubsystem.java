package com.github.mittyrobotics.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.interfaces.IMotorSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase implements IMotorSubsystem {

    private static SwerveSubsystem instance;

    private InverseKinematics inverseKinematics;
    private ForwardKinematics forwardKinematics;

    private TalonFX[] driveFalcon = new TalonFX[4];
    private TalonFX[] rotationFalcon = new TalonFX[4];

    private Encoder[] encoder = new Encoder[4];

    private double[] prevEnc = new double[]{0, 0, 0, 0};

    boolean flip;

    public SwerveSubsystem() {
        super();
        setName("Swerve Modules");
    }

    public Pose getPose() {
        return new Pose(forwardKinematics.getPose(), new Angle(Gyro.getInstance().getHeadingRadians()));
    }

    public Angle getDirectionOfTravel() {
        return forwardKinematics.getDirectionOfTravel();
    }

    public static SwerveSubsystem getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystem();
        }
        return instance;
    }

    @Override
    public void updateDashboard() {

    }

    @Override
    public void initHardware() {
        for(int i = 0; i < 4; i++) {
            driveFalcon[i] = new WPI_TalonFX(SwerveConstants.DRIVE_FALCON[i]);
            driveFalcon[i].configFactoryDefault();
            driveFalcon[i].setNeutralMode(NeutralMode.Coast);
            driveFalcon[i].setSelectedSensorPosition(0);
//            driveFalcon[i].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            driveFalcon[i].config_kP(0, SwerveConstants.LINEAR_VELOCITY_P);
            driveFalcon[i].config_kI(0, SwerveConstants.LINEAR_VELOCITY_I);
            driveFalcon[i].config_kD(0, SwerveConstants.LINEAR_VELOCITY_D);
            driveFalcon[i].config_kF(0, SwerveConstants.SPEED_FEED_FORWARD);

            driveFalcon[i].configClosedloopRamp(0.5);

            rotationFalcon[i] = new WPI_TalonFX(SwerveConstants.ROTATION_FALCON[i]);
            rotationFalcon[i].configFactoryDefault();
            rotationFalcon[i].setNeutralMode(NeutralMode.Coast);
            rotationFalcon[i].setSelectedSensorPosition(0);
            rotationFalcon[i].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            rotationFalcon[i].config_kP(0, SwerveConstants.ANGULAR_POSITION_P);
            rotationFalcon[i].config_kI(0, SwerveConstants.ANGULAR_POSITION_I);
            rotationFalcon[i].config_kD(0, SwerveConstants.ANGULAR_POSITION_D);

            encoder[i] = new Encoder(SwerveConstants.MAG_ENCODER_CHANNEL[i][0], SwerveConstants.MAG_ENCODER_CHANNEL[i][1], true);
            encoder[i].reset();
            encoder[i].setDistancePerPulse(1./SwerveConstants.TICKS_PER_RADIAN_MAG_ENCODER);

//            encoder[i] = new Encoder(SwerveConstants.MAG_ENCODER_CHANNEL[i][0], SwerveConstants.MAG_ENCODER_CHANNEL[i][1]);
            prevEnc[i] = driveFalcon[i].getSelectedSensorPosition();
//            encoder[i].setDistancePerRotation(2048);
//            encoder[i].setPositionOffset(10);
//            rotationFalcon[i].setSelectedSensorPosition(encoder[i].getAbsolutePosition() * SwerveConstants.TICKS_PER_RADIAN_MAG_ENCODER);

            inverseKinematics = new InverseKinematics(SwerveConstants.TRACK_WIDTH, SwerveConstants.LENGTH);
            forwardKinematics = new ForwardKinematics(SwerveConstants.TRACK_WIDTH, SwerveConstants.LENGTH);


//            setDefaultCommand(new SwerveCommand());

        }
    }

//    public double getMagEncoderAbsRadians(int i) {
//        return encoder[i].getDistance();
//                / SwerveConstants.TICKS_PER_RADIAN_MAG_ENCODER;
//    }

    public void resetMagEncoder(int i) {
        encoder[i].reset();
    }

    public void resetMagEncoderAll() {
        for(int i = 0; i < 4; i++) {
            encoder[i].reset();
        }
    }

    public double getMagEncoderDistance(int i) {
        return encoder[i].getDistance();
    }

    //set all motors
    @Override
    public void overrideSetMotor(double percent) {
        setWheelPercentOutput(percent);
    }

    public void setAllControlMode(NeutralMode mode) {
        for(int i = 0; i < 4; i++) {
            driveFalcon[i].setNeutralMode(mode);
            rotationFalcon[i].setNeutralMode(mode);
        }
    }

    public void setSwerveModule(Vector linearVel, double angularVel) {
        inverseKinematics.setSwerveModule(linearVel, angularVel);
    }

    public double getPigeonHeading() {
        return Gyro.getInstance().getHeadingAngle();
    }

    public double[] desiredVelocities() {
        double[] velocities = new double[4];

        for(int i = 0; i < 4; i++) {
//            velocities[i] = Math.sqrt(inverseKinematics.swerveModule[i].getX() * inverseKinematics.swerveModule[i].getX() + inverseKinematics.swerveModule[i].getY()
//                    * inverseKinematics.swerveModule[i].getY());
            velocities[i] = inverseKinematics.swerveModule[i].getMagnitude();
        }

        return velocities;
    }

    public double[] desiredAngles() {
        double[] angles = new double[4];
        for(int i = 0; i < 4; i++) {
//            angles[i] = Math.atan2(inverseKinematics.swerveModule[i].getY(), inverseKinematics.swerveModule[i].getX());
            angles[i] = inverseKinematics.swerveModule[i].getAngle().getRadians();
        }

        return angles;
    }

    public void setSwerveVelocity(double[] desiredVelocities) {
        for(int i = 0; i < 4; i++) {
            double acDes = desiredVelocities[i] * (flip ? -1 : 1);

            driveFalcon[i].set(ControlMode.Velocity, acDes * SwerveConstants.TICKS_PER_METER * 0.1);
        }

    }

    public static double standardize(double radians) {
        return (radians %= (Math.PI * 2)) >= 0 ? radians : (radians + 2 * Math.PI);
    }

    public void setZero() {
        for(int i = 0; i < 4; i++) {
            driveFalcon[i].set(TalonFXControlMode.PercentOutput, 0);
        }
    }

    public double vel(int i) {
        return driveFalcon[i].getSelectedSensorVelocity() / SwerveConstants.TICKS_PER_METER * 10;
    }

    public void setWheelPercentOutput(double percent) {
        for(int i = 0; i < 4; i++) {
            driveFalcon[i].set(TalonFXControlMode.PercentOutput, percent);
        }
    }

    public void setAnglesZero() {
        for(int i = 0; i < 4; i++) {
//            rotationFalcon[i].setSelectedSensorPosition(encoder[i].getAbsolutePosition());
            rotationFalcon[i].set(ControlMode.Position, 0);
        }
    }

    public boolean flip() {
        return flip;
    }

    public double angle(int i) {
        double curSP = rotationFalcon[i].getSelectedSensorPosition() / SwerveConstants.TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO;
//        double curSP = encoder[i].getDistance();
//        double curSP = encoder[i].getAbsolutePosition();
        return standardize(curSP);
    }

    public double diff(double a1, double a2) {
        double df = (a2 - a1 + Math.PI) % (2 * Math.PI) - Math.PI;
        return df < -Math.PI ? df + 2 * Math.PI : df;
    }

    public void setSwerveAngle(double[] desiredAngles) {
        int flipNum = 0;

        for(int i = 0; i < 4; i++) {
            double norm = angle(i);

            double normDes = standardize(desiredAngles[i]);

            if (flip) norm = standardize(norm + Math.PI);

            if (Math.abs(diff(normDes, norm)) > Math.PI / 2) {
                flipNum++;
            }
        }


        if(flipNum > 2) {
            flip = !flip;
        }

        for(int i = 0; i < 4; i++) {
//            rotationFalcon[i].setSelectedSensorPosition(encoder[i].getAbsolutePosition());
//            double curSP = rotationFalcon[i].getSelectedSensorPosition();

            double curSP = rotationFalcon[i].getSelectedSensorPosition(0);

            double norm = standardize(curSP / SwerveConstants.TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO);

            double normDes = standardize(desiredAngles[i]);

            boolean right;
            double diff;

            if (flip) norm = standardize(norm + Math.PI);

            if (normDes < norm) {
                if (norm - normDes > Math.PI) {
                    right = true;
                    diff = normDes + 2 * Math.PI - norm;
                } else {
                    right = false;
                    diff = norm - normDes;
                }
            } else {
                if (normDes - norm > Math.PI) {
                    right = false;
                    diff = norm + 2 * Math.PI - normDes;
                } else {
                    right = true;
                    diff = normDes - norm;
                }
            }

//            rotationFalcon[i].setSelectedSensorPosition(encoder[i].getAbsolutePosition());
//            rotationFalcon[i].set(ControlMode.Position, curSP + (diff * (right ? 1 : -1) * SwerveConstants.TICKS_PER_RADIAN_FALCON_MAG_ENCODER));
            rotationFalcon[i].set(ControlMode.Position, curSP + (diff * (right ? 1 : -1) * SwerveConstants.TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO));
        }
    }

    public void setMeterPerSecond(double speed) {
        for(int i = 0; i < 4; i++) {
            driveFalcon[i].set(ControlMode.Velocity, speed * SwerveConstants.TICKS_PER_METER / 10);
        }
    }

    public double getSpeedOneMeters() {
        return driveFalcon[1].getSelectedSensorVelocity() / SwerveConstants.TICKS_PER_METER * 10;
    }

    public double ticks(int i) {
        return driveFalcon[i].getSelectedSensorPosition();
    }

    public double getSpeedOneTicks() {
        return driveFalcon[1].getSelectedSensorVelocity();
    }

    public void setPose(Point p) {
        forwardKinematics.pose = p;
    }

    public void resetPose() {
        setPose(new Point(0, 0));
    }

    public void updateForwardKinematics() {
        Vector[] modules = new Vector[4];

        for (int i = 0; i < 4; i++) {
            double cur = driveFalcon[i].getSelectedSensorPosition();

            modules[i] = new Vector(new Angle(-angle(i) + Gyro.getInstance().getHeadingRadians() + Math.PI/2.), (cur - prevEnc[i]) / SwerveConstants.TICKS_PER_METER);
//            System.out.println(i + ": " + angle(i));

            prevEnc[i] = cur;
        }

        SmartDashboard.putNumber("module angle", angle(0));

//        for(int i = 0; i < 4; i++) {
//            System.out.println(modules[i].toStringMetric());
//        }
        forwardKinematics.updateForwardKinematics(modules);
    }

    public static class InverseKinematics {
        private final Vector swerveModule[] = new Vector[4];
        private Vector tangentialVelocityVector[] = new Vector[4];

        private Vector r;

        private Vector linearVel;
        private double angularVel;

        public InverseKinematics(double width, double length) {
            r = new Vector(width/2, length/2);
        }

        public void setSwerveModule(Vector linearVel, double angularVel) {

            this.linearVel = linearVel;
            this.angularVel = angularVel;

            tangentialVelocityVector[0] = new Vector(-this.angularVel*r.getY(), this.angularVel*r.getX());
            tangentialVelocityVector[1] = new Vector(this.angularVel*r.getY(), this.angularVel*r.getX());
            tangentialVelocityVector[2] = new Vector(this.angularVel*r.getY(), -this.angularVel*r.getX());
            tangentialVelocityVector[3] = new Vector(-this.angularVel*r.getY(), -this.angularVel*r.getX());


            for(int i = 0; i < 4; i++) {
                swerveModule[i] = Vector.add(linearVel, tangentialVelocityVector[i]);
//                System.out.println(i + ": " + swerveModule[i]);
            }
        }
    }

    public static class ForwardKinematics {
        private final Vector r;
        private Point pose = new Point(0, 0);
        private Angle directionOfTravel = new Angle(0);


        public ForwardKinematics(double width, double length) {
            r = new Vector(width/2, length/2);
        }

        public void updateForwardKinematics(Vector[] modules) {

            Point new_ = new Point(0, 0);
            for (int i = 0; i < 4; i++) new_ = Point.add(new_, new Point(modules[i]));
            new_ = Point.multiply(0.25, new_);

            pose = Point.add(pose, new_);
            directionOfTravel = new Angle(new Vector(new_).getAngle().getRadians() - Math.PI/2);

//            System.out.println(pose.toString() + " " + new_.toString());


//            System.out.println(pose.toStringMetric());
        }

        public Vector getR(int i) {
            return new Vector(r.getX() * (i == 0 || i == 3 ? -1 : 1), r.getY() * (i > 1 ? -1 : 1));
        }

        public Point getPose() {
            return pose;
        }

        public Angle getDirectionOfTravel() {
            return directionOfTravel;
        }
    }
}



