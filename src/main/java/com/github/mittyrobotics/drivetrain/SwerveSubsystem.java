package com.github.mittyrobotics.drivetrain;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.github.mittyrobotics.drivetrain.commands.SwerveCommand;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.interfaces.IMotorSubsystem;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.github.mittyrobotics.drivetrain.SwerveConstants.*;
import static java.lang.Math.PI;

public class SwerveSubsystem extends SubsystemBase implements IMotorSubsystem {
    private int count = 0;

    private static SwerveSubsystem instance;

    private SwerveModuleState[] modules;

    private Translation2d[] modulePos = new Translation2d[4];

    private TalonFX[] angleMotors = new TalonFX[4], driveMotors = new TalonFX[4];

    private SwerveDriveKinematics kinematics;

    private com.reduxrobotics.sensors.canandcoder.Canandcoder[] absEncoders = new Canandcoder[4];

    private SwerveDriveOdometry odometry;

    public SwerveSubsystem() {

    }

    public static SwerveSubsystem getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystem();
        }
        return instance;
    }



    @Override
    public void initHardware() {
        for (int i = 0; i < 4; i++) {
            absEncoders[i] = new Canandcoder(SwerveConstants.ABS_ENCODER_IDS[i]);

            driveMotors[i] = new WPI_TalonFX(DRIVE_MOTOR_IDS[i]);
            angleMotors[i] = new WPI_TalonFX(ANGLE_MOTOR_IDS[i]);

            driveMotors[i].configFactoryDefault();
            driveMotors[i].setSelectedSensorPosition(0);
            driveMotors[i].config_kP(0, LINEAR_VELOCITY_P);
            driveMotors[i].config_kI(0, LINEAR_VELOCITY_I);
            driveMotors[i].config_kD(0, LINEAR_VELOCITY_D);
            driveMotors[i].config_kF(0, SPEED_FEED_FORWARD);
            driveMotors[i].setInverted(ROTATION_FALCON_INVERT);

            angleMotors[i].configFactoryDefault();
            angleMotors[i].config_kP(0, ANGLE_LOCK_P);
            angleMotors[i].config_kI(0, ANGLE_LOCK_I);
            angleMotors[i].config_kD(0, ANGLE_LOCK_D);
            angleMotors[i].setSelectedSensorPosition(0);
//            angleMotors[i].setInverted(ANGLE_INVERTED[i]);
            angleMotors[i].setNeutralMode(NeutralMode.Coast);
            angleMotors[i].setInverted(true);
//            angleMotors[i].setSelectedSensorPosition(angleMotors[i].getSelectedSensorPosition() + PI/2. * TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO);
        }

//        modulePos[0] = new Translation2d(SwerveConstants.L, -SwerveConstants.W);
//        modulePos[1] = new Translation2d(SwerveConstants.L, SwerveConstants.W);
//        modulePos[2] = new Translation2d(SwerveConstants.L, -SwerveConstants.W);
//        modulePos[3] = new Translation2d(-SwerveConstants.L, -SwerveConstants.W);

        setRelative();

        modulePos[0] = new Translation2d(SwerveConstants.L, SwerveConstants.W);
        modulePos[1] = new Translation2d(-SwerveConstants.L, SwerveConstants.W);
        modulePos[2] = new Translation2d(-SwerveConstants.L, -SwerveConstants.W);
        modulePos[3] = new Translation2d(SwerveConstants.L, -SwerveConstants.W);



        kinematics = new SwerveDriveKinematics(modulePos[0], modulePos[1], modulePos[2], modulePos[3]);

        setDefaultCommand(new SwerveCommand());
    }

    @Override
    public void updateDashboard() {

    }

    @Override
    public void overrideSetMotor(double percent) {

    }

    public void rotateTwo() {

    }

    public void driveFwd() {
        for (int i = 0; i < 4; i++) {
            driveMotors[i].set(ControlMode.PercentOutput, 0.1);
        }
    }

    public void setZero() {
        System.out.println("ID 30: " + angleMotors[3].getSelectedSensorPosition() + "ABS: " + absEncoders[3].getAbsPosition());
        count++;
        for (int i = 0; i < 4; i++) {
            if(count < 2)
            angleMotors[i].setSelectedSensorPosition((absEncoders[i].getAbsPosition() + 1/4) * TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO * 2. * PI);


            angleMotors[i].set(ControlMode.Position, 0);
        }

    }

    public void setRelative() {
        for (int i = 0; i < 4; i++) {
            angleMotors[i].setSelectedSensorPosition((absEncoders[i].getAbsPosition()) * TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO * 2. * PI);

        }
    }

    public double getAbsEncoderPosition(int i) {
        return absEncoders[i].getAbsPosition();
    }

    public void setModuleStates(ChassisSpeeds speed) {
        modules = kinematics.toSwerveModuleStates(speed);
//        System.out.println("MODULES: " + Arrays.toString(modules));
        //OPTIMIZATION
        for (int i = 0; i < 4; i++) {
            modules[i] = SwerveModuleState.optimize(modules[i],
                    new Rotation2d(angleMotors[i].getSelectedSensorPosition() / TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO));
        }
    }

    public void setModules() {
        for (int i = 0; i < 4; i++) {
//            getContinousOutput(modules[i], Gyro.getInstance().getRadians());
//            double[] currentDes = getContinousOutput(modules[i], angleMotors[i].getSelectedSensorPosition() / TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO);
//
//            angleMotors[i].set(ControlMode.Position, currentDes[1] * TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO);
//            driveMotors[i].set(ControlMode.Velocity, currentDes[0] * TICKS_PER_METER / 10);

//            System.out.println("angle " + i + modules[i].angle);
            angleMotors[i].set(ControlMode.Position, modules[i].angle.getRadians() * TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO);
            driveMotors[i].set(ControlMode.Velocity, modules[i].speedMetersPerSecond * TICKS_PER_METER / 10);
//            System.out.println("SPEED MOTOR: " + i + modules[i].speedMetersPerSecond);
        }
    }

    private double[] getContinousOutput(SwerveModuleState desiredState, double currentAngle) {
        double absoluteHeading = currentAngle % (2.0 * Math.PI);
        if (absoluteHeading < 0.0) {
            absoluteHeading += 2.0 * Math.PI;
        }
        double adjustedDesiredAngle = standardize0To2PI(desiredState.angle.getRadians()) + currentAngle - absoluteHeading;
        if (standardize0To2PI(desiredState.angle.getRadians()) - absoluteHeading > Math.PI) {
            return new double[] {
                    desiredState.speedMetersPerSecond,
                    adjustedDesiredAngle - 2.0 * Math.PI
            };
        } else if (standardize0To2PI(desiredState.angle.getRadians()) - absoluteHeading < -Math.PI) {
            return new double[] {
                    desiredState.speedMetersPerSecond,
                    adjustedDesiredAngle + 2.0 * Math.PI
            };
        } else {
            return new double[] {
                    desiredState.speedMetersPerSecond,
                    adjustedDesiredAngle
            };
        }
    }

    public double standardize0To2PI(double angle) {
        return ((angle % (2 * PI)) + 2 * PI) % (2 * PI);
    }

    public Rotation2d[] getModuleAngles() {
        Rotation2d[] angles = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            angles[i] = new Rotation2d(angleMotors[i].getSelectedSensorPosition() / TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO);
        }
        return angles;
    }

    public Pose2d getRobotPose() {
        return odometry.getPoseMeters();
    }

    public double[] getDriveDistanceMeters() {
        double[] distances = new double[4];
        for (int i = 0; i < 4; i++) {
            distances[i] = driveMotors[i].getSelectedSensorPosition() / TICKS_PER_METER;
        }
        return distances;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = new SwerveModulePosition(getDriveDistanceMeters()[i], getModuleAngles()[i]);
        }
        return modulePositions;
    }

    public void initPose(int startX, int startY, int startHeading) {
        odometry = new SwerveDriveOdometry(kinematics, Gyro.getInstance().getRotation2D(), getModulePositions(), new Pose2d(startX, startY, new Rotation2d(startHeading)));
    }

    //to be called in periodic()
    public void updatePose() {
        odometry.update(Gyro.getInstance().getRotation2D(), getModulePositions());
    }
}


