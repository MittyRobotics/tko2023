package com.github.mittyrobotics.drivetrain;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.github.mittyrobotics.drivetrain.commands.SwerveCommand;
import com.github.mittyrobotics.util.interfaces.IMotorSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.github.mittyrobotics.drivetrain.SwerveConstants.*;
import static java.lang.Math.PI;

public class SwerveSubsystemPhoenix6 extends SubsystemBase implements IMotorSubsystem {

    private static SwerveSubsystemPhoenix6 instance;

    private SwerveModuleState[] modules;

    private Translation2d[] modulePos;

    private TalonFX[] angleMotors, driveMotors;

    private SwerveDriveKinematics kinematics;

    public SwerveSubsystemPhoenix6() {

    }

    public static SwerveSubsystemPhoenix6 getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystemPhoenix6();
        }
        return instance;
    }



    @Override
    public void initHardware() {
        for (int i = 0; i < 4; i++) {

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
        }

        modulePos[0] = new Translation2d(SwerveConstants.W, SwerveConstants.L);
        modulePos[1] = new Translation2d(-SwerveConstants.W, SwerveConstants.L);
        modulePos[2] = new Translation2d(-SwerveConstants.W, -SwerveConstants.L);
        modulePos[3] = new Translation2d(SwerveConstants.W, -SwerveConstants.L);

        kinematics = new SwerveDriveKinematics(modulePos[0], modulePos[1], modulePos[2], modulePos[3]);

        setDefaultCommand(new SwerveCommand());
    }

    @Override
    public void updateDashboard() {

    }

    @Override
    public void overrideSetMotor(double percent) {

    }

    public void setModuleStates(ChassisSpeeds speed) {
        modules = kinematics.toSwerveModuleStates(speed);
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

            angleMotors[i].set(ControlMode.Position, modules[i].angle.getRadians() * TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO);
            driveMotors[i].set(ControlMode.Velocity, modules[i].speedMetersPerSecond * TICKS_PER_METER / 10);
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
}


