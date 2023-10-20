package com.github.mittyrobotics.drivetrain;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.github.mittyrobotics.util.interfaces.IMotorSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.github.mittyrobotics.drivetrain.SwerveConstants.*;

public class SwerveSubsystem extends SubsystemBase implements IMotorSubsystem {

    private static SwerveSubsystem instance;

    private SwerveModuleState[] modules;

    private Translation2d[] modulePos;

    private TalonFX[] angleMotors, driveMotors;

    private SwerveDriveKinematics kinematics;

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
            driveMotors[i] = new TalonFX(DRIVE_MOTOR_IDS[i]);
            driveMotors[i].setRotorPosition(0);
//            driveMotors[i].setInverted(ANGLE_INVERTED[i]);

            angleMotors[i] = new TalonFX(ANGLE_MOTOR_IDS[i]);
            angleMotors[i].setRotorPosition(0);
//            angleMotors[i].setInverted(ANGLE_INVERTED[i]);
//            WHERE NEUTRALMODE

            TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();

            //PID RETUNE
            angleMotorConfig.Slot0.kP = ANGLE_LOCK_P;
            angleMotorConfig.Slot0.kI = ANGLE_LOCK_I;
            angleMotorConfig.Slot0.kD = ANGLE_LOCK_D;
            angleMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
            angleMotorConfig.Feedback.SensorToMechanismRatio = SwerveConstants.TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO
                    / (2. * Math.PI);
            //INROTATIONS

            angleMotors[i].getConfigurator().apply(angleMotorConfig);

            TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

            driveMotorConfig.Slot0.kP = LINEAR_VELOCITY_P;
            driveMotorConfig.Slot0.kI = LINEAR_VELOCITY_I;
            driveMotorConfig.Slot0.kD = LINEAR_VELOCITY_D;
            driveMotorConfig.Slot0.kV = SPEED_FEED_FORWARD;
            driveMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;
            driveMotorConfig.Feedback.SensorToMechanismRatio = TICKS_PER_METER / (RADIUS_OF_WHEEL * 2. * Math.PI);
            //FIX CONVERSION DRIVE

            driveMotors[i].getConfigurator().apply(driveMotorConfig);
        }

        modulePos[0] = new Translation2d(SwerveConstants.xRad, SwerveConstants.yRad);
        modulePos[1] = new Translation2d(-SwerveConstants.xRad, SwerveConstants.yRad);
        modulePos[2] = new Translation2d(-SwerveConstants.xRad, -SwerveConstants.yRad);
        modulePos[3] = new Translation2d(SwerveConstants.xRad, -SwerveConstants.yRad);

        kinematics = new SwerveDriveKinematics(modulePos[0], modulePos[1], modulePos[2], modulePos[3]);
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
                    new Rotation2d(angleMotors[i].getRotorPosition().getValue()));
        }
    }

    public void setModules() {
        for (int i = 0; i < 4; i++) {
            angleMotors[i].setControl(new PositionVoltage(modules[i].angle.getRadians()));
            driveMotors[i].setControl(new VelocityVoltage(modules[i].speedMetersPerSecond));
        }
    }
}
