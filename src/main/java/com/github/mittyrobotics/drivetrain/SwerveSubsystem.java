package com.github.mittyrobotics.drivetrain;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.github.mittyrobotics.util.interfaces.IMotorSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

            //SET PIDF
            angleMotors[i] = new TalonFX(SwerveConstants.ANGLE_MOTOR_IDS[i]);
            angleMotors[i].configFactoryDefault();
            angleMotors[i].setSelectedSensorPosition(0);

            driveMotors[i] = new TalonFX(SwerveConstants.DRIVE_MOTOR_IDS[i]);
            driveMotors[i].configFactoryDefault();
            driveMotors[i].setSelectedSensorPosition(0);
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
                    new Rotation2d(angleMotors[i].getSelectedSensorPosition()));
        }
    }

    public void setModules() {
        for (int i = 0; i < 4; i++) {
            //ADJUST FOR FLIP OVER 2PI/0 LINE
            angleMotors[i].setSelectedSensorPosition(modules[i].angle.getRadians());
            driveMotors[i].setSelectedSensorPosition(modules[i].speedMetersPerSecond);
        }
    }
}
