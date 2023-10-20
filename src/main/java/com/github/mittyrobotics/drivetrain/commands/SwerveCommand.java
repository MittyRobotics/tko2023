package com.github.mittyrobotics.drivetrain.commands;

import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.drivetrain.SwerveSubsystemPhoenix6;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveCommand extends CommandBase {

    private double leftStickX;
    private double leftStickY;
    private double rightStickY;

    private ChassisSpeeds speed;
    private Rotation2d angle;
    private double velX, velY, angVel;

    @Override
    public void initialize() {
        velX = 0;
        velY = 0;
        angVel = 0;

        leftStickX = 0;
        leftStickY = 0;
        rightStickY = 0;

        angle = new Rotation2d(0);

        speed = new ChassisSpeeds(velX, velY, angVel);
    }

    @Override
    public void execute() {
        leftStickX = OI.getInstance().getDriveController().getLeftX();
        leftStickY = OI.getInstance().getDriveController().getLeftY();
        rightStickY = OI.getInstance().getDriveController().getRightY();

        leftStickX = Math.abs(leftStickX) > 0.1 ? leftStickX : 0;
        leftStickY = Math.abs(leftStickY) > 0.1 ? leftStickY : 0;
        rightStickY = Math.abs(rightStickY) > 0.1 ? rightStickY : 0;

        velX = leftStickX * SwerveConstants.MAX_LINEAR_VEL;
        velY = leftStickY * SwerveConstants.MAX_LINEAR_VEL;
        angVel = rightStickY * SwerveConstants.MAX_ANGULAR_VEL;

        angle = new Rotation2d(Gyro.getInstance().getHeadingAngle());

        //can merge if wanted
        speed = new ChassisSpeeds(velX, velY, angVel);
        speed = speed.fromFieldRelativeSpeeds(speed, angle);

        SwerveSubsystemPhoenix6.getInstance().setModuleStates(speed);
        SwerveSubsystemPhoenix6.getInstance().setModules();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
