package com.github.mittyrobotics.drivetrain.commands;

import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystemPhoenix6;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static com.github.mittyrobotics.drivetrain.SwerveConstants.TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO;
import static java.lang.Math.PI;

public class SwerveCommand extends CommandBase {
    int count = 0;

    private double leftStickX;
    private double leftStickY;
    private double rightStickX;

    private ChassisSpeeds speed;
    private Rotation2d angle;
    private double velX, velY, angVel;

    public SwerveCommand() {
        addRequirements(SwerveSubsystemPhoenix6.getInstance());
    }
    @Override
    public void initialize() {
        velX = 0;
        velY = 0;
        angVel = 0;

        leftStickX = 0;
        leftStickY = 0;
        rightStickX = 0;

        angle = new Rotation2d(0);

        speed = new ChassisSpeeds(velX, velY, angVel);
    }

    @Override
    public void execute() {

        if(count < 2)
            SwerveSubsystemPhoenix6.getInstance().setRelative();



        leftStickX = OI.getInstance().getDriveController().getLeftX();
        leftStickY = OI.getInstance().getDriveController().getLeftY();
        rightStickX = OI.getInstance().getDriveController().getRightX();

        leftStickX = Math.abs(leftStickX) > 0.1 ? leftStickX : 0;
        leftStickY = Math.abs(leftStickY) > 0.1 ? leftStickY : 0;
        rightStickX = Math.abs(rightStickX) > 0.1 ? rightStickX : 0;

        velX = leftStickX * SwerveConstants.MAX_LINEAR_VEL;
        velY = leftStickY * SwerveConstants.MAX_LINEAR_VEL;
        angVel = rightStickX * SwerveConstants.MAX_ANGULAR_VEL;
//
//        System.out.println("ABS ENCODER: " + SwerveSubsystemPhoenix6.getInstance().getAbsEncoderPosition(0));

//        angle = Gyro.getInstance().getRotation2D();
        angle = new Rotation2d(-Gyro.getInstance().getHeadingRadians());

        //can merge if wanted
        speed = new ChassisSpeeds(velX, velY, angVel);
        speed = speed.fromFieldRelativeSpeeds(speed, angle);

        SwerveSubsystemPhoenix6.getInstance().setModuleStates(speed);
        SwerveSubsystemPhoenix6.getInstance().setModules();

        count++;
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
