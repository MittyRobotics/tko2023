package com.github.mittyrobotics.drivetrain.commands;

import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.OI;
import com.github.mittyrobotics.util.math.Angle;
import com.github.mittyrobotics.util.math.Vector;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDefaultCommand extends CommandBase {
    private double throttleX, throttleY, throttleAngular, joystickDeadzone;
    public SwerveDefaultCommand() {
        setName("Swerve Default Command");
        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        throttleX = 0;
        throttleY = 0;
        throttleAngular = 0;
        joystickDeadzone = 0;
    }

    @Override
    public void execute() {
        throttleX = -OI.getInstance().getDriverController().getLeftY();
        throttleY = -OI.getInstance().getDriverController().getLeftX();
        throttleAngular = OI.getInstance().getDriverController().getRightX();

        if (Math.abs(throttleX) > joystickDeadzone && Math.abs(throttleY) > joystickDeadzone)
            SwerveSubsystem.getInstance().calculateInputs(new Vector(5 * throttleX, 5 * throttleY), 0);
//        if (OI.getInstance().getDriverController().getRightBumper()) {
//            SwerveSubsystem.getInstance().calculateInputs(new Vector(0, 5), 0);
//        } else  {
//            SwerveSubsystem.getInstance().calculateInputs(new Vector(5, 0), 0);
//        }
        SwerveSubsystem.getInstance().applyCalculatedInputs();

        SmartDashboard.putNumber("Gyro", Gyro.getInstance().getHeadingRadians());

        double[] desiredAngles = SwerveSubsystem.getInstance().getDesiredAngles(),
                desiredMagnitudes = SwerveSubsystem.getInstance().getDesiredMagnitudes();
        SmartDashboard.putString("Wheel 1", new Vector(new Angle(desiredAngles[0], true),
                desiredMagnitudes[0]).toString());
        SmartDashboard.putString("Wheel 2", new Vector(new Angle(desiredAngles[1], true),
                desiredMagnitudes[1]).toString());
        SmartDashboard.putString("Wheel 3", new Vector(new Angle(desiredAngles[2], true),
                desiredMagnitudes[2]).toString());
        SmartDashboard.putString("Wheel 4", new Vector(new Angle(desiredAngles[3], true),
                desiredMagnitudes[3]).toString());

        SmartDashboard.putNumber("Wheel 1 angle", desiredAngles[0]);
        SmartDashboard.putNumber("Wheel 2 angle", desiredAngles[1]);
        SmartDashboard.putNumber("Wheel 3 angle", desiredAngles[2]);
        SmartDashboard.putNumber("Wheel 4 angle", desiredAngles[3]);

        SmartDashboard.putBoolean("Wheel 1 flip", SwerveSubsystem.getInstance().flipped[0]);
        SmartDashboard.putBoolean("Wheel 2 flip", SwerveSubsystem.getInstance().flipped[1]);
        SmartDashboard.putBoolean("Wheel 3 flip", SwerveSubsystem.getInstance().flipped[2]);
        SmartDashboard.putBoolean("Wheel 4 flip", SwerveSubsystem.getInstance().flipped[3]);

        SmartDashboard.putNumber("wheel 1 ticks", SwerveSubsystem.getInstance().getRawPosition(0));
        SmartDashboard.putNumber("wheel 1 position", SwerveSubsystem.getInstance().getEncoderModuleAngle(0));

        SmartDashboard.putNumber("wheel 1 velocity", SwerveSubsystem.getInstance().getRawWheelVelocity(0));
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
