package com.github.mittyrobotics.drivetrain.commands;

import com.github.mittyrobotics.drivetrain.SwerveConstants;
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
