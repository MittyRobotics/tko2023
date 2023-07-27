package com.github.mittyrobotics.drivetrain.commands;

import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDefaultCommand extends CommandBase {
    private double throttleX, throttleY, throttleAngular;
    public SwerveDefaultCommand() {
        setName("Swerve Default Command");
        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        throttleX = 0;
        throttleY = 0;
        throttleAngular = 0;
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
