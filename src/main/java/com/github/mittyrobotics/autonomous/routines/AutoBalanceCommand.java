package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalanceCommand extends CommandBase {
    PIDController controller = new PIDController(SwerveConstants.BALANCE_P, SwerveConstants.BALANCE_I, SwerveConstants.BALANCE_D);
    public AutoBalanceCommand() {
        addRequirements(SwerveSubsystem.getInstance());
    }
    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        double pitch = Gyro.getInstance().getPitch();
        double output = controller.calculate(pitch, 0);
        SwerveSubsystem.getInstance().setSwerveInvKinematics(new Vector(output, 0), 0);

        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());
        SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().desiredAngles());
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
