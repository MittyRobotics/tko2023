package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class HighFlywheel extends CommandBase {
    private Shooter shooter;

    public HighFlywheel(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        shooter.setSpeed(Constants.ShooterConstants.HIGH_SHOOTER_RPM);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
