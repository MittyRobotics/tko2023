package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class StopFlywheel extends CommandBase {
    private Shooter shooter;

    public StopFlywheel(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        shooter.setSpeed(0);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return shooter.getVelocityError() < 5;
    }
}
