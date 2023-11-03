package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class MidFlywheel extends CommandBase {
    private Shooter shooter;

    public MidFlywheel(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        shooter.setSpeed(Constants.ShooterConstants.MID_SHOOTER_RPM);

        SmartDashboard.putNumber("Current RPM", shooter.getVelocity());

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
