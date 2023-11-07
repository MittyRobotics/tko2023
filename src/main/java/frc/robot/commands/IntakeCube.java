package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class IntakeCube extends CommandBase {
    private Conveyor conveyor;

    private double initTime;

    public IntakeCube(Conveyor conveyor) {
        this.conveyor = conveyor;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        initTime = Timer.getFPGATimestamp();
        SmartDashboard.putBoolean("Ended", false);
    }

    @Override
    public void execute() {
        conveyor.setMotor(0.6);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Ended", true);
        conveyor.setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return conveyor.getLimitSwitchTripped();
    }
}
