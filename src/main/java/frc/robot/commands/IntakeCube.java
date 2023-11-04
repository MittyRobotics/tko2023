package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class IntakeCube extends CommandBase {
    private Conveyor conveyor;

    private double initTime;
    private Double initPose = null;

    public IntakeCube(Conveyor conveyor) {
        this.conveyor = conveyor;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        initTime = Timer.getFPGATimestamp();
        initPose = null;
        SmartDashboard.putBoolean("Ended", false);
    }

    @Override
    public void execute() {
        conveyor.setMotor(0.8);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Ended", true);
        conveyor.setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return conveyor.getLimitSwitchTripped() || Timer.getFPGATimestamp() - initTime > 10;
    }
}
