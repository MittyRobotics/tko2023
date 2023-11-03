package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class BringCubeToHolding extends CommandBase {
    private Conveyor conveyor;

    private double initTime;
    private Double initPose = null;

    public BringCubeToHolding(Conveyor conveyor) {
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
        conveyor.setMotor(1.0);

        if (initPose == null && conveyor.getLimitSwitchTripped()) {
            initPose = conveyor.getPosition();
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Ended", true);
        conveyor.setMotor(0);
    }

    @Override
    public boolean isFinished() {
        boolean condition = initPose != null && conveyor.getPosition() - initPose > 0;
        SmartDashboard.putBoolean("isFinished", condition);
        return condition || Timer.getFPGATimestamp() - initTime > 10;
    }
}
