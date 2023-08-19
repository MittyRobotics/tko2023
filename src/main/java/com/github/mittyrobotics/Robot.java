package com.github.mittyrobotics;

import com.github.mittyrobotics.arm.ArmMotionProfiles;
import com.github.mittyrobotics.arm.pivot.PivotSubsystem;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    @Override
    public void robotInit() {
        OI.getInstance().initHardware();
        Gyro.getInstance().initHardware();
//        SwerveSubsystem.getInstance().initHardware();
        PivotSubsystem.getInstance().initHardware();
//        TelevatorSubsystem.getInstance().initHardware();
//
//        Limelight.init(new Pose(0, 0, 0, false), 0);
//        Odometry.getInstance();
//
//        ArmSetpoints.initSetpoints();
        ArmMotionProfiles.createMPs();
    }

    @Override
    public void robotPeriodic() {
//        StateMachine.update(1, 1);
//        SwerveSubsystem.getInstance().updateForwardKinematics();
//        Limelight.update();
//        Odometry.getInstance().update();
//        ArmKinematics.updateDesiredArmPositionFromState();

//        CommandScheduler.getInstance().run();
        System.out.println("PIVOT POS: " + PivotSubsystem.getInstance().getCurrentAngle().getRadians());
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {
        super.autonomousPeriodic();
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
//        super.teleopPeriodic();
        PivotSubsystem.getInstance().setRaw(-0.2);
    }

    @Override
    public void disabledInit() {
        super.disabledInit();
    }

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();
    }
}
