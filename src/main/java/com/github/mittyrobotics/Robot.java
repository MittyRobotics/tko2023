package com.github.mittyrobotics;

import com.github.mittyrobotics.arm.ArmKinematics;
import com.github.mittyrobotics.arm.ArmSetpoints;
import com.github.mittyrobotics.arm.MotionProfiles;
import com.github.mittyrobotics.arm.pivot.PivotSubsystem;
import com.github.mittyrobotics.arm.televator.TelevatorSubsystem;
import com.github.mittyrobotics.autonomous.Limelight;
import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.math.Pose;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    @Override
    public void robotInit() {
        SwerveSubsystem.getInstance().initHardware();
        PivotSubsystem.getInstance().initHardware();
        TelevatorSubsystem.getInstance().initHardware();
        Limelight.init(new Pose(0, 0, 0, false), 0);
        ArmSetpoints.initSetpoints();
        MotionProfiles.createMPs();
    }

    @Override
    public void robotPeriodic() {
//        StateMachine.update(1, 1);
        Odometry.getInstance().update();
        ArmKinematics.updateDesiredArmPositionFromState();

        CommandScheduler.getInstance().run();
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
        super.teleopPeriodic();
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
