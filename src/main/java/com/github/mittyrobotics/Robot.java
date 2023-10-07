package com.github.mittyrobotics;

import com.github.mittyrobotics.arm.ArmKinematics;
import com.github.mittyrobotics.arm.ArmMotionProfiles;
import com.github.mittyrobotics.arm.ArmSetpoints;
import com.github.mittyrobotics.arm.StateMachine;
import com.github.mittyrobotics.arm.pivot.PivotSubsystem;
import com.github.mittyrobotics.arm.televator.TelevatorSubsystem;
import com.github.mittyrobotics.autonomous.Limelight;
import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.NewPathFollowingCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.SwervePath;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.drivetrain.commands.SwerveDefaultCommand;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.OI;
import com.github.mittyrobotics.util.math.Angle;
import com.github.mittyrobotics.util.math.Point;
import com.github.mittyrobotics.util.math.Pose;
import com.github.mittyrobotics.util.math.autonomous.QuinticHermiteSpline;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static java.lang.Math.PI;

public class Robot extends TimedRobot {

    @Override
    public void robotInit() {
        OI.getInstance().initHardware();
        Gyro.getInstance().initHardware();
//        StateMachine.init();
        SwerveSubsystem.getInstance().initHardware();
//        PivotSubsystem.getInstance().initHardware();
//        TelevatorSubsystem.getInstance().initHardware();

        SwerveSubsystem.getInstance().forwardKinematics.init();
        Limelight.init(new Pose(0, 0, 0, false), 0);
        Odometry.getInstance();

//        ArmSetpoints.initSetpoints();
//        ArmMotionProfiles.createMPs();
    }

    @Override
    public void robotPeriodic() {
        Odometry.getInstance().FIELD_LEFT_SIDE = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue);
        SwerveSubsystem.getInstance().updateForwardKinematics();
        Limelight.update();
        Odometry.getInstance().update();
//        ArmKinematics.updateDesiredArmPositionFromState();
        System.out.println("POSE: " + Odometry.getInstance().getState());
        System.out.println("FLS: " + Odometry.getInstance().FIELD_LEFT_SIDE);

        // commented for safety
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        SwerveSubsystem.getInstance().setDefaultCommand(null);
        Gyro.getInstance().setAngleOffset(PI / 2, true);
        Pose start = new Pose(Odometry.getInstance().getState().getPoint(), new Angle(PI / 2, true));
        Pose one = new Pose(start.getPoint().getX() + 60, start.getPoint().getY(), -PI / 4, true);
        Pose two = new Pose(one.getPoint().getX() + 60, start.getPoint().getY(), PI / 2, true);
        Pose three = new Pose(one.getPoint().getX() - 60, start.getPoint().getY(), 5 * PI / 4, true);
        Pose four = new Pose(three.getPoint().getX() - 60, start.getPoint().getY(), PI / 2, true);
//        SmartDashboard.putString("start", start.toString());
//        SmartDashboard.putString("end", end.toString());

        SwervePath path1 = new SwervePath(
                new QuinticHermiteSpline(start, one),
                start.getHeading(), one.getHeading(),
                0, 0, 140, 120, 120, 0,
                0.2, 0.5, 1, 1,
                2, 0, 0.0001
        );
        SwervePath path2 = new SwervePath(
                new QuinticHermiteSpline(one, two),
                one.getHeading(), two.getHeading(),
                0, 0, 140, 160, 80, 0,
                0.2, 0.5, 1, 1,
                2, 0, 0.0001
        );
        SwervePath path3 = new SwervePath(
                new QuinticHermiteSpline(two, three),
                two.getHeading(), three.getHeading(),
                0, 0, 140, 120, 120, 0,
                0.2, 0.5, 1, 1,
                2, 0, 0.0001
        );
        SwervePath path4 = new SwervePath(
                new QuinticHermiteSpline(three, four),
                three.getHeading(), four.getHeading(),
                0, 0, 140, 120, 120, 0,
                0.2, 0.5, 1, 1,
                2, 0, 0.0001
        );

        SequentialCommandGroup group = new SequentialCommandGroup();
        group.addCommands(
                new NewPathFollowingCommand(path1),
                new NewPathFollowingCommand(path2),
                new NewPathFollowingCommand(path3),
                new NewPathFollowingCommand(path4)
        );

        group.schedule();
    }

    @Override
    public void autonomousPeriodic() {
        super.autonomousPeriodic();
    }

    @Override
    public void teleopInit() {
//        SwerveSubsystem.getInstance().zeroRelativeEncoders();
//        OI.getInstance().setupControls();
        SwerveSubsystem.getInstance().setDefaultCommand(new SwerveDefaultCommand());
    }

    @Override
    public void teleopPeriodic() {
//        super.teleopPeriodic();
//        PivotSubsystem.getInstance().setRaw(0.1);
//        TelevatorSubsystem.getInstance().setRaw(0.1);
//        PivotSubsystem.getInstance().setPosition(Math.PI / 4);
//        TelevatorSubsystem.getInstance().setPosition(5);
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
