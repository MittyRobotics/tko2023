package com.github.mittyrobotics;

import com.github.mittyrobotics.autonomous.Limelight;
import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.arm.AutoArmScoreCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.autonomous.pathfollowing.v2.PathFollowingCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.v2.SwervePath;
import com.github.mittyrobotics.autonomous.routines.PPOneAuto;
import com.github.mittyrobotics.autonomous.routines.PPTwoAuto;
import com.github.mittyrobotics.autonomous.routines.PTaxi;
import com.github.mittyrobotics.autonomous.routines.PreloadAndBalanceAuto;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.intake.IntakeSubsystem;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.led.LedSubsystem;
import com.github.mittyrobotics.pivot.PivotSubsystem;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.OI;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.ejml.All;

import java.util.Objects;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * 6trt * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public String auto = "balance";
    public String low = "DOWN", bal = "YES";

    private SendableChooser<String> sideChooser = new SendableChooser<>();
    private SendableChooser<String> autoChooser = new SendableChooser<>();
    private SendableChooser<String> balChooser = new SendableChooser<>();
    private SendableChooser<String> heightChooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        sideChooser.setDefaultOption("Blue", "LEFT");
        sideChooser.addOption("Red", "RIGHT");
        SmartDashboard.putData("side", sideChooser);

        autoChooser.setDefaultOption("Preload only", "PRELOAD");
        autoChooser.addOption("Pick one piece", "BALANCE");
        autoChooser.addOption("Pick two pieces", "PICK");
        SmartDashboard.putData("auto", autoChooser);

        balChooser.setDefaultOption("Neither", "NO");
        balChooser.addOption("Balance", "BAL");
        balChooser.addOption("Taxi", "TAXI");
        SmartDashboard.putData("balance", balChooser);

        heightChooser.setDefaultOption("Non-bump side (HP side)", "UP");
        heightChooser.addOption("Bump side (away from HP)", "DOWN");
        SmartDashboard.putData("low", heightChooser);

        Limelight.init(null, 0);

        LedSubsystem.getInstance().initHardware();
        Gyro.getInstance().initHardware();
        SwerveSubsystem.getInstance().initHardware();

        TelescopeSubsystem.getInstance().initHardware();
        PivotSubsystem.getInstance().initHardware();
        IntakeSubsystem.getInstance().initHardware();

        PivotSubsystem.getInstance().setCoastMode();
        TelescopeSubsystem.getInstance().setCoastMode();

        Odometry.getInstance().FIELD_LEFT_SIDE = true;
        double x = 0;
        double y = 0;
        double t = 0;
        Odometry.getInstance().setState(x, y, t);
        SwerveSubsystem.getInstance().forwardKinematics.init();
//        Odometry.getInstance().setScoringCam(true);
        Odometry.getInstance().update();

//        SwerveSubsystem.getInstance().zeroRelativeEncoders();
    }

    @Override
    public void robotPeriodic() {
        Odometry.getInstance().FIELD_LEFT_SIDE = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue);
        SwerveSubsystem.getInstance().updateForwardKinematics();

        //UPDATE FROM BEAGLE AND JETSON
        Odometry.getInstance().update();
        Limelight.updateClosestTag();
        Limelight.updatePose();
//        ArmKinematics.updateAngleToGamePiece(StateMachine.getInstance().getCurrentPieceState()
//                == StateMachine.PieceState.CONE, 0);

//        LoggerInterface.getInstance().put("Heading", Gyro.getInstance().getHeadingRadians());
//        LoggerInterface.getInstance().put("Pose", Odometry.getInstance().getState());

//        LoggerInterface.getInstance().put("Pose X", Odometry.getInstance().getPose()[0]);
//        LoggerInterface.getInstance().put("Pose Y", Odometry.getInstance().getPose()[1]);

//        LoggerInterface.getInstance().put("Spark Current", IntakeSubsystem.getInstance().getCurrent());
//        System.out.println("CURRENT: " + IntakeSubsystem.getInstance().getCurrent());

        CommandScheduler.getInstance().run();

//        System.out.print(Odometry.getInstance().getState().getPosition());
//        System.out.println("   Angle: " + Gyro.getInstance().getHeadingRadians());
//        System.out.println(Limelight.getPose());
//        System.out.println("FLS: " + Odometry.getInstance().FIELD_LEFT_SIDE);
    }

    @Override
    public void autonomousInit() {
        PivotSubsystem.getInstance().setBrakeMode();
        TelescopeSubsystem.getInstance().setBrakeMode();
//        SwerveSubsystem.getInstance().zeroRelativeEncoders();
        Limelight.setCheckingGyro(true);
//        Gyro.getInstance().setAngleOffset(0, true);
        Limelight.setAngleOffset();
//        Odometry.getInstance().FIELD_LEFT_SIDE = Limelight.getClosestTag() >= 5;
//        Odometry.getInstance().FIELD_LEFT_SIDE = true;
//        Gyro.getInstance().setAngleOffset(180, false);
//        Odometry.getInstance().FIELD_LEFT_SIDE = LoggerInterface.getInstance().getValue("fieldside").equals("left");

//        auto = SmartDashboard.getString("auto", "N");
//        low = SmartDashboard.getString()
//        auto = LoggerInterface.getInstance().getValue("auto");
//        low = LoggerInterface.getInstance().getValue("autoside").equals("L");
//        bal = LoggerInterface.getInstance().getValue("autobal").equals("T");

//        auto = "balance";
//        low = true;
//        bal = false;

//        Odometry.getInstance().FIELD_LEFT_SIDE = sideChooser.getSelected().equals("LEFT");
        auto = autoChooser.getSelected();
        bal = balChooser.getSelected();
        low = heightChooser.getSelected();

        SmartDashboard.putString("TESTS", String.valueOf(Odometry.getInstance().FIELD_LEFT_SIDE));
//        SmartDashboard.putString("TESTA", auto);
        SmartDashboard.putString("TESTB", bal);
        SmartDashboard.putBoolean("TESTL", low.equals("DOWN"));

        switch(auto) {
            case "PRELOAD":
//                new PreloadAndBalanceAuto(Odometry.getInstance().FIELD_LEFT_SIDE, bal.equals("BAL"), bal.equals("TAXI"), low.equals("DOWN")).schedule();
                SmartDashboard.putString("TESTA", "P+B");
                break;
            case "BALANCE":
//                new PPOneAuto(low.equals("DOWN"), Odometry.getInstance().FIELD_LEFT_SIDE, StateMachine.PieceState.CUBE, bal.equals("YES")).schedule();
//                new PTaxi(low.equals("DOWN"), Odometry.getInstance().FIELD_LEFT_SIDE, StateMachine.PieceState.CONE).schedule();
                SmartDashboard.putString("TESTA", "PPOne");
                break;
            case "PICK":
//                new PPTwoAuto(low.equals("DOWN"), Odometry.getInstance().FIELD_LEFT_SIDE).schedule();
//                new PTaxi(low.equals("DOWN"), Odometry.getInstance().FIELD_LEFT_SIDE, StateMachine.PieceState.CONE).schedule();
                SmartDashboard.putString("TESTA", "PPTwo");
                break;
        }

//        new PTaxi(false, Odometry.getInstance().FIELD_LEFT_SIDE, StateMachine.PieceState.CONE).schedule();
//        new PreloadAndBalanceAuto(Odometry.getInstance().FIELD_LEFT_SIDE, false, false, true).schedule();
//        new PreloadAndBalanceAuto(Odometry.getInstance().FIELD_LEFT_SIDE, true, false, true).schedule();
//        new PPOneAuto(true, Odometry.getInstance().FIELD_LEFT_SIDE, StateMachine.PieceState.CUBE, false);

//                new PreloadAndBalanceAuto(Odometry.getInstance().FIELD_LEFT_SIDE, true, bal.equals("TAXI"), low.equals("DOWN")).schedule();

        new AutoArmScoreCommand(StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE).schedule();

//        com.github.mittyrobotics.util.math.Pose p = Odometry.getInstance().getState();
//        Pose start = new Pose(new Point(p.getPoint().getX(), p.getPoint().getY()), new Angle(1 * Math.PI));
//        Pose end = new Pose(new Point(start.getPosition().getX() - 150, start.getPosition().getY() + 30), new Angle(Math.PI));
//        new PathFollowingCommand(
//                new SwervePath(
//                        new QuinticHermiteSpline(start, end),
//                        8, 100, 200, 200, 0, 0, true, false
//                ), Math.PI/2, 3, 0.05,
//                0, 0.6, 0.75, 0, 0.01, true
//        ).schedule();
//        );


//        Odometry.getInstance().FIELD_LEFT_SIDE = false;
//
//        new PPTwoAuto(false, false).schedule();

//        Pose init = Odometry.getInstance().getState();
//        Pose end = new Pose(Point.add(init.getPosition(), new Point(150, 100)), init.getHeading());
//        SwervePath path = new SwervePath(new QuinticHermiteSpline(init, end),
//                15, 3, 3, 3, 0, 0, true);
//        new PathFollowingCommand(path, init.getHeading().getRadians() + Math.PI, 5, 0.05,
//                0.1, 0.8, 6, 0, 0.02).schedule();

//        new AutoScoreCommand(8, 0, 3, 3, 3, 0, 0).schedule();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
//        Limelight.updatePose();
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        Limelight.setCheckingGyro(true);
        Limelight.setAngleOffset();
        PivotSubsystem.getInstance().setBrakeMode();
        TelescopeSubsystem.getInstance().setBrakeMode();
//        Gyro.getInstance().setAngleOffset(3.141592, true);
//        for (int i = 0; i < 4; i++) {
//            SwerveSubsystem.getInstance().angleMotors[i].setSelectedSensorPosition(0);
//        }
//        SwerveSubsystem.getInstance().setAllAngleEncodersZero();
//        SwerveSubsystem.getInstance().zeroRelativeEncoders();
//        SwerveSubsystem.getInstance().setRampRate(0.5);
//        Odometry.getInstance().FIELD_LEFT_SIDE = Limelight.getClosestTag() >= 5;
//        Odometry.getInstance().FIELD_LEFT_SIDE = false;
        OI.getInstance().setupControls();
        OI.getInstance().zeroAll();
        StateMachine.getInstance().setIntakeOff();


//        new InitAutoCommand(new Pose(new Point(40.45 + 32, 42.19 - 20.873), new Angle(Math.PI))).schedule();

//        SwerveSubsystem.getInstance().setRampRate(0);
//        new FastOvershootBalance(3, 0.7, false).schedule();
//        new Balance(false).schedule();
        //for false
//        new TimedBangBangBalance(2.5, 650, 0.3, false).schedule();
        //for true
//        new TimedBangBangBalance(2, 700, 0.5, true).schedule();


    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
//        Limelight.updatePose();
//        System.out.println("angles: " + SwerveSubsystem.getInstance().getAngl);
//        System.out.println(OI.getInstance().getOperatorController().getRightTriggerAxis());
//        System.out.println("POSE: " + Odometry.getInstance().getState());
//        System.out.println("POSE: " + Limelight.getPose());
//        LoggerInterface.getInstance().put("AGNEL TO GP", ArmKinematics.getSplineToGamePiece());
//    System.out.println("ANGLE: " + PivotSubsystem.getInstance().getPositionRadians());
//    System.out.println("RADIUS: "  + TelescopeSubsystem.getInstance().getDistanceMeters());
    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
        SwerveSubsystem.getInstance().setAngleCoastMode();
        PivotSubsystem.getInstance().setCoastMode();
        TelescopeSubsystem.getInstance().setCoastMode();

        LedSubsystem.getInstance().turnOff();

//        SwerveSubsystem.getInstance().setAnglesZero();
    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
//        OI.getInstance().getDriveController().setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
//        OI.getInstance().getDriveController().setRumble(GenericHID.RumbleType.kRightRumble, 0.5);
//        System.out.println(LoggerInterface.getInstance().getValue("fieldside"));
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}
