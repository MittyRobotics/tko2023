// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.*;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Gyro gyro;
    private final Limelight limelight;
    private final Shooter shooter;
    private final Intake intake;
    private final Conveyor conveyor;
    private final Swerve swerve;
    private final PoseEstimator poseEstimator;
    private final AutoSelector autoSelector;

    private final Command midFlywheel;
    private final Command highFlywheel;
    private final Command unloadConveyor;
    private final Command bringCubeToHolding;
    private final Command lowerIntake;
    private final Command raiseIntake;
    private final Command zeroIntake;


    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final XboxController driverController =
            new XboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operatorController =
            new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        gyro = new Gyro();
        limelight = new Limelight();
        shooter = new Shooter();
        intake = new Intake();
        conveyor = new Conveyor();
        swerve = new Swerve(gyro);
        poseEstimator = new PoseEstimator(limelight, swerve);
        autoSelector = new AutoSelector(swerve, gyro, poseEstimator,
                conveyor, shooter, intake);

        midFlywheel = new MidFlywheel(shooter);
        highFlywheel = new HighFlywheel(shooter);
        unloadConveyor = new UnloadConveyor(conveyor);
        bringCubeToHolding = new BringCubeToHolding(conveyor);
        lowerIntake = new LowerIntake(intake);
        raiseIntake = new RaiseIntake(intake);
        zeroIntake = new ZeroIntake(intake);

        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        swerve.setDefaultCommand(new SwerveDefaultCommand(
                swerve, poseEstimator,
                driverController::getLeftY, driverController::getLeftY, driverController::getRightX,
                driverController::getRightBumper, driverController::getLeftBumper, driverController::getAButton
        ));

        operatorController.a().onTrue(bringCubeToHolding);
        new Trigger(() ->
                operatorController.b().getAsBoolean() &&
                        conveyor.getLimitSwitchTripped() &&
                        shooter.getVelocityError() < ShooterConstants.THRESHOLD
        ).whileTrue(unloadConveyor);
        new Trigger(() -> operatorController.getRightTriggerAxis() > 0.5).whileTrue(highFlywheel);
        operatorController.rightBumper().onTrue(lowerIntake);
        operatorController.leftBumper().onTrue(raiseIntake);
        operatorController.rightStick().onTrue(zeroIntake);
    }

    public void robotPeriodic() {
        poseEstimator.FIELD_LEFT_SIDE = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue);
    }

    public void autoInit() {
        if (!intake.hasBeenZeroed()) zeroIntake.schedule();
    }

    public void teleopInit() {
        if (!intake.hasBeenZeroed()) zeroIntake.schedule();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
//        return Autos.exampleAuto(m_exampleSubsystem);
        return autoSelector.getAuto();
    }
}