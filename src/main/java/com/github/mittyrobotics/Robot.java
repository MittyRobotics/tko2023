package com.github.mittyrobotics;

import com.github.mittyrobotics.Auton.CreateTrajectory;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.function.Consumer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * 6trt * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public String auto = "balance";
    public boolean low = false, bal = false;

    @Override
    public void robotInit() {
        Gyro.getInstance().initHardware();
        SwerveSubsystem.getInstance().initHardware();
        SwerveSubsystem.getInstance().initPose(0, 0, 0);
    }

    @Override
    public void robotPeriodic() {
       SwerveSubsystem.getInstance().updatePose();
       CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
//        SwerveSubsystem.getInstance().setRelative();
//        SwerveSubsystem.getInstance().setRelative();
//        SwerveSubsystem.getInstance().setRelative();
//        SwerveSubsystem.getInstance().setRelative();
//        SwerveSubsystem.getInstance().setRelative();
//        SwerveSubsystem.getInstance().setRelative();
//        SwerveSubsystem.getInstance().setRelative();
//        SwerveSubsystem.getInstance().setRelative();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        System.out.println("X: " + SwerveSubsystem.getInstance().getRobotPose().getX() * 39.37 + " Y: " + SwerveSubsystem.getInstance().getRobotPose().getY() * 39.37);
//        System.out.println("ABSOLUTE: ");
//        for (int i = 0; i < 4; i++) {
//            System.out.print(SwerveSubsystem.getInstance().absEncPos()[i] + " ");;
//        }
//        System.out.println();
//        System.out.println("FALCONS: ");
//        for (int i = 0; i < 4; i++) {
//            System.out.print(SwerveSubsystem.getInstance().getAngles()[i] + " ");
//        }
//        System.out.println();
    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {

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
