package com.github.mittyrobotics.Auton.Commands;

import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.function.Consumer;

public class FollowTrajectoryCommand {

    private Trajectory t;
    private double desHeading, maxAngVel, maxAngAccel;
    private double[] X_PID, Y_PID, ANGULAR_PID;
    public FollowTrajectoryCommand(Trajectory t, double desHeading, double[] X_PID, double[] Y_PID, double[] ANGULAR_PID, double maxAngVel, double maxAngAccel) {
        this.t = t;
        this.desHeading = desHeading;
        this.X_PID = X_PID;
        this.Y_PID = Y_PID;
        this.ANGULAR_PID = ANGULAR_PID;
        this.maxAngVel = maxAngVel;
        this.maxAngAccel = maxAngAccel;
    }


    public Command get() {
        PIDController xController = new PIDController(X_PID[0], X_PID[1], X_PID[2]);
        PIDController yController = new PIDController(Y_PID[0], Y_PID[1], Y_PID[2]);
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxAngVel, maxAngAccel);
        ProfiledPIDController rotationController = new ProfiledPIDController(ANGULAR_PID[0], ANGULAR_PID[1], ANGULAR_PID[2], constraints);

        HolonomicDriveController controller = new HolonomicDriveController(xController, yController, rotationController);

        Consumer<SwerveModuleState[]> outputModuleStates = (State) -> SwerveSubsystem.getInstance().setModuleStates(State);

        return new SwerveControllerCommand(t, SwerveSubsystem.getInstance()::getRobotPose, SwerveSubsystem.getInstance().getKinematics(), controller, () -> new Rotation2d(desHeading), outputModuleStates, SwerveSubsystem.getInstance());

    }


}
