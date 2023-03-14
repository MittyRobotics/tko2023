package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static com.github.mittyrobotics.autonomous.pathfollowing.PathFollowingConstants.*;

public class SwerveAutoDriveToPickupCommand extends CommandBase {
    private SwervePath path;
    private double linearThreshold, angularThreshold;
    private Pose robot;
    private PIDController angularController;
    private double speed = 0, targetAngle;

    boolean isCone;
    int index;

    public SwerveAutoDriveToPickupCommand(double linearThreshold, double angularThreshold, boolean isCone, int index, SwervePath path) {
        setName("Swerve Pure Pursuit");
        this.path = path;
        this.linearThreshold = linearThreshold;
        this.angularThreshold = angularThreshold;
        this.isCone = isCone;
        this.index = index;
        this.angularController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);

        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();
        speed = path.getInitSpeed();
        targetAngle = path.getEndHeading().getRadians();
    }

    @Override
    public void execute() {
        robot = Odometry.getInstance().getState();

        angularController = new PIDController(path.getKp(), path.getKi(), path.getKd());

        double leftY = OI.getInstance().getDriveController().getLeftX();
        double leftX = -OI.getInstance().getDriveController().getLeftY();
        speed = Math.sqrt(leftY * leftY + leftX * leftX) * SwerveConstants.MAX_LINEAR_VEL;

        double tempAngle = ArmKinematics.getAngleToGamePiece(isCone, index);
        if (!Double.isNaN(tempAngle)) targetAngle = tempAngle;
        Vector linearVel = new Vector(new Angle(-targetAngle), speed);

        double angularVel = angularController.calculate(-targetAngle);

        SwerveSubsystem.getInstance().setSwerveInvKinematics(linearVel, angularVel);

        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());
        SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().desiredAngles());
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().setZero();
        SmartDashboard.putBoolean("Halted", true);
    }

    @Override
    public boolean isFinished() {
        return StateMachine.getInstance().getIntakingState() == StateMachine.IntakeState.STOW;
    }
}
