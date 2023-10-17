package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.util.math.Pose;
import frc.robot.util.math.Vector;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveDefaultCommand extends CommandBase {
    private Swerve swerve;
    private PoseEstimator poseEstimator;

    private DoubleSupplier xSupplier, ySupplier, angularSupplier;
    private BooleanSupplier rBumperSupplier, lBumperSupplier, aSupplier;
    private double throttleX, throttleY, throttleAngular, joystickDeadzone;
    private boolean rightBumper, leftBumper, a;

    public SwerveDefaultCommand(Swerve swerve, PoseEstimator poseEstimator,
                                DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier angularSupplier,
                                BooleanSupplier rBumperSupplier, BooleanSupplier lBumperSupplier, BooleanSupplier aSupplier) {

        this.swerve = swerve;
        this.poseEstimator = poseEstimator;

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.angularSupplier = angularSupplier;

        this.rBumperSupplier = rBumperSupplier;
        this.lBumperSupplier = lBumperSupplier;
        this.aSupplier = aSupplier;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        throttleX = 0;
        throttleY = 0;
        throttleAngular = 0;
        joystickDeadzone = 0.05;
        rightBumper = false;
        leftBumper = false;
        a = false;
    }

    @Override
    public void execute() {
        throttleX = (poseEstimator.FIELD_LEFT_SIDE ? 1 : -1) * xSupplier.getAsDouble();
        throttleY = (poseEstimator.FIELD_LEFT_SIDE ? -1 : 1) * ySupplier.getAsDouble();
        throttleAngular = -angularSupplier.getAsDouble();
        rightBumper = rBumperSupplier.getAsBoolean();
        leftBumper = lBumperSupplier.getAsBoolean();
        a = aSupplier.getAsBoolean();

        if (Math.abs(throttleX) < joystickDeadzone) throttleX = 0;
        if (Math.abs(throttleY) < joystickDeadzone) throttleY = 0;
        if (Math.abs(throttleAngular) < joystickDeadzone) throttleAngular = 0;

//        System.out.println("FLS: " + Odometry.getInstance().FIELD_LEFT_SIDE + ", X: " + throttleX);

//        System.out.printf("X: %.2f, Y: %.2f, A: %.2f", throttleX, throttleY, throttleAngular);
//        System.out.println();
        if (a) {
            swerve.setZero();
            swerve.lockWheels();
        } else if (throttleX == 0 && throttleY == 0 && throttleAngular == 0) {
            swerve.setZero();
        } else {
            swerve.calculateInputs(
                    new Vector(
                            Constants.SwerveConstants.MAX_LINEAR_SPEED_INCHES_PER_SECOND * Math.abs(throttleX) * throttleX * (rightBumper ? 1.5 : 1),
                            Constants.SwerveConstants.MAX_LINEAR_SPEED_INCHES_PER_SECOND * Math.abs(throttleY) * throttleY * (rightBumper ? 1.5 : 1)
                    ),
                    Constants.SwerveConstants.MAX_ANGULAR_SPEED * throttleAngular * (leftBumper ? 1.5 : 1)
            );
            swerve.applyCalculatedInputs();
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}