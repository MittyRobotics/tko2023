package com.github.mittyrobotics.drivetrain.commands;

import com.github.mittyrobotics.LoggerInterface;
import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickThrottleCommand extends CommandBase {

    double fieldX, fieldY, rightX, rightY, leftTrigger, rightTrigger;

    boolean a, b, x, y;

    boolean notMoving;

    double angularVel, heading;

    Vector linearVel;

    boolean disabled;

    PIDController controller = new PIDController(SwerveConstants.ANGLE_LOCK_P,
            SwerveConstants.ANGLE_LOCK_I, SwerveConstants.ANGLE_LOCK_D);

    int anglePreset = 2;
    double currentAngle = 0, currentDesired = 0;

    public JoystickThrottleCommand() {
        this.anglePreset = anglePreset;
        addRequirements(SwerveSubsystem.getInstance(), Gyro.getInstance());
    }

    @Override
    public void initialize() {
        controller.setSetpoint(0);
    }

    @Override
    public void execute() {
        a = OI.getInstance().getDriveController().getAButton();
//        b = OI.getInstance().getPS4Controller().getCircleButton();
//        x = OI.getInstance().getPS4Controller().getSquareButton();
        y = OI.getInstance().getDriveController().getYButton();

        fieldY = OI.getInstance().getDriveController().getLeftX() * (Odometry.getInstance().FIELD_LEFT_SIDE ? -1 : 1);
        fieldX = -OI.getInstance().getDriveController().getLeftY() * (Odometry.getInstance().FIELD_LEFT_SIDE ? 1 : -1);

        rightX = OI.getInstance().getDriveController().getRightX();
        rightTrigger = OI.getInstance().getDriveController().getRightTriggerAxis();

        notMoving = false;

        if(rightTrigger < SwerveConstants.JOYSTICK_DEADZONE && fieldX < SwerveConstants.JOYSTICK_DEADZONE && fieldY < SwerveConstants.JOYSTICK_DEADZONE && !OI.getInstance().getPS4Controller().getCircleButton()) {
            notMoving = true;
        }

        if (Math.abs(fieldX) < SwerveConstants.JOYSTICK_DEADZONE) fieldX = 0;
        if (Math.abs(fieldY) < SwerveConstants.JOYSTICK_DEADZONE) fieldY = 0;
        if (Math.abs(rightX) < SwerveConstants.JOYSTICK_DEADZONE) rightX = 0;
        if (Math.abs(rightY) < SwerveConstants.JOYSTICK_DEADZONE) rightY = 0;
        if (Math.abs(rightTrigger) < SwerveConstants.TRIGGER_THRESHOLD) rightTrigger = 0;

        disabled = false;
        if ((fieldX == 0 && fieldY == 0) && rightX == 0 && !OI.getInstance().getPS4Controller().getCircleButton()) {
            disabled = true;
        }

        heading = Gyro.getInstance().getHeadingRadians();

        double input = Math.sqrt(fieldY * fieldY + fieldX * fieldX);
        double throttle;

        throttle = Math.pow(input, 2) * (OI.getInstance().getDriveController().getRightBumper() ? SwerveConstants.MAX_BOOST_LINEAR_VEL :
                        SwerveConstants.MAX_LINEAR_VEL);

        double angle_field = Math.atan2(fieldY, fieldX);
        double robot_relative_angle = angle_field - heading;

        if(disabled) throttle = 0;

        linearVel = new Vector(
                new Angle(robot_relative_angle), throttle
        );

        if(rightX < 0) {
            angularVel = (Math.pow(rightX, 2) * SwerveConstants.MAX_ANGULAR_VEL);
        } else angularVel = -Math.pow(rightX, 2) * SwerveConstants.MAX_ANGULAR_VEL;

        SmartDashboard.putNumber("rightx", rightX);
        SmartDashboard.putNumber("angular vel", angularVel);

        if(y || a) {
            currentAngle = SwerveSubsystem.standardize(Gyro.getInstance().getHeadingRadians());
            currentDesired = SwerveSubsystem.standardize((y ? 0 : Math.PI) + (Odometry.getInstance().FIELD_LEFT_SIDE ? 0 : Math.PI));

            boolean right;
            double dist;

            if (currentDesired == Math.PI) {
                if (currentAngle < Math.PI) {
                    right = true;
                    dist = Math.PI - currentAngle;
                } else {
                    right = false;
                    dist = currentAngle - Math.PI;
                }
            } else {
                if (currentAngle > Math.PI) {
                    right = true;
                    dist = 2 * Math.PI - currentAngle;
                } else {
                    right = false;
                    dist = currentAngle;
                }
            }

            angularVel = -controller.calculate(dist * (right ? 1 : -1), 0);
            disabled = false;
        }

//        LoggerInterface.getInstance().put("Desired linear velocity", linearVel.toString());
//        LoggerInterface.getInstance().put("Desired angular velocity", angularVel);

//        System.out.println("Linear: " + linearVel);
        SwerveSubsystem.getInstance().setSwerveInvKinematics(Vector.multiply(1, linearVel), angularVel);

        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());

        if(disabled && DriverStation.getMatchTime() < 15. && DriverStation.getMatchTime() != -1.) {
            SwerveSubsystem.getInstance().fortyFiveAngle();
        } else if(!disabled) {
            SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().desiredAngles());
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
