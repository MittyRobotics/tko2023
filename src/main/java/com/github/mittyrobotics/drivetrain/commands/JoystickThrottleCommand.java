package com.github.mittyrobotics.drivetrain.commands;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickThrottleCommand extends CommandBase {

    double leftX, leftY, rightX, rightY, leftTrigger, rightTrigger;

    boolean notMoving;

    double linearVelAngle, linearVelMagnitude, angularVel, heading;

    Vector linearVel;

    boolean disabled;

    public JoystickThrottleCommand() {
        addRequirements(SwerveSubsystem.getInstance(), Gyro.getInstance());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        leftY = OI.getInstance().getDriveController().getLeftX();
        leftX = -   OI.getInstance().getDriveController().getLeftY();
//        rightY = OI.getInstance().getDriveController().getRightX();
        rightX = -OI.getInstance().getDriveController().getRightX();
//        leftTrigger = OI.getInstance().getDriveController().getLeftTriggerAxis();
        rightTrigger = OI.getInstance().getDriveController().getRightTriggerAxis();
//        System.out.println(rightTrigger);
//        System.out.println("leftx: " +  leftX);

        notMoving = false;

        if(rightTrigger < 0.001 && leftX < 0.001 && leftY < 0.001 && !OI.getInstance().getPS4Controller().getCircleButton()) {
            notMoving = true;
        }



        if (Math.abs(leftX) < SwerveConstants.JOYSTICK_DEADZONE) leftX = 0;
        if (Math.abs(leftY) < SwerveConstants.JOYSTICK_DEADZONE) leftY = 0;
        if (Math.abs(rightX) < SwerveConstants.JOYSTICK_DEADZONE) rightX = 0;
        if (Math.abs(rightY) < SwerveConstants.JOYSTICK_DEADZONE) rightY = 0;
        if (Math.abs(rightTrigger) < SwerveConstants.TRIGGER_THRESHOLD) rightTrigger = 0;

        disabled = false;

        if ((leftX == 0 && leftY == 0) && rightX == 0 && !OI.getInstance().getPS4Controller().getCircleButton()) {
            disabled = true;
        }

        heading = Gyro.getInstance().getHeadingRadians();

        double angle = Math.atan2(leftY, leftX) + heading;
        double throttle = Math.sqrt(leftY * leftY + leftX * leftX) * SwerveConstants.MAX_LINEAR_VEL;
        if (OI.getInstance().getPS4Controller().getCircleButton())
            throttle /= 5;
        if(disabled) throttle = 0;

        linearVel = new Vector(
                new Angle(angle), throttle
        );

//        if (OI.getInstance().getPS4Controller().getCircleButton()) {
//            linearVel = new Vector(new Angle(heading), 0);
//        }

        if(rightX < 0) {
            angularVel = -(Math.pow(rightX, 4) * SwerveConstants.MAX_ANGULAR_VEL);
        } else angularVel = Math.pow(rightX, 4) * SwerveConstants.MAX_ANGULAR_VEL;

        angularVel = -angularVel;

//        System.out.println("inputs: " + linearVel + " " + angularVel);

        SwerveSubsystem.getInstance().setSwerveModule(Vector.multiply(OI.getInstance().getPS4Controller().getR1Button() ? SwerveConstants.BOOST_THROTTLE : 1, linearVel), angularVel);

//        System.out.println(linearVel.toStringMetric() + ", " + angularVel);

        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());

        if(!disabled) {
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
