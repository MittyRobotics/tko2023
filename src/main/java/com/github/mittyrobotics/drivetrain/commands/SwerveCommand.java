package com.github.mittyrobotics.drivetrain.commands;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveCommand extends CommandBase {

    double leftX, leftY, rightX, rightY, leftTrigger, rightTrigger;

    boolean notMoving;

    double linearVelAngle, linearVelMagnitude, angularVel, heading;

    Vector linearVel;

    boolean disabled;

    public SwerveCommand() {
        addRequirements(SwerveSubsystem.getInstance(), Gyro.getInstance());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        leftY = OI.getInstance().getPS4Controller().getLeftX();
        leftX = -OI.getInstance().getPS4Controller().getLeftY();
//        rightY = OI.getInstance().getDriveController().getRightX();
        rightX = -OI.getInstance().getPS4Controller().getRightX();
//        leftTrigger = OI.getInstance().getDriveController().getLeftTriggerAxis();
        rightTrigger = (OI.getInstance().getPS4Controller().getR2Axis() + 1) / 2.;
//        System.out.println(rightTrigger);
//        System.out.println("leftx: " +  leftX);

        notMoving = false;

        if(rightTrigger < 0.001 && leftX < 0.001 && leftY < 0.001) {
            notMoving = true;
        }



        if (Math.abs(leftX) < SwerveConstants.JOYSTICK_DEADZONE) leftX = 0;
        if (Math.abs(leftY) < SwerveConstants.JOYSTICK_DEADZONE) leftY = 0;
        if (Math.abs(rightX) < SwerveConstants.JOYSTICK_DEADZONE) rightX = 0;
        if (Math.abs(rightY) < SwerveConstants.JOYSTICK_DEADZONE) rightY = 0;
        if (Math.abs(rightTrigger) < SwerveConstants.TRIGGER_THRESHOLD) rightTrigger = 0;

        disabled = false;

        if ((rightTrigger == 0 || (leftX == 0 && leftY == 0)) && rightX == 0) {
            disabled = true;
        }

        heading = Gyro.getInstance().getHeadingRadians();

        double angle = Math.atan2(leftY, leftX) + heading;
        double throttle = rightTrigger * SwerveConstants.MAX_LINEAR_VEL;
        if(disabled) throttle = 0;

        linearVel = new Vector(
                Math.cos(angle) * throttle, Math.sin(angle) * throttle
        );

        if(rightX < 0) {
            angularVel = -(Math.pow(rightX, 4) * SwerveConstants.MAX_ANGULAR_VEL);
        } else angularVel = Math.pow(rightX, 4) * SwerveConstants.MAX_ANGULAR_VEL;

        angularVel = -angularVel;

//        System.out.println("inputs: " + linearVel + " " + angularVel);

        SwerveSubsystem.getInstance().setSwerveInvKinematics(Vector.multiply(OI.getInstance().getPS4Controller().getR1Button() ? 2 : 1, linearVel), angularVel);

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
