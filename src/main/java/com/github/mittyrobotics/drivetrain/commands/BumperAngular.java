package com.github.mittyrobotics.drivetrain.commands;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BumperAngular extends CommandBase {
        double leftX, leftY, rightX, rightY, leftTrigger, rightTrigger;

        boolean notMoving, rightBumper, leftBumper;

        double linearVelAngle, linearVelMagnitude, angularVel, heading;

        Vector linearVel;

        boolean disabled;

    public BumperAngular() {
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

        leftTrigger = (OI.getInstance().getPS4Controller().getL2Axis() + 1) / 2.;
        rightTrigger = (OI.getInstance().getPS4Controller().getR2Axis() + 1) / 2.;

        leftBumper = (OI.getInstance().getPS4Controller().getL1Button());
        rightBumper = (OI.getInstance().getPS4Controller()).getR1Button();
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
        if (Math.abs(leftTrigger) < SwerveConstants.TRIGGER_THRESHOLD) leftTrigger = 0;
        if (Math.abs(rightTrigger) < SwerveConstants.TRIGGER_THRESHOLD) rightTrigger = 0;

        disabled = false;

        if (!OI.getInstance().getPS4Controller().getCircleButton() && leftX == 0 && leftY == 0 && leftTrigger < SwerveConstants.TRIGGER_THRESHOLD && rightTrigger < SwerveConstants.TRIGGER_THRESHOLD) {
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
//            linearVel = new Vector(new Angle(heading), 0.1);
//        }

//        angularVel = leftTrigger > SwerveConstants.TRIGGER_THRESHOLD ? leftTrigger * SwerveConstants.MAX_ANGULAR_VEL : -1 * rightTrigger * SwerveConstants.MAX_ANGULAR_VEL;

        if(rightBumper) {
            angularVel = SwerveConstants.BUMPER_ANGULAR_VEL;
        } else if(leftBumper) {
            angularVel = -SwerveConstants.BUMPER_ANGULAR_VEL;
        } else {
            angularVel = 0;
        }

//        if(rightX < 0) {
//            angularVel = -(Math.pow(rightX, 4) * SwerveConstants.MAX_ANGULAR_VEL);
//        } else angularVel = Math.pow(rightX, 4) * SwerveConstants.MAX_ANGULAR_VEL;



//        System.out.println("inputs: " + linearVel + " " + angularVel);


        SwerveSubsystem.getInstance().setSwerveInvKinematics(linearVel, angularVel);

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
