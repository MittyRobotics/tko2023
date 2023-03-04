package com.github.mittyrobotics.drivetrain.commands;

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

    double leftX, leftY, rightX, rightY, leftTrigger, rightTrigger;

    boolean a, b, x, y;

    boolean notMoving;

    double angularVel, heading;

    Vector linearVel;

    boolean disabled;

    PIDController controller = new PIDController(SwerveConstants.ANGLE_LOCK_P,
            SwerveConstants.ANGLE_LOCK_I, SwerveConstants.ANGLE_LOCK_D);

    int anglePreset = 2;
    double currentAngle = 0;

    double[] quad2sp = {0, Math.PI/2, Math.PI, -Math.PI/2};
    double[] quad3sp = {0, Math.PI/2, Math.PI, 3*Math.PI/2};
    double[] quad4sp = {2*Math.PI, Math.PI/2, Math.PI, 3*Math.PI/2};
    double[] quad1sp = {2*Math.PI, 5*Math.PI/2, Math.PI, 3*Math.PI/2};

    double[] correctSP = quad2sp;

    public JoystickThrottleCommand() {
        this.anglePreset = anglePreset;
        addRequirements(SwerveSubsystem.getInstance(), Gyro.getInstance());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        a = OI.getInstance().getPS4Controller().getCrossButton();
        b = OI.getInstance().getPS4Controller().getCircleButton();
        x = OI.getInstance().getPS4Controller().getSquareButton();
        y = OI.getInstance().getPS4Controller().getTriangleButton();

        leftY = OI.getInstance().getPS4Controller().getLeftX();
        leftX = -OI.getInstance().getPS4Controller().getLeftY();
        rightX = -OI.getInstance().getDriveController().getRightX();
        rightTrigger = (OI.getInstance().getPS4Controller().getR2Axis() + 1) / 2.;

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

        currentAngle = SwerveSubsystem.standardize(Gyro.getInstance().getHeadingAngle());



        if(currentAngle >= 3*Math.PI/2 && currentAngle < 2*Math.PI) {
            correctSP = quad1sp;
        } else if(currentAngle >= 0 && currentAngle < Math.PI/2) {
            correctSP = quad2sp;
        } else if(currentAngle >= Math.PI/2 && currentAngle < Math.PI) {
            correctSP = quad3sp;
        } else {
            correctSP = quad4sp;
        }

        if(rightX < 0) {
            angularVel = -(Math.pow(rightX, 4) * SwerveConstants.MAX_ANGULAR_VEL);
        } else angularVel = Math.pow(rightX, 4) * SwerveConstants.MAX_ANGULAR_VEL;
        angularVel = -angularVel;
        SmartDashboard.putNumber("rightx", rightX);
        SmartDashboard.putNumber("angular vel", angularVel);

        if(y) {
            anglePreset = 0;
            controller.setSetpoint(correctSP[anglePreset]);
            angularVel = controller.calculate(currentAngle);
        } else if(b) {
            anglePreset = 1;
            controller.setSetpoint(correctSP[anglePreset]);
            angularVel = controller.calculate(currentAngle);
        } else if(a) {
            anglePreset = 2;
            controller.setSetpoint(correctSP[anglePreset]);
            angularVel = controller.calculate(currentAngle);
        } else if(x) {
            anglePreset = 3;
            controller.setSetpoint(correctSP[anglePreset]);
            angularVel = controller.calculate(currentAngle);
        }

//        SwerveSubsystem.getInstance().setSwerveInvKinematics(Vector.multiply(OI.getInstance().getPS4Controller().getR1Button() ? SwerveConstants.BOOST_THROTTLE : 1, linearVel), angularVel);
        SwerveSubsystem.getInstance().setSwerveInvKinematics(Vector.multiply(1, linearVel), angularVel);

//        SwerveSubsystem.getInstance().setSwerveInvKinematics(new Vector(0.2, 0.5), 0);

        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());

        //wheel angles positive clockwise
        double[] FortyFive = new double[]{-Math.PI/4, Math.PI/4, -Math.PI/4, Math.PI/4};

        if(disabled && DriverStation.getMatchTime() < 15.) {
            SwerveSubsystem.getInstance().setSwerveAngle(FortyFive);
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
