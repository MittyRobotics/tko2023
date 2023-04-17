package com.github.mittyrobotics.drivetrain.commands;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SnapToAngle extends CommandBase {
    PIDController controller = new PIDController(SwerveConstants.ANGLE_LOCK_P,
            SwerveConstants.ANGLE_LOCK_I, SwerveConstants.ANGLE_LOCK_D);

    int angle = 2;
    double currentAngle = 0;

    double[] quad2sp = {0, Math.PI/2, Math.PI, -Math.PI/2};
    double[] quad3sp = {0, Math.PI/2, Math.PI, 3*Math.PI/2};
    double[] quad4sp = {2*Math.PI, Math.PI/2, Math.PI, 3*Math.PI/2};
    double[] quad1sp = {2*Math.PI, 5*Math.PI/2, Math.PI, 3*Math.PI/2};

    double[] correctSP = quad2sp;

    //angle 0 means +y, angle 1 means -x, angle 2 means -y, angle 3 means +x
    public SnapToAngle(int angle) {
        super();
        this.angle = angle;
        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
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

        controller.setSetpoint(correctSP[angle]);
        double output = controller.calculate(currentAngle);

        SwerveSubsystem.getInstance().setSwerveInvKinematics(new Vector(0,0), -output);
        SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().desiredAngles());
        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().setSwerveInvKinematics(new Vector(0,0), 0);
        SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().desiredAngles());
        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());
    }

    @Override
    public boolean isFinished() {
        double diff = Math.abs(correctSP[angle] - currentAngle);
        return diff < SwerveConstants.ANGLE_LOCK_THRESHOLD;
    }
}
