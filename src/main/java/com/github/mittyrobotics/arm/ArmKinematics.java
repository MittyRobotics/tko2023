package com.github.mittyrobotics.arm;

import com.github.mittyrobotics.arm.pivot.PivotSubsystem;
import com.github.mittyrobotics.arm.televator.TelevatorSubsystem;
import com.github.mittyrobotics.util.math.Angle;
import com.github.mittyrobotics.util.math.Vector;

import static java.lang.Math.*;

public class ArmKinematics {
    private static double angle;
    private static double radius;

    public static void updateDesiredArmPositionFromState() {
        StateMachine.ArmState desiredArmState = StateMachine.getArmState();
        ArmPosition desiredArmPosition = ArmSetpoints.positions.get(desiredArmState);
        setArmPositionPolar(desiredArmPosition.angle, desiredArmPosition.radius);
    }

    public static void setArmPositionPolar(Angle angle, double radius) {
        ArmKinematics.angle = PI/2 - angle.getRadians();
        ArmKinematics.radius = radius;
    }

    public static void setArmPositionRectangular(double x, double y) {
        setArmPositionPolar(new Angle(atan2(y, x), true), sqrt(x * x + y * y));
    }

    public static void setArmPositionRectangular(Vector arm) {
        setArmPositionRectangular(arm.getX(), arm.getY());
    }

    public static double getDesiredExtension() {
        return radius;
    }

    public static Angle getDesiredAngle() {
        return new Angle(angle, true);
    }

    public static ArmPosition getDesiredArmPosition() {
        return new ArmPosition(new Angle(angle, true), radius);
    }

    public static ArmPosition getCurrentArmPosition() {
        return new ArmPosition(
                PivotSubsystem.getInstance().getCurrentAngle(),
                TelevatorSubsystem.getInstance().getCurrentExtension()
        );
    }

    static class ArmPosition {
        private Angle angle;
        private double radius;

        public ArmPosition(Angle angle, double radius) {
            this.angle = angle;
            this.radius = radius;
        }

        public Angle getAngle() {
            return angle;
        }

        public double getRadius() {
            return radius;
        }

        static ArmPosition getDifference(ArmPosition p1, ArmPosition p2) {
            return new ArmPosition(
                    new Angle(abs(p1.angle.getRadians() - p2.angle.getRadians()), true),
                    abs(p1.radius - p2.radius)
            );
        }
    }
}
