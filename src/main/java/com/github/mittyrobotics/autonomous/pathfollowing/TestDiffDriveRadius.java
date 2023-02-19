package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.pathfollowing.math.*;
import com.github.mittyrobotics.drivetrain.SwerveConstants;

import java.util.Arrays;

public class TestDiffDriveRadius {
    static DiffDriveKinematics kinematics = new DiffDriveKinematics(SwerveConstants.TRACK_WIDTH, SwerveConstants.LENGTH);
    public static void main(String[] args) {
        SwervePath path = new SwervePath(
                new QuinticHermiteSpline(new Point(0, 0), new Angle(0), new Point(1, 1), new Angle(0)),
                new Angle(0), new Angle(0),
                0, 0, 6., 12., 3, 0.0, 0.2, 0.0, 0, 0.00, 0.5
        );

//        SwerveAutoPickupCommand command = new SwerveAutoPickupCommand(0.05, path);
        double radius = SwerveAutoPickupCommand.getRadiusFromPoints(new Pose(new Point(0.5, 0.3), new Angle(Math.PI/2)), path.getByT(0.55).getPosition());
        kinematics.updateFromLinearVelocityAndRadius(5, 0.5);
        System.out.println(Arrays.toString(kinematics.linearVels));
        System.out.println(Arrays.toString(kinematics.angles));
    }
    public static class DiffDriveKinematics {
        protected double linearVelocity, angularVelocity, leftVelocity, rightVelocity, radius, trackWidth, trackLength;
        private final double[] linearVels = new double[4];
        private final double[] angles = new double[4];
        private final int[] radSigns = new int[]{1, -1, -1, 1};
        private final int[] angSigns = new int[]{1, 1, -1, -1};

        public DiffDriveKinematics(double trackWidth, double trackLength) {
            this.trackWidth = trackWidth;
            this.trackLength = trackLength;
        }

        public void updateFromLinearAndAngularVelocity(double linearVelocity, double angularVelocity) {
            this.linearVelocity = linearVelocity;
            this.angularVelocity = angularVelocity;

            if(Math.abs(angularVelocity) < 2e-9) {
                this.radius = Double.POSITIVE_INFINITY;
                for(int i = 0; i < 4; ++i) {
                    angles[i] = 0;
                    linearVels[i] = linearVelocity;
                }
            } else {
                this.radius = linearVelocity / angularVelocity;
                for(int i = 0; i < 4; ++i) {
                    double w = radius + radSigns[i] * trackWidth/2;
                    double l = trackLength / 2;
                    System.out.println("radius " + this.radius);
                    System.out.println("w " + w);
                    System.out.println("atan " + i + " " + Math.atan2(l, w));
                    angles[i] = (radius > 0 ? -1 : 1) * angSigns[i] * Math.abs(Math.atan2(l, Math.abs(w)));
                    linearVels[i] = angularVelocity * Math.sqrt(l * l + w * w);
                }
            }
        }

        public void updateFromLinearVelocityAndRadius(double linearVelocity, double radius) {
            if(Double.isInfinite(radius)) {
                this.angularVelocity = 0;
            } else {
                this.angularVelocity = linearVelocity / radius;
            }
            System.out.println("linear: " + linearVelocity);
            System.out.println("angular: " + angularVelocity);
            updateFromLinearAndAngularVelocity(linearVelocity, this.angularVelocity);
        }
    }
//ur mother
    //gwennie was here U//w//U
    //rawr xd
    //
    public static class InverseKinematics {
        private final Vector swerveModule[] = new Vector[4];
        private Vector tangentialVelocityVector[] = new Vector[4];

        private Vector r;

        private Vector linearVel;
        private double angularVel;

        public InverseKinematics(double width, double length) {
            r = new Vector(width/2, length/2);
        }

        public void setSwerveModule(Vector linearVel, double angularVel) {

            this.linearVel = linearVel;
            this.angularVel = angularVel;

            tangentialVelocityVector[0] = new Vector(-this.angularVel*r.getY(), this.angularVel*r.getX());
            tangentialVelocityVector[1] = new Vector(this.angularVel*r.getY(), this.angularVel*r.getX());
            tangentialVelocityVector[2] = new Vector(this.angularVel*r.getY(), -this.angularVel*r.getX());
            tangentialVelocityVector[3] = new Vector(-this.angularVel*r.getY(), -this.angularVel*r.getX());


            for(int i = 0; i < 4; i++) {
                swerveModule[i] = Vector.add(linearVel, tangentialVelocityVector[i]);
            }
        }
    }

    public static class ForwardKinematics {
        private final Vector r;
        private Point pose = new Point(0, 0);
        private Angle directionOfTravel = new Angle(0);


        public ForwardKinematics(double width, double length) {
            r = new Vector(width/2, length/2);
        }

        public void updateForwardKinematics(Vector[] modules) {

            Point new_ = new Point(0, 0);
            for (int i = 0; i < 4; i++) new_ = Point.add(new_, new Point(modules[i]));
            new_ = Point.multiply(0.25, new_);

            pose = Point.add(pose, new_);
            directionOfTravel = new Angle(new Vector(new_).getAngle().getRadians() - Math.PI/2);

        }

        public Vector getR(int i) {
            return new Vector(r.getX() * (i == 0 || i == 3 ? -1 : 1), r.getY() * (i > 1 ? -1 : 1));
        }

        public Point getPose() {
            return pose;
        }

        public Angle getDirectionOfTravel() {
            return directionOfTravel;
        }
    }
}
