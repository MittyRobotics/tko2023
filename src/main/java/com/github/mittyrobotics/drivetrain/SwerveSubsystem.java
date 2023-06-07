package com.github.mittyrobotics.drivetrain;

import com.github.mittyrobotics.util.Angle;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.Vector;

public class SwerveSubsystem {

    static class InverseKinematics {
        private double[] angles;
        private double[] magnitudes;

        private int length, width;

        public void calculateInputs(Vector linearVel, double angularVel) {
            linearVel = new Vector(
                    new Angle(
                            linearVel.getAngle().getRadians() - Gyro.getInstance().getRadians().getRadians(), true),
                    linearVel.getMagnitude()
            );

            for (int i = 0; i < 4; i++) {
                Vector wheelVector = Vector.add(linearVel, Vector.multiply(angularVel, getR(i)));
                angles[i] = wheelVector.getAngle().getRadians();
                magnitudes[i] = wheelVector.getMagnitude();
            }
        }

        private Vector getR(int i) {
            return new Vector(0, 0);
        }
    }
}
