package com.github.mittyrobotics.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.github.mittyrobotics.util.math.Angle;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.math.Vector;

import static com.github.mittyrobotics.drivetrain.SwerveConstants.*;

public class SwerveSubsystem {
    private static SwerveSubsystem instance;

    public static SwerveSubsystem getInstance() {
        if (instance == null) instance = new SwerveSubsystem();
        return instance;
    }

    public final InverseKinematics inverseKinematics = new InverseKinematics();

    private WPI_TalonFX[] driveMotors = new WPI_TalonFX[4];
    private WPI_TalonFX[] angleMotors = new WPI_TalonFX[4];

    public void initHardware() {
        for (int i = 0; i < 4; i++) {
            driveMotors[i] = new WPI_TalonFX(DRIVE_MOTOR_IDS[i]);
            angleMotors[i] = new WPI_TalonFX(ANGLE_MOTOR_IDS[i]);

            driveMotors[i].configFactoryDefault();
            driveMotors[i].setSelectedSensorPosition(0);
            driveMotors[i].config_kP(0, DRIVE_PID[0]);
            driveMotors[i].config_kP(0, DRIVE_PID[1]);
            driveMotors[i].config_kP(0, DRIVE_PID[2]);
            driveMotors[i].setInverted(DRIVE_INVERTED[i]);

            angleMotors[i].configFactoryDefault();
            angleMotors[i].setSelectedSensorPosition(0);
            angleMotors[i].config_kP(0, ANGLE_PID[0]);
            angleMotors[i].config_kP(0, ANGLE_PID[1]);
            angleMotors[i].config_kP(0, ANGLE_PID[2]);
            angleMotors[i].config_kF(0, ANGLE_PID[3]);
            angleMotors[i].setInverted(ANGLE_INVERTED[i]);
        }
    }

    public void calculateInputs(Vector linearVel, double angularVel) {
        inverseKinematics.calculateInputs(linearVel, angularVel);
    }

    public void applyCalculatedInputs() {
        setDriveMotors(inverseKinematics.getMagnitudes());
        setAngleMotors(inverseKinematics.getAngles());
    }

    public void setDriveMotors(double[] values) {
        for (int i = 0; i < 4; i++) {
            driveMotors[i].set(ControlMode.Velocity, values[i]);
        }
    }

    public void setAngleMotors(double[] values) {
        for (int i = 0; i < 4; i++) {
            angleMotors[i].set(ControlMode.Position, values[i]);
        }
    }

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

        public double[] getAngles() {
            return angles;
        }

        public double[] getMagnitudes() {
            return magnitudes;
        }
    }

    static class ForwardKinematics{
        // TODO: 6/13/2023 WRITE!!!!!
    }
}
