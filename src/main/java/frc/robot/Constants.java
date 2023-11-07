// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static class SwerveConstants {
        public static final int[] ANGLE_MOTOR_IDS = {10, 12, 14, 16};
        public static final int[] DRIVE_MOTOR_IDS = {11, 13, 15, 17};
        public static final int[] ABS_ENCODER_IDS = {30, 31, 32, 33};

        //    public static final double[] ANGLE_PID = {0.135, 0, 0.03};
        public static final double[] ANGLE_PID = {0.135, 0.0005, 0.002};
        public static final double[][] DRIVE_PID = {
                {0.175, 0, 0.01, 1024. / 22328.48},
                {0.175, 0, 0.01, 1024. / 22328.48},
                {0.175, 0, 0.01, 1024. / 22328.48},
                {0.175, 0, 0.01, 1024. / 22328.48}
        };
        public static final double MAX_LINEAR_SPEED_INCHES_PER_SECOND = 15 * 12;
        public static final double MAX_ANGULAR_SPEED = 1;
        public static final boolean[] DRIVE_INVERTED = {false, false, false, false};
        public static final boolean[] ANGLE_INVERTED = {false, false, false, false};

        private static final double RADIUS_OF_WHEEL = 2. * 2.54 / 100.;

        private static final double DRIVE_WHEEL_TO_FALCON_GEAR_RATIO = 1. / 6.54;
        private static final double MODULE_TO_FALCON_GEAR_RATIO = 7. / 108.;

        private static final double FALCON_TICKS = 2048.;

        private static final double TICKS_PER_METER = FALCON_TICKS / DRIVE_WHEEL_TO_FALCON_GEAR_RATIO / (2 * Math.PI * RADIUS_OF_WHEEL);

        public static final double TICKS_PER_INCH = TICKS_PER_METER / 39.37;

        public static final double TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO = FALCON_TICKS / MODULE_TO_FALCON_GEAR_RATIO / (2 * Math.PI);

    }

    public static class ShooterConstants {
        public static final int[] MOTOR_ID = new int[]{ 20, 21 };

        public static final double[] PID = new double[]{3e-4, 0, 3e-5, 1. / 2920};

        public static final double MOTOR_TURNS_PER_REV = 2;

        //RPM
        public static final double MID_SHOOTER_RPM = 1150;
        public static final double HIGH_SHOOTER_RPM = 1900;
        public static final double THRESHOLD = 100;
    }

    public static class ConveyorConstants {
        public static final int MOTOR_ID = 22;

        public static final double INCHES_PER_REV = 0;

        public static final int LIMIT_SWITCH_ID = 8;
    }

    public static class IntakeConstants {
        public static final int MOTOR_ID = 23;

        public static final int LIMIT_SWITCH_ID = 9;

        public static final double RADIANS_PER_REV = 2 * Math.PI / (16 * 30. / 12);

        public static final double[] PID = new double[]{1.2, 0, 0, 0};

        //RADIANS
        public static final double ZERO_POSITION = 1.997;
        public static final double UP_POSITION = Math.PI / 2;
        public static final double DOWN_POSITION = -0.19;
        public static final double LOW_SCORE_POSITION = 0.2;
        public static final double THRESHOLD = 0.5 * 3.1415 / 180;

    }
}
