package com.github.mittyrobotics.drivetrain;

import java.awt.*;

public class SwerveConstants {

    public static final double W = 17.75;

    public static final double L = 20.75;

    //in the order of quadrants

    public static final int DRIVE_MOTOR_IDS[] = {15, 17, 11, 13}; // 13, 15, 17, 11


    //in the order of quadrants

    public static final int ANGLE_MOTOR_IDS[] = {14, 16, 10, 12};//12, 14, 16, 10

//    public static final int DRIVE_MOTOR_IDS[] = {11, 13, 15, 17}; // 3, 10, 11, 7
//
//    //in the order of quadrants
//
//    public static final int ANGLE_MOTOR_IDS[] = {10, 12, 14, 16};//6, 4, 9, 2


    public static final int MAG_ENCODER_CHANNEL[][] = {{0, 1}, {2, 3}, {4, 5}, {6,7}};
    public static final int[] ABS_ENCODER_IDS = {31, 32, 33, 30};

    public static boolean ROTATION_FALCON_INVERT = true;



    public static final double MAX_LINEAR_VEL = 3 / 3.28084; //10 ft/s
    public static final double MAX_BOOST_LINEAR_VEL = 17 / 3.28084; //18 ft/s


    public static final double MAX_ANGULAR_VEL = 0.5; // was 3.5

    public static final double BUMPER_ANGULAR_VEL = 2;

    public static final double LINEAR_VELOCITY_P = 0.175;

    public static final double LINEAR_VELOCITY_I = 0;

    public static final double LINEAR_VELOCITY_D = 0.01;

    public static final double LINEAR_VELOCITY_FF = 1024./22328.48;

    public static final double ANGLE_LOCK_P = 0.135;

    public static final double ANGLE_LOCK_I = 0.0005;

    public static final double ANGLE_LOCK_D = 0.002;

    public static final double BALANCE_P = 0.1;
    public static final double BALANCE_I = 0;
    public static final double BALANCE_D = 0;

//    public static final double LINEAR_VELOCITY_P = 0;
//    public static final double LINEAR_VELOCITY_I = 0;
//    public static final double LINEAR_VELOCITY_D = 0;


    public static final double ANGULAR_POSITION_P = 0.135;

    public static final double ANGULAR_POSITION_I = 0.0005;

    public static final double ANGULAR_POSITION_D = 0.002;

    public static final double SPEED_FEED_FORWARD = 1023. / 18700.;
//    public static final double SPEED_FEED_FORWARD = 0;



    public static final double STARTING_HEADING = 0;
    public static final double TRACK_WIDTH = 22.75 * 2.54 / 100.;
    public static final double LENGTH = 25.75 * 2.54 / 100.;

    public static final double JOYSTICK_DEADZONE = 0.05;

    public static final double ROTATION_TO_SPEED_RATIO = -13. / 30.;

    public static final double ANGLE_CHANGING_THRESHOLD = 0.00;
    public static final double RADIUS_OF_WHEEL = 2. * 2.54 / 100.;

    public static final double DRIVE_WHEEL_TO_FALCON_GEAR_RATIO = 1. / 6.54;
    public static final double MODULE_TO_FALCON_GEAR_RATIO = 7./108.;

    public static final double FALCON_TICKS = 2048.;
    public static final double MAG_ENCODER_TICKS = 4096.;

    public static final double MAG_ENCODER_SAMPLING_RATE = 4;

    public static final double TICKS_PER_METER = FALCON_TICKS / DRIVE_WHEEL_TO_FALCON_GEAR_RATIO / (2 * Math.PI * RADIUS_OF_WHEEL);

    public static final double TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO = FALCON_TICKS / MODULE_TO_FALCON_GEAR_RATIO / (2 * Math.PI);

    public static final double TICKS_PER_RADIAN_MAG_ENCODER = MAG_ENCODER_TICKS / MAG_ENCODER_SAMPLING_RATE / (2*Math.PI);

    public static final int PIGEON_TWO_ID = 61;
    public static final double[] PIGEON_TWO_MOUNT_CONFIG = {-90, 0, 0}; //(-Yaw) Angle --> +CW, Pitch, Roll
    public static final double TRIGGER_THRESHOLD = 0.05;


    public static final double BOOST_THROTTLE = 1.5;
    public static final double ANGLE_LOCK_THRESHOLD = 0;
}

