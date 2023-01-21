package com.github.mittyrobotics.armclaw;

public class ClawConstants {

    //TODO: figure literally everything out
    public static final double GRABBER_GEAR_RATIO = 1;
    public static final double GRABBER_ROTATIONS_PER_DEGREE = GRABBER_GEAR_RATIO*(1/360);
    public static final double GRABBER_MAX_ANGLE = 180;
    public static final double GRABBER_MIN_ANGLE = 0;
    public static final int GRABBER_SPARK_ID = 6;
    public static final boolean GRABBER_SPARK_INVERTED = false;

    public static final double CONE_DEGREES = 45;
    public static final double CUBE_DEGREES = 90;
    public static final double OPEN_DEGREES = 120;


    public static final double ROLLER_GEAR_RATIO = 1;
    public static final int ROLLER_SPARK_ID = 7;
    public static final boolean ROLLER_SPARK_INVERTED = false;

    //TODO: setID
    public static final int CLAW_PROX_SENSOR_CHANNEL = 0;

}
