package com.github.mittyrobotics.intake;

public class IntakeConstants {

    //TODO: figure literally everything out
    public static final double GRABBER_TO_MOTOR_GEAR_RATIO = 1/125.;
    public static final double GRABBER_ROTATIONS_PER_DEGREE = GRABBER_TO_MOTOR_GEAR_RATIO *(1/360.);
    public static final double GRABBER_MAX_ANGLE = 180;
    public static final double GRABBER_MIN_ANGLE = 0;
    public static final int GRABBER_SPARK_ID = 31;
    public static final boolean GRABBER_SPARK_INVERTED = false;

    public static final double CONE_DEGREES = 45;
    public static final double CUBE_DEGREES = 90;
    public static final double OPEN_DEGREES = 120;


    public static final double ROLLER_TO_MOTOR_GEAR_RATIO = 1/9.;
    public static final int ROLLER_SPARK_ID = 12;
    public static final double ROLLER_INTAKE_SPEED = 1;
    public static final boolean ROLLER_SPARK_INVERTED = false;
    public static final int CLAW_PROX_SENSOR_CHANNEL = 4;

}
