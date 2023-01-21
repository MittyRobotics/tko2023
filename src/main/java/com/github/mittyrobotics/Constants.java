package com.github.mittyrobotics;

public class Constants {

    public static final int[] Arm_Spark_IDs = {1,2,3,4,5};
    public static final double TICKS_PER_DEGREE = 1;
    public static final double TICKS_PER_INCH = 1;
    public static final double TICKS_PER_METERS = TICKS_PER_INCH*0.0254;
    public static final int[] ARM_ENCODER_IDS = {1};
    public static final double MAX_EXTENSION = 44*TICKS_PER_INCH;
    public static final int[] EXTENSION_ENCODER_IDS = {1};
}

