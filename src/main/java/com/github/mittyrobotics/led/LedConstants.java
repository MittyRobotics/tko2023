package com.github.mittyrobotics.led;

public class LedConstants {
    //TODO: Update port
    public static final int STRIP_PWM_PORT = 0;

    public static final int STRIP_LENGTH = 150;

    //TODO: Tune values
    //Order: Row = R-O-Y-G-B-I-V, Column = H-S-V
    //Example: First row is array "R" (Red) of length three for "H" (Hue), "S" (Saturation), "V" (value)
    public static final int[][] HSV_VALUES = new int[][]
            {
             {0, 0, 0}, //Red
             {0, 0, 0}, //Orange
             {0, 0, 0}, //Yellow
             {0, 0, 0}, //Green
             {0, 0, 0}, //Blue
             {0, 0, 0}, //Indigo
             {0, 0, 0}, //Violet
            };
}
