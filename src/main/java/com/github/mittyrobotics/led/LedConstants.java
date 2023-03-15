package com.github.mittyrobotics.led;

public class LedConstants {
    public static final int STRIP_PWM_PORT_FIRST = 8;

    public static final int STRIP_PWM_PORT_SECOND = 0;

    public static final int STRIP_ONE_LENGTH = 120; //CHANGED FROM 120

    public static final int STRIP_TWO_LENGTH = 120;

    //TODO: Tune last two vals
    //Order: Row = R-O-Y-G-B-I-V, Column = R-G-B
    public static final int[][] RGB_VALUES = new int[][]
            {
                    {255, 0, 0}, //Red
                    {230, 50, 0}, //Orange
                    {255, 128, 0}, //Yellow
                    {0, 255, 0}, //Green
                    {0, 0, 255}, //Blue
                    {255, 0, 255}, //Indigo  //Indigo and Violet are currently both the same  //CHANGED FROM 255, 0, 255
                    {255, 0, 255}, //Violet
                    {255, 255, 255} //White
            };
    public static final double TIME_BETWEEN_SWITCH = 0.3;
}
