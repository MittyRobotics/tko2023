package com.github.mittyrobotics.led;

public class LedConstants {
    //TODO: Update port
    public static final int STRIP_PWM_PORT = 9;

    public static final int STRIP_LENGTH = 40; //CHANGED FROM 150

    //TODO: Tune last two vals
    //Order: Row = R-O-Y-G-B-I-V, Column = R-G-B
    public static final int[][] RGB_VALUES = new int[][]
            {
             {255, 0, 0}, //Red
             {230, 50, 0}, //Orange
             {255, 128, 0}, //Yellow
             {0, 255, 0}, //Green
             {0, 0, 255}, //Blue
             {255, 0, 250}, //Indigo  //Indigo and Violet are currently both the same  //CHANGED FROM 255, 0, 255
             {255, 0, 255}, //Violet
            };
}
