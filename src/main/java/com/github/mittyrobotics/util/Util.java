package com.github.mittyrobotics.util;

import java.util.TimerTask;

public class Util {
    public static void triggerFunctionAfterTime(Runnable r, long time) {
        new java.util.Timer().schedule(
                new TimerTask() {
                    @Override
                    public void run() {
                        r.run();
                    }
                },
                time
        );
    }

    public static final double INCHES_PER_METER = 39.37;
    public static final double METERS_PER_INCH = 1 / INCHES_PER_METER;
    public static final double CENTIMETERS_PER_INCH = 2.54;
    public static final double INCHES_PER_CENTIMETER = 1 / CENTIMETERS_PER_INCH;
}
