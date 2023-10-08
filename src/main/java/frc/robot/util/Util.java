package com.github.mittyrobotics.util;

public class Util {
    public static void triggerFunctionAfterTime(Runnable runnable, long time) {
        new java.util.Timer().schedule(
                new java.util.TimerTask() {
                    @Override
                    public void run() {
                        runnable.run();
                    }
                },
                time
        );
    }
}
