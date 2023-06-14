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
}
