package com.github.mittyrobotics.util;

public class TrapezoidalMotionProfile {
    private double maxAccel, maxDecel, maxVel, startPos, endPos, minOutput;
    public TrapezoidalMotionProfile(double maxAccel, double maxDecel, double maxVel, double startPos, double endPos, double minOutput) {
        this.maxAccel = maxAccel * 60;
        this.maxDecel = maxDecel * 60;
        this.maxVel = maxVel;
        this.startPos = startPos;
        this.endPos = endPos;
        this.minOutput = minOutput;
    }

    public TrapezoidalMotionProfile(double maxAccel, double maxVel, double endPos) {
        this(maxAccel, maxAccel, maxVel, 0, endPos, 1);
    }

    public double update(double dt, double curPos) {
        double output = 0;
//        System.out.println("DIFF: " + (curPos - endPos));
        if (curPos < endPos) {
            output = Math.min(maxVel, getMaxVelFromPos(curPos - startPos) + dt / 60 * maxAccel);
            output = Math.min(output, getMaxVelFromPos(endPos - curPos));
            if (curPos - startPos < 2) output = Math.max(output, minOutput);
        } else {
            output = Math.max(-maxVel, -getMaxVelFromPos(curPos - startPos) - dt / 60 * maxAccel);
            output = Math.max(output, -getMaxVelFromPos(endPos - curPos));
//            output = Math.min(output, -minOutput);
        }

        return output;
    }

    public double getMaxVelFromPos(double pos) {
        return Math.sqrt(Math.abs(2 * maxDecel * pos));
    }

//    public static void main(String... args) {
//        TrapezoidalMotionProfile p = new TrapezoidalMotionProfile(1, 1, 5, 0, 10);
//
//        double pos = 0;
//        double curVel = 0;
//        while(pos < 5) {
//            double output = p.update(0.02, curVel, pos);
//            curVel = output;
//            pos += curVel * 0.02;
//            System.out.println(pos);
//        }
//    }
}
