package com.github.mittyrobotics.util;

public class TrapezoidalMotionProfile {
    private double maxAccel, maxDecel, maxVel, startPos, endPos, minOutput;
    public TrapezoidalMotionProfile(double maxAccel, double maxDecel, double maxVel, double startPos, double endPos, double minOutput) {
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.maxVel = maxVel;
        this.startPos = startPos;
        this.endPos = endPos;
        this.minOutput = minOutput;
    }

    public TrapezoidalMotionProfile(double maxAccel, double maxVel, double endPos) {
        this(maxAccel, maxAccel, maxVel, 0, endPos, 1);
    }

    public double update(double dt, double curVel, double curPos) {
        double output = Math.min(maxVel, curVel + dt * maxAccel);
        output = Math.min(output, getMaxVelFromPos(endPos - curPos));
        return Math.max(minOutput, output) * (curPos < endPos ? 1 : -1);
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
