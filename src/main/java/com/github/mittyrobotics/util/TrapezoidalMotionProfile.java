package com.github.mittyrobotics.util;

public class TrapezoidalMotionProfile {
    private double maxAccel, maxDecel, maxVel, startPos, endPos, minOutput, eps;
    public TrapezoidalMotionProfile(double maxAccel, double maxDecel, double maxVel, double startPos, double endPos, double minOutput, double eps) {
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.maxVel = maxVel;
        this.startPos = startPos;
        this.endPos = endPos;
        this.minOutput = minOutput;
        this.eps = eps;
    }

    public void setSetpoint(double setpoint) {
        this.endPos = setpoint;
    }

    public void setDecel(double decel) {
        this.maxDecel = decel;
    }

    public TrapezoidalMotionProfile(double maxAccel, double maxVel, double endPos) {
        this(maxAccel, maxAccel, maxVel, 0, endPos, 1, 0.5);
    }

    public double update(double dt, double curPos, double curVel) {
        double output = 0;
        System.out.println(Math.abs(curPos - startPos) + "  " + 0.2 * Math.abs(endPos - startPos) + "  END: " + getMaxVelFromPos(endPos - curPos));

//        System.out.println("DIFF: " + (DIFFcurPos - endPos));
        if (curPos < endPos) {
//            output = maxVel;
            output = Math.min(maxVel, getMaxVelFromPos(startPos - curPos) + dt * maxAccel);
            output = Math.min(output, getMaxVelFromPos(endPos - curPos));
            if (Math.abs(curPos - endPos) > eps) output = Math.max(output, minOutput);
        } else {
//            output = -maxVel;
            output = Math.max(-maxVel, -getMaxVelFromPos(startPos - curPos) - dt * maxAccel);
            output = Math.max(output, -getMaxVelFromPos(endPos - curPos));
//            output = Math.min(output, -minOutput);
            if (Math.abs(curPos - endPos) > eps) output = Math.min(output, -minOutput);
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
