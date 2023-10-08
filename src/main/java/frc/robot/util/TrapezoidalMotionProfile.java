package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;

public class TrapezoidalMotionProfile {
    private double maxAccel, maxDecel, maxVel, startPos, endPos, minOutput, startVel, eps;

    public TrapezoidalMotionProfile(double maxAccel, double maxDecel, double maxVel, double startVel, double startPos, double endPos, double minOutput, double eps) {
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.maxVel = maxVel;
        this.startVel = startVel;
        this.startPos = startPos;
        this.endPos = endPos;
        this.minOutput = minOutput;
        this.eps = eps;
    }

    public void changeSetpoint(double setpoint, double curPos, double curVel) {
        if (setpoint != this.endPos) {
            this.endPos = setpoint;
            this.startPos = curPos;
            this.startVel = curVel;
        }
    }

    public void setSetpoint(double setpoint) {
        this.endPos = setpoint;
    }

    public void setStartVel(double startVel) {
        this.startVel = startVel;
    }

    public void setStartPos(double startPos) {
        this.startPos = startPos;
    }

    public double getSetpoint() {
        return this.endPos;
    }

    public void setDecel(double decel) {
        this.maxDecel = decel;
    }

    public void setMinOutput(double minOutput) {
        this.minOutput = minOutput;
    }

    public TrapezoidalMotionProfile(double maxAccel, double maxVel, double endPos) {
        this(maxAccel, maxAccel, maxVel, 0, 0, endPos, 1, 0.5);
    }

    public double update(double dt, double curPos) {
        double output;
//        System.out.println("Distance from start: " + Math.abs(curPos - startPos) + "  Start Vel: " + getMaxVelFromStart(curPos - startPos)
//        + "\n" + "Distance from end: " + Math.abs(endPos - curPos) + "  End Vel: " + getMaxVelFromEnd(endPos - curPos));

        if (curPos < endPos) {
            output = Math.min(maxVel, getMaxVelFromStart(startPos - curPos) + dt * maxAccel);
            output = Math.min(output, getMaxVelFromEnd(endPos - curPos));

            if (Math.abs(curPos - startPos) < eps * Math.abs(endPos - startPos)) output = Math.max(output, minOutput);
        } else {
            output = Math.max(-maxVel, -getMaxVelFromStart(startPos - curPos) - dt * maxAccel);
            output = Math.max(output, -getMaxVelFromEnd(endPos - curPos));

            if (Math.abs(curPos - startPos) < eps * Math.abs(endPos - startPos)) output = Math.min(output, -minOutput);
        }

        return output;
    }

    public double getMaxVelFromEnd(double pos) {
        return Math.sqrt(Math.abs(2 * maxDecel * pos));
    }

    public double getMaxVelFromStart(double pos) {
        return Math.sqrt(startVel * startVel + Math.abs(2 * maxAccel * pos));
    }
}
