package com.github.mittyrobotics.arm;

import com.github.mittyrobotics.util.math.Angle;

import java.util.HashMap;

import static com.github.mittyrobotics.arm.StateMachine.ArmState;
import static com.github.mittyrobotics.arm.ArmKinematics.ArmPosition;

public class ArmSetpoints {
    public static final HashMap<ArmState, ArmPosition> positions = new HashMap<>();

    public static void initSetpoints() {
        positions.put(ArmState.STOWED, new ArmPosition(new Angle(0, true), 0));
        positions.put(ArmState.HIGH, new ArmPosition(new Angle(0, true), 0));
        positions.put(ArmState.MID, new ArmPosition(new Angle(0, true), 0));
        positions.put(ArmState.LOW, new ArmPosition(new Angle(0, true), 0));
    }
}
