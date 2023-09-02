package com.github.mittyrobotics.arm;

import com.github.mittyrobotics.util.math.Angle;

import java.util.HashMap;

import static com.github.mittyrobotics.arm.StateMachine.ArmState;
import static com.github.mittyrobotics.arm.StateMachine.ArmState.*;
import static com.github.mittyrobotics.arm.ArmKinematics.ArmPosition;
import static com.github.mittyrobotics.arm.StateMachine.PieceState;
import static com.github.mittyrobotics.arm.StateMachine.PieceState.*;

public class ArmSetpoints {
    public static final HashMap<ArmState, HashMap<PieceState, ArmPosition>> positions = new HashMap<>();

    public static void initSetpoints() {
        positions.put(STOWED, new HashMap<>());

        positions.put(HIGH, new HashMap<>());

        positions.put(MID, new HashMap<>());

        positions.put(LOW, new HashMap<>());

        positions.put(SCORING, new HashMap<>());

        positions.put(RETRACTED, new HashMap<>());


        positions.get(STOWED).put(CONE, new ArmPosition(new Angle(0, true), 0));
        positions.get(STOWED).put(CUBE, new ArmPosition(new Angle(0, true), 0));

        positions.get(LOW).put(CONE, new ArmPosition(new Angle(0, true), 0));
        positions.get(LOW).put(CUBE, new ArmPosition(new Angle(0, true), 0));

        positions.get(MID).put(CONE, new ArmPosition(new Angle(0, true), 0));
        positions.get(MID).put(CUBE, new ArmPosition(new Angle(0, true), 0));

        positions.get(HIGH).put(CONE, new ArmPosition(new Angle(0, true), 0));
        positions.get(HIGH).put(CUBE, new ArmPosition(new Angle(0, true), 0));

        positions.get(HP).put(CONE, new ArmPosition(new Angle(0, true), 0));
        positions.get(HP).put(CUBE, new ArmPosition(new Angle(0, true), 0));

        positions.get(RETRACTED).put(CONE, new ArmPosition(positions.get(SCORING).get(CONE).getAngle(), 0));
        positions.get(RETRACTED).put(CUBE, new ArmPosition(positions.get(SCORING).get(CUBE).getAngle(), 0));
    }
}
