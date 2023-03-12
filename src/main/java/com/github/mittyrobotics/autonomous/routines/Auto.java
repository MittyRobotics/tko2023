package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.HashMap;

public class Auto extends SequentialCommandGroup {
    HashMap<StartPose, Pose> approximateStartPoses = new HashMap<>();

    public Auto(StartPose approximateStartPose, SequentialCommandGroup autoRoutine) {
        super();

        approximateStartPoses.put(StartPose.LEFT_TOP, new Pose(new Point(41, 195), new Angle(Math.PI)));
        approximateStartPoses.put(StartPose.LEFT_MIDDLE, new Pose(new Point(41, 129), new Angle(Math.PI)));
        approximateStartPoses.put(StartPose.LEFT_BOTTOM, new Pose(new Point(41, 22), new Angle(Math.PI)));
        approximateStartPoses.put(StartPose.RIGHT_TOP, new Pose(new Point(619, 195), new Angle(Math.PI)));
        approximateStartPoses.put(StartPose.RIGHT_MIDDLE, new Pose(new Point(619, 129), new Angle(Math.PI)));
        approximateStartPoses.put(StartPose.RIGHT_BOTTOM, new Pose(new Point(619, 22), new Angle(Math.PI)));

        addCommands(
                new InitAutoCommand(approximateStartPoses.get(approximateStartPose)),
                autoRoutine
        );
    }

    enum StartPose {
        LEFT_TOP,
        LEFT_BOTTOM,
        LEFT_MIDDLE,
        RIGHT_TOP,
        RIGHT_BOTTOM,
        RIGHT_MIDDLE
    }


}
