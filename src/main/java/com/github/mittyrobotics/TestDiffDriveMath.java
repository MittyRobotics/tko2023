package com.github.mittyrobotics;

import com.github.mittyrobotics.autonomous.pathfollowing.SwerveAutoPickupCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;

public class TestDiffDriveMath {
    public static void main(String[] args) {
        System.out.println(SwerveAutoPickupCommand.getRadiusFromPoints(new Pose(new Point(0, 0), new Angle(0)), new Point(5, 3)));
    }
}
