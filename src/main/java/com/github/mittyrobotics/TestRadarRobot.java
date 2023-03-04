package com.github.mittyrobotics;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.pivot.ArmKinematics;
import edu.wpi.first.wpilibj.TimedRobot;
import org.json.JSONException;

import java.util.Arrays;

public class TestRadarRobot extends TimedRobot {
    @Override
    public void robotInit() {
        super.robotInit();
    }

    @Override
    public void robotPeriodic() {
        try {
            double[] vector = ArmKinematics.getVectorToGamePiece(false, 0);
            System.out.println(39.37 * vector[0] + " " + 39.37 * vector[1] + " " + 39.37 * vector[2]);
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }
    }
}
