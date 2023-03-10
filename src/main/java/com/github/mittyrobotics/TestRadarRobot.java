package com.github.mittyrobotics;

import com.github.mittyrobotics.pivot.ArmKinematics;
import edu.wpi.first.wpilibj.TimedRobot;
import org.json.JSONException;

public class TestRadarRobot extends TimedRobot {
    @Override
    public void robotInit() {
        super.robotInit();
    }

    @Override
    public void robotPeriodic() {
        double angle = ArmKinematics.getAngleToGamePiece(false, 0);
        System.out.println(angle);
    }
}
