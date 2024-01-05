package com.github.mittyrobotics;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot2 extends TimedRobot {
    private com.reduxrobotics.sensors.canandcoder.Canandcoder[] absEncoders = new Canandcoder[4];

    @Override
    public void teleopInit() {
        for (int i = 0; i < 4; i++) {
            absEncoders[i] = new Canandcoder(SwerveConstants.ABS_ENCODER_IDS[i]);
        }
    }

    @Override
    public void teleopPeriodic() {
        for (int i = 0; i <4 ; i++) {
            System.out.println("QUAD " + (i) + ": " + absEncoders[i].getAbsPosition());
        }

    }
}
