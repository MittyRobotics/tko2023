package com.github.mittyrobotics;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot2 extends TimedRobot {
    private com.reduxrobotics.sensors.canandcoder.Canandcoder[] absEncoders = new Canandcoder[4];

    @Override
    public void teleopInit() {
        SwerveSubsystem.getInstance().initHardware();
        SwerveSubsystem.getInstance().driveFwd();
    }

    @Override
    public void teleopPeriodic() {

    }
}
