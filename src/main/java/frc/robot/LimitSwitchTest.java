package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.reduxrobotics.sensors.canandcoder.CANandcoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;

public class LimitSwitchTest extends TimedRobot {
    DigitalInput limitSwitch;

    @Override
    public void robotInit() {
        limitSwitch = new DigitalInput(9);
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
        System.out.println(limitSwitch.get());
    }
}
