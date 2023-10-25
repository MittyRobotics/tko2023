package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.TimedRobot;

public class RobotTest extends TimedRobot {
    WPI_TalonFX falcon;
    @Override
    public void teleopInit() {
        falcon = new WPI_TalonFX(7);
    }

    @Override
    public void teleopPeriodic() {
        falcon.set(0.5);
    }
}
