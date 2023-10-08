package frc.robot;

import frc.robot.subsystems.Gyro;
import com.reduxrobotics.sensors.canandcoder.CANandcoder;
import edu.wpi.first.wpilibj.TimedRobot;

public class AbsEncTest extends TimedRobot {
    CANandcoder[] encoder;

    @Override
    public void robotInit() {
        encoder = new CANandcoder[4];
        for (int i = 0; i < 4; i++) {
//            encoder[i] = new CANandcoder(30 + i);
        }
        Gyro.getInstance().initHardware();
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {
        for (int i = 0; i < 4; i++) {
            encoder[i].setAbsPosition(0);
        }
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        for (int i = 0; i < 4; i++) {
//            System.out.print(encoder[i].getAbsPosition() + " ");
        }
//        System.out.println();
        System.out.println(Gyro.getInstance().getHeadingRadians());
    }
}
