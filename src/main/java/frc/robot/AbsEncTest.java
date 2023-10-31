package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.reduxrobotics.sensors.canandcoder.CANandcoder;
import edu.wpi.first.wpilibj.TimedRobot;

public class AbsEncTest extends TimedRobot {
    CANandcoder[] encoder;
    WPI_TalonFX[] falcons;

    @Override
    public void robotInit() {
        encoder = new CANandcoder[4];
        falcons = new WPI_TalonFX[4];
        for (int i = 0; i < 4; i++) {
            falcons[i] = new WPI_TalonFX(10 + 2 * i);
            encoder[i] = new CANandcoder(30 + i);
        }
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
        for (int i = 0; i < 4; i++) {
            falcons[i].setNeutralMode(NeutralMode.Coast);
        }
    }

    @Override
    public void teleopPeriodic() {
        for (int i = 0; i < 4; i++) {
            System.out.print(encoder[i].getAbsPosition() + " ");
        }
        System.out.println();
    }
}
