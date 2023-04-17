package com.github.mittyrobotics;

import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.led.LedConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import edu.wpi.first.wpilibj.*;

public class RobotMatchTime extends TimedRobot {
    AddressableLED ledStrip;
    AddressableLEDBuffer buffer, buffer2;

    XboxController controller;

    int state = 0;

    int[] purpleHsv, yellowHsv, blueHsv, redHsv;

    double time, prevTime, countTime;



    @Override
    public void robotInit() {
        controller = new XboxController(0);
        ledStrip = new AddressableLED(9);
        buffer = new AddressableLEDBuffer(60);
        ledStrip.setLength(buffer.getLength());
        ledStrip.setData(buffer);
        ledStrip.start();
        buffer2 = new AddressableLEDBuffer(60);


        prevTime = Timer.getFPGATimestamp();
        countTime = Timer.getFPGATimestamp();

        yellowHsv = new int[3];
        purpleHsv = new int[3];
        blueHsv = new int[3];
        redHsv = new int[3];


        for (int i = 0; i < 3; i++) {
            purpleHsv[i] = LedConstants.RGB_VALUES[5][i];
            yellowHsv[i] = LedConstants.RGB_VALUES[2][i];
            blueHsv[i] = LedConstants.RGB_VALUES[4][i];
            redHsv[i] = LedConstants.RGB_VALUES[0][i];
        }

    }

    @Override
    public void disabledInit() {
        super.disabledInit();
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
    }

    @Override
    public void robotPeriodic() {

    }

    @Override
    public void disabledPeriodic() {
        ledStrip.setData(buffer2);
    }

    @Override
    public void autonomousPeriodic() {
        super.autonomousPeriodic();
    }

    @Override
    public void teleopPeriodic() {
        if (controller.getRightTriggerAxis() > 0.5) {
            state = 1;
        } else if (controller.getLeftTriggerAxis() > 0.5) {
            state = 2;
        } else if (controller.getAButton()) {
            state = 0;
        }


        time = Timer.getFPGATimestamp();

        if (state == 1) {
            if (DriverStation.getMatchTime() > 30. || DriverStation.getMatchTime() == -1.)
            {
                setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                        yellowHsv[0], yellowHsv[1], yellowHsv[2]);

                prevTime = Timer.getFPGATimestamp();
            }
            else if(DriverStation.getMatchTime() < 30. && DriverStation.getMatchTime() > 15.)
            {
                if (time - prevTime < 0.3)
                {
                    setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                            yellowHsv[0], yellowHsv[1], yellowHsv[2]);
                    countTime = Timer.getFPGATimestamp();
                }
                else
                {
                    setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                            blueHsv[0], blueHsv[1], blueHsv[2]);
                }

                if (time - countTime > 0.3) {
                    prevTime = Timer.getFPGATimestamp();
                }
            }
            else if(DriverStation.getMatchTime() < 15.) {
                if (time - prevTime < 0.3)
                {
                    setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                            yellowHsv[0], yellowHsv[1], yellowHsv[2]);
                    countTime = Timer.getFPGATimestamp();
                }
                else
                {
                    setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                            redHsv[0], redHsv[1], redHsv[2]);
                }

                if (time - countTime > 0.3) {
                    prevTime = Timer.getFPGATimestamp();
                }
            }
        }

        else if (state == 2) {
            if (DriverStation.getMatchTime() > 30. || DriverStation.getMatchTime() == -1.)
            {
                setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                        purpleHsv[0], purpleHsv[1], purpleHsv[2]);

                prevTime = Timer.getFPGATimestamp();
            }
            else if(DriverStation.getMatchTime() < 30. && DriverStation.getMatchTime() > 15.)
            {
                if (time - prevTime < 0.3)
                {
                    setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                            purpleHsv[0], purpleHsv[1], purpleHsv[2]);
                    countTime = Timer.getFPGATimestamp();
                }
                else
                {
                    setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                            blueHsv[0], blueHsv[1], blueHsv[2]);
                }

                if (time - countTime > 0.3) {
                    prevTime = Timer.getFPGATimestamp();
                }
            }
            else if(DriverStation.getMatchTime() < 15.) {
                if (time - prevTime < 0.3)
                {
                    setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                            purpleHsv[0], purpleHsv[1], purpleHsv[2]);
                    countTime = Timer.getFPGATimestamp();
                }
                else
                {
                    setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                            redHsv[0], redHsv[1], redHsv[2]);
                }

                if (time - countTime > 0.3) {
                    prevTime = Timer.getFPGATimestamp();
                }
            }
        }

        else if (state == 0)
        {
            ledStrip.setData(buffer2);
        }



    }

    public void setRgbRange(int startIndex, int endIndex, int r, int g, int b) {
        for (int i = startIndex; i < endIndex; i++) {
            buffer.setRGB(i, r, g, b);
        }
        ledStrip.setData(buffer);
    }
}
