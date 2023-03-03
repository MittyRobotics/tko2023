package com.github.mittyrobotics;

import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.led.LedConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class TestLED extends TimedRobot {
    XboxController controller;

    AddressableLED ledStripOne, ledStripTwo;

    AddressableLEDBuffer bufferOne, bufferTwo;

    int[] yellowHsv, purpleHsv;

//    protected TestLED() {
//        super();
//    }


    //        ledStripTwo.setLength(bufferTwo.getLength());
//        ledStripTwo.setData(bufferTwo);
//        ledStripTwo.start();
    @Override
    public void robotInit() {


        controller = new XboxController(1);

        ledStripOne = new AddressableLED(LedConstants.STRIP_PWM_PORT_FIRST);
//        ledStripTwo = new AddressableLED(LedConstants.STRIP_PWM_PORT_SECOND);

        bufferOne = new AddressableLEDBuffer(LedConstants.STRIP_ONE_LENGTH);
//        bufferTwo = new AddressableLEDBuffer(LedConstants.STRIP_TWO_LENGTH);
//
//        ledStripTwo.setLength(bufferTwo.getLength());
//        ledStripTwo.setData(bufferTwo);
//        ledStripTwo.start();

        ledStripOne.setLength(bufferOne.getLength());
        ledStripOne.setData(bufferOne);
        ledStripOne.start();

        yellowHsv = new int[3];
        purpleHsv = new int[3];

        for (int i = 0; i < 3; i++) {
            purpleHsv[i] = LedConstants.RGB_VALUES[5][i];
            yellowHsv[i] = LedConstants.RGB_VALUES[2][i];
        }
    }

    @Override
    public void robotPeriodic() {
//        ledStripOne.start();
//        if(controller.getRightTriggerAxis() > 0.5) {
//            for (int i = 0; i < LedConstants.STRIP_ONE_LENGTH; i++) {
//                System.out.println(true);
////                bufferOne.setRGB(i, yellowHsv[0], yellowHsv[1], yellowHsv[2]);
//                bufferOne.setRGB(i, yellowHsv[0], yellowHsv[1], yellowHsv[2]);
//                System.out.println(yellowHsv[0] + " " + yellowHsv[1] + " " + yellowHsv[2]);
////            bufferTwo.setRGB(i, r, g, b);
//            }
////            ledStripOne.setData(bufferOne);
//            ledStripOne.setData(bufferOne);
//        } else if (controller.getLeftTriggerAxis() > 0.5){
//            for (int i = 0; i < LedConstants.STRIP_ONE_LENGTH; i++) {
//                bufferOne.setRGB(i, purpleHsv[0], purpleHsv[1], purpleHsv[2]);
////                bufferTwo.setRGB(i, yellowHsv[0], yellowHsv[1], yellowHsv[2]);
////            bufferTwo.setRGB(i, r, g, b);
//            }
//            ledStripOne.setData(bufferOne);
////            ledStripTwo.setData(bufferTwo);
//        } else {
//
//        }


    }
    @Override
    public void disabledInit() {

//        ledStripOne.stop();
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
    }

    @Override
    public void disabledPeriodic() {
        ledStripOne.stop();
    }

    @Override
    public void teleopPeriodic() {
        ledStripOne.start();
        if(controller.getRightTriggerAxis() > 0.5) {
            for (int i = 0; i < LedConstants.STRIP_ONE_LENGTH; i++) {
                System.out.println(true);
//                bufferOne.setRGB(i, yellowHsv[0], yellowHsv[1], yellowHsv[2]);
                bufferOne.setRGB(i, yellowHsv[0], yellowHsv[1], yellowHsv[2]);
                System.out.println(yellowHsv[0] + " " + yellowHsv[1] + " " + yellowHsv[2]);
//            bufferTwo.setRGB(i, r, g, b);
            }
//            ledStripOne.setData(bufferOne);
            ledStripOne.setData(bufferOne);
        } else if (controller.getLeftTriggerAxis() > 0.5){
            for (int i = 0; i < LedConstants.STRIP_ONE_LENGTH; i++) {
                bufferOne.setRGB(i, purpleHsv[0], purpleHsv[1], purpleHsv[2]);
//                bufferTwo.setRGB(i, yellowHsv[0], yellowHsv[1], yellowHsv[2]);
//            bufferTwo.setRGB(i, r, g, b);
            }
            ledStripOne.setData(bufferOne);
//            ledStripTwo.setData(bufferTwo);
        } else {

        }
    }


}
