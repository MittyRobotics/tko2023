package com.github.mittyrobotics;

import com.github.mittyrobotics.led.LedConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class TestLEDV2 extends TimedRobot {
    XboxController controller;

    AddressableLED ledStripOne, ledStripTwo;

    AddressableLEDBuffer bufferOne, bufferTwo;

    int[] yellowHsv, purpleHsv;




    //        ledStripTwo.setLength(bufferTwo.getLength());
//        ledStripTwo.setData(bufferTwo);
//        ledStripTwo.start();
    @Override
    public void robotInit() {

        controller = new XboxController(1);

//        ledStripOne = new AddressableLED(LedConstants.STRIP_PWM_PORT_FIRST);
        ledStripTwo = new AddressableLED(LedConstants.STRIP_PWM_PORT_SECOND);

//        bufferOne = new AddressableLEDBuffer(LedConstants.STRIP_ONE_LENGTH);
        bufferTwo = new AddressableLEDBuffer(LedConstants.STRIP_TWO_LENGTH);

        ledStripTwo.setLength(bufferTwo.getLength());
        ledStripTwo.setData(bufferTwo);
        ledStripTwo.start();

//        ledStripOne.setLength(bufferOne.getLength());
//        ledStripOne.setData(bufferOne);
//        ledStripOne.start();

        yellowHsv = new int[3];
        purpleHsv = new int[3];

        for (int i = 0; i < 3; i++) {
            purpleHsv[i] = LedConstants.RGB_VALUES[5][i];
            yellowHsv[i] = LedConstants.RGB_VALUES[2][i];
        }
    }

    @Override
    public void robotPeriodic() {
//        if(controller.getRightTriggerAxis() > 0.5) {
//            for (int i = 0; i < LedConstants.STRIP_TWO_LENGTH; i++) {
////                bufferOne.setRGB(i, yellowHsv[0], yellowHsv[1], yellowHsv[2]);
//                bufferTwo.setRGB(i, yellowHsv[0], yellowHsv[1], yellowHsv[2]);
////            bufferTwo.setRGB(i, r, g, b);
//            }
////            ledStripOne.setData(bufferOne);
//            ledStripTwo.setData(bufferTwo);
//        } else if (controller.getLeftTriggerAxis() > 0.5){
//            for (int i = 0; i < LedConstants.STRIP_TWO_LENGTH; i++) {
////                bufferOne.setRGB(i, purpleHsv[0], purpleHsv[1], purpleHsv[2]);
//                bufferTwo.setRGB(i, yellowHsv[0], yellowHsv[1], yellowHsv[2]);
////            bufferTwo.setRGB(i, r, g, b);
//            }
////            ledStripOne.setData(bufferOne);
//            ledStripTwo.setData(bufferTwo);
//        } else {
//
//        }
        for (int i = 0; i < LedConstants.STRIP_TWO_LENGTH; i++) {
                bufferTwo.setRGB(i, yellowHsv[0], yellowHsv[1], yellowHsv[2]);
        }
        ledStripTwo.setData(bufferTwo);



    }
    @Override
    public void disabledInit() {
//        ledStripOne.stop();
        ledStripTwo.stop();
    }

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();
    }
}
