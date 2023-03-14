package com.github.mittyrobotics.led;

import com.github.mittyrobotics.led.commands.LEDDefaultCommand;
import com.github.mittyrobotics.util.interfaces.ISubsystem;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase implements ISubsystem {
    //TODO: need to find way to completely stop output (set blank)
    private static LedSubsystem instance;

    private AddressableLED ledStrip;
    private AddressableLEDBuffer buffer, buffer2;
    private boolean outtaking;

    private LedSubsystem() {
        super();
        setName("Leds");
    }

    public static LedSubsystem getInstance() {
        if (instance == null) {
            instance = new LedSubsystem();
        }
        return instance;
    }

    public void setBlinkOuttaking(boolean blink) {
        outtaking = blink;
    }

    public boolean getBlinkOuttaking() {
        return outtaking;
    }

    @Override
    public void updateDashboard() {

    }

    @Override
    public void initHardware() {
        ledStrip = new AddressableLED(LedConstants.STRIP_PWM_PORT_FIRST);
//        ledStripTwo = new AddressableLED(LedConstants.STRIP_PWM_PORT_SECOND);

        buffer = new AddressableLEDBuffer(LedConstants.STRIP_ONE_LENGTH);
//        bufferTwo = new AddressableLEDBuffer(LedConstants.STRIP_TWO_LENGTH);

        ledStrip.setLength(buffer.getLength());
        ledStrip.setData(buffer);
        ledStrip.start();

        buffer2 = new AddressableLEDBuffer(LedConstants.STRIP_ONE_LENGTH);

//        ledStripTwo.setLength(bufferTwo.getLength());
//        ledStripTwo.setData(bufferTwo);
//        ledStripTwo.start();

        setDefaultCommand(new LEDDefaultCommand());
    }

    public void stopOutput() {
        ledStrip.stop();
//        ledStripTwo.stop();
    }

    public void startOutput() {
        ledStrip.start();
//        ledStripTwo.start();
    }

    public void setNothing() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
            ledStrip.setData(buffer);

//            bufferTwo.setRGB(i, 0, 0, 0);
//            ledStripTwo.setData(bufferTwo);
        }
    }

    public void setHsvIndividual(int index, int h, int s, int v) {
        buffer.setHSV(index, h, s, v);
        ledStrip.setData(buffer);
//
//        bufferTwo.setHSV(index, h, s, v);
//        ledStripTwo.setData(bufferTwo);
    }

    public void setHsvRange(int startIndex, int endIndex, int h, int s, int v) {
        //indexes from 0 | i.e. first led = 0, second led = 1, and so on
        for (int i = startIndex; i < endIndex; i++) {
            buffer.setHSV(i, h, s, v);
//            bufferTwo.setHSV(i, h, s, v);
        }
        ledStrip.setData(buffer);
//        ledStripTwo.setData(bufferTwo);
    }

    public void setRgb(int index, int r, int g, int b) {
        buffer.setRGB(index, r, g, b);
//        bufferTwo.setRGB(index, r, g, b);

        ledStrip.setData(buffer);
//        ledStripTwo.setData(bufferTwo);
    }

    public void setRgbRange(int startIndex, int endIndex, int r, int g, int b) {
        for (int i = startIndex; i < endIndex; i++) {
            buffer.setRGB(i, r, g, b);
//            bufferTwo.setRGB(i, r, g, b);
        }
        ledStrip.setData(buffer);
//        ledStripTwo.setData(bufferTwo);
    }

    public void setAlternating(int startIndex, int endIndex, int r, int g, int b) {
        for (int i = startIndex; i < endIndex; i++) {
            if (i % 2 == 0) {
                buffer.setRGB(i, r, g, b);
            }
            ledStrip.setData(buffer);
        }
    }

    public void turnOff() {
        ledStrip.setData(buffer2);
    }

    public void disable() {
        ledStrip.close();

//        ledStripTwo.close();
    }

}
