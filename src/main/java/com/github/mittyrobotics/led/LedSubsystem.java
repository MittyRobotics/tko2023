package com.github.mittyrobotics.led;

import com.github.mittyrobotics.led.commands.DefaultCommand;
import com.github.mittyrobotics.util.interfaces.ISubsystem;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase implements ISubsystem {
    //TODO: need to find way to completely stop output (set blank)
    private static LedSubsystem instance;

    private AddressableLED ledStripOne, ledStripTwo;
    private AddressableLEDBuffer bufferOne, bufferTwo;
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


    @Override
    public void updateDashboard() {

    }

    @Override
    public void initHardware() {
        ledStripOne = new AddressableLED(LedConstants.STRIP_PWM_PORT_FIRST);
        ledStripTwo = new AddressableLED(LedConstants.STRIP_PWM_PORT_SECOND);

        bufferOne = new AddressableLEDBuffer(LedConstants.STRIP_LENGTH);
        bufferTwo = new AddressableLEDBuffer(LedConstants.STRIP_LENGTH);

        ledStripOne.setLength(bufferOne.getLength());
        ledStripOne.setData(bufferOne);
        ledStripTwo.setLength(bufferTwo.getLength());
        ledStripTwo.setData(bufferTwo);
        ledStripOne.start();
        ledStripTwo.start();

        setDefaultCommand(new DefaultCommand());
    }

    public void stopOutput() {
        ledStripOne.stop();
    }

    public void startOutput() {
        ledStripOne.start();
        ledStripTwo.start();
    }

    public void setNothing() {
        for (int i = 0; i < bufferOne.getLength(); i++) {
            bufferOne.setRGB(i, 0, 0, 0);
            ledStripOne.setData(bufferOne);
        }
    }

    public void setHsvIndividual(int index, int h, int s, int v) {
        bufferOne.setHSV(index, h, s, v);
        ledStripOne.setData(bufferOne);
    }

    public void setHsvRange(int startIndex, int endIndex, int h, int s, int v) {
        //indexes from 0 | i.e. first led = 0, second led = 1, and so on
        for (int i = startIndex; i < endIndex; i++) {
            bufferOne.setHSV(i, h, s, v);
        }
        ledStripOne.setData(bufferOne);
    }

    public void setRgb(int index, int r, int g, int b) {
        bufferOne.setRGB(index, r, g, b);
        ledStripOne.setData(bufferOne);
    }

    public void setRgbRange(int startIndex, int endIndex, int r, int g, int b) {
        for (int i = startIndex; i < endIndex; i++) {
            bufferOne.setRGB(i, r, g, b);
            bufferTwo.setRGB(i, r, g, b);
        }
        ledStripOne.setData(bufferOne);
        ledStripTwo.setData(bufferTwo);
    }

    public void disable() {
        ledStripOne.close();
    }

}
