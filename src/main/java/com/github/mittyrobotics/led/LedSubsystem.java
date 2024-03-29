package com.github.mittyrobotics.led;

import com.github.mittyrobotics.led.commands.LEDDefaultCommand;
import com.github.mittyrobotics.util.interfaces.ISubsystem;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Set;
import java.util.SortedSet;

public class LedSubsystem extends SubsystemBase implements ISubsystem {
    //TODO: need to find way to completely stop output (set blank)
    private static LedSubsystem instance;

    private AddressableLED ledStrip;
    private AddressableLEDBuffer buffer, buffer2;
    private ArrayList<Color> desiredAltColors = new ArrayList<>();

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

    public void setAltColor(Color c) {
        if (!desiredAltColors.contains(c))
            desiredAltColors.add(c);
    }

    public void disableIntakeAltColor() {
        desiredAltColors.remove(Color.GREEN);
        desiredAltColors.remove(Color.WHITE);
    }

    public void disableDriveAltColor() {
        desiredAltColors.remove(Color.BLUE);
    }

    public int getAltColor() {
        Color res = Color.NONE;
        int ord = Integer.MIN_VALUE;
        for (Color c : desiredAltColors) {
            if (c.order > ord) {
                ord = c.order;
                res = c;
            }
        }
        return res.index;
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

    public enum Color {
        RED(0, -1),
        ORANGE(1, -1),
        YELLOW(2, 0),
        GREEN(3, 3),
        LIGHTBLUE(4, 0),
        BLUE(5, 2),
        PURPLE(6, 0),
        WHITE(7, 1),
        NONE(-1, -2);

        public final Integer index;
        public final Integer order;
        Color(final Integer index, final Integer order) {
            this.index = index;
            this.order = order;
        }
    }

}
