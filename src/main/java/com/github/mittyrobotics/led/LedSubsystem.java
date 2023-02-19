package com.github.mittyrobotics.led;

import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.led.commands.DefaultCommand;
import com.github.mittyrobotics.util.interfaces.ISubsystem;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase implements ISubsystem {
    //TODO: need to find way to completely stop output (set blank)
    private static LedSubsystem instance;

    private AddressableLED ledStrip;
    private AddressableLEDBuffer buffer;
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
        ledStrip = new AddressableLED(LedConstants.STRIP_PWM_PORT);
        buffer = new AddressableLEDBuffer(LedConstants.STRIP_LENGTH);

        ledStrip.setLength(buffer.getLength());
        ledStrip.setData(buffer);
        ledStrip.start();

        setDefaultCommand(new DefaultCommand());
    }

    public void stopOutput() {
        ledStrip.stop();
    }

    public void startOutput() {
        ledStrip.start();
    }

    public void setNothing() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
            ledStrip.setData(buffer);
        }
    }

    public void setHsvIndividual(int index, int h, int s, int v) {
        buffer.setHSV(index, h, s, v);
        ledStrip.setData(buffer);
    }

    public void setHsvRange(int startIndex, int endIndex, int h, int s, int v) {
        //indexes from 0 | i.e. first led = 0, second led = 1, and so on
        for (int i = startIndex; i < endIndex; i++) {
            buffer.setHSV(i, h, s, v);
        }
        ledStrip.setData(buffer);
    }

    public void setRgb(int index, int r, int g, int b) {
        buffer.setRGB(index, r, g, b);
        ledStrip.setData(buffer);
    }

    public void setRgbRange(int startIndex, int endIndex, int r, int g, int b) {
        for (int i = startIndex; i < endIndex; i++) {
            buffer.setRGB(i, r, g, b);
        }
        ledStrip.setData(buffer);
    }

    public void disable() {
        ledStrip.close();
    }

}
