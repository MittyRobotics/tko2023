package com.github.mittyrobotics.led.commands;

import com.github.mittyrobotics.led.LedConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PurpleCommand extends CommandBase {

    int[] purpleHsv;
    boolean first = true;

    public PurpleCommand() {
        super();
        addRequirements(LedSubsystem.getInstance());
        setName("purple");
    }

    @Override
    public void initialize() {
        LedSubsystem.getInstance().startOutput();
        purpleHsv = new int[3];

        for (int i = 0; i < 3; i++) {
            purpleHsv[i] = LedConstants.RGB_VALUES[5][i];
        }
    }

    @Override
    public void execute() {
        LedSubsystem.getInstance().setRgbRange(0, LedConstants.STRIP_LENGTH, 255, 0, 255);
        // LedSubsystem.getInstance().setRgbRange(0, LedConstants.STRIP_LENGTH, purpleHsv[0], purpleHsv[1], purpleHsv[2]);
        /*
        for (int i = 0; i < 256; i++) {
            for (int j = 0; j < 256; j++) {
                    LedSubsystem.getInstance().setRgbRange(0, LedConstants.STRIP_LENGTH, i, j, 200);
                    System.out.println("i " + i + " j " + j);
                    try {
                        Thread.sleep((int) (0.01 * 1000));
                    } catch (InterruptedException ex) {
                        ex.printStackTrace();
                    }
            }
        }
        */

        //first = false;
    }

    @Override
    public void end(boolean interrupted) {
        LedSubsystem.getInstance().stopOutput();
    }

    @Override
    public boolean isFinished() {
        return (OI.getInstance().getOperatorController().getRightTriggerAxis() > 0.5) ||
                (OI.getInstance().getOperatorController().getRightTriggerAxis() < 0.5 && OI.getInstance().getOperatorController().getLeftTriggerAxis() < 0.5);
    }
}
