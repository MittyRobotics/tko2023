package com.github.mittyrobotics.led.commands;

import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.led.LedConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultCommand extends CommandBase {

    int[] purpleHsv;
    int[] yellowHsv;

    StateMachine.PieceState state;

    public DefaultCommand() {
        super();
        addRequirements(LedSubsystem.getInstance());
        setName("Default led");
    }

    @Override
    public void initialize() {
        LedSubsystem.getInstance().startOutput();

        yellowHsv = new int[3];
        purpleHsv = new int[3];

        for (int i = 0; i < 3; i++) {
            purpleHsv[i] = LedConstants.RGB_VALUES[5][i];
            yellowHsv[i] = LedConstants.RGB_VALUES[2][i];
        }

    }

    @Override
    public void execute() {
        state = StateMachine.getInstance().getLastPieceState();

        if(state == StateMachine.PieceState.CONE) {
            LedSubsystem.getInstance().setRgbRange(0, LedConstants.STRIP_LENGTH,
                    yellowHsv[0], yellowHsv[1], yellowHsv[2]);
        } else if (state == StateMachine.PieceState.CUBE){
            LedSubsystem.getInstance().setRgbRange(0, LedConstants.STRIP_LENGTH,
                    purpleHsv[0], purpleHsv[1], purpleHsv[2]);
        } else if (state == StateMachine.PieceState.NONE) {

        }
    }

    @Override
    public void end(boolean interrupted) {
        LedSubsystem.getInstance().stopOutput();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
