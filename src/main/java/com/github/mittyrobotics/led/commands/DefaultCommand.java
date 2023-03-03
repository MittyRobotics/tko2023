package com.github.mittyrobotics.led.commands;

import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.led.LedConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultCommand extends CommandBase {

    int[] purpleHsv, yellowHsv, blueHsv, redHsv;

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
    public void execute() {
        state = StateMachine.getInstance().getLastPieceState();

        if (state == StateMachine.PieceState.CONE) {
            LedSubsystem.getInstance().setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                    yellowHsv[0], yellowHsv[1], yellowHsv[2]);
        } else if (state == StateMachine.PieceState.CUBE){
            LedSubsystem.getInstance().setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                    purpleHsv[0], purpleHsv[1], purpleHsv[2]);
        } else if (state == StateMachine.PieceState.NONE) {

        }

        if (DriverStation.getMatchTime() < 30. && DriverStation.getMatchTime() > 15.) {
            LedSubsystem.getInstance().setAlternating(0, LedConstants.STRIP_ONE_LENGTH,
                    blueHsv[0], blueHsv[1], blueHsv[2]);
        } else if (DriverStation.getMatchTime() < 15.) {
            LedSubsystem.getInstance().setAlternating(0, LedConstants.STRIP_ONE_LENGTH,
                    redHsv[0], redHsv[1], redHsv[2]);
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
