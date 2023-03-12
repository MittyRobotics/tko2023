package com.github.mittyrobotics.led.commands;

import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.led.LedConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultCommand extends CommandBase {

    int[] purpleHsv, yellowHsv, blueHsv, redHsv;

    StateMachine.PieceState state;

    double time, prevTime, countTime;

    public DefaultCommand() {
        super();
        addRequirements(LedSubsystem.getInstance());
        setName("Default led");
    }

    @Override
    public void initialize() {
        prevTime = Timer.getFPGATimestamp();
        countTime = Timer.getFPGATimestamp();

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

        time = Timer.getFPGATimestamp();

        if (state == StateMachine.PieceState.CONE) {
            runLed(yellowHsv);
        } else if (state == StateMachine.PieceState.CUBE) {
            runLed(purpleHsv);
        } else {
            LedSubsystem.getInstance().turnOff();
        }
    }

    private void runLed(int[] hsv) {
        if (DriverStation.getMatchTime() > 30. || DriverStation.getMatchTime() == -1.)
        {
            LedSubsystem.getInstance().setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                    hsv[0], hsv[1], hsv[2]);

            prevTime = Timer.getFPGATimestamp();
        }
        else if(DriverStation.getMatchTime() < 30. && DriverStation.getMatchTime() > 15.)
        {
            ledSwitch(hsv, blueHsv);
        }
        else if(DriverStation.getMatchTime() < 15.) {
            ledSwitch(hsv, redHsv);
        }
    }

    private void ledSwitch(int[] hsv, int[] altHsv) {
        if (time - prevTime < LedConstants.TIME_BETWEEN_SWITCH)
        {
            LedSubsystem.getInstance().setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                    hsv[0], hsv[1], hsv[2]);
            countTime = Timer.getFPGATimestamp();
        }
        else
        {
            LedSubsystem.getInstance().setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                    altHsv[0], altHsv[1], altHsv[2]);
        }

        if (time - countTime > LedConstants.TIME_BETWEEN_SWITCH) {
            prevTime = Timer.getFPGATimestamp();
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
