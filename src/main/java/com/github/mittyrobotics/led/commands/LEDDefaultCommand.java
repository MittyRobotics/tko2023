package com.github.mittyrobotics.led.commands;

import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.led.LedConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LEDDefaultCommand extends CommandBase {

    int[] purpleHsv, yellowHsv, blueHsv, redHsv, greenHsv;

    StateMachine.PieceState state;

    double time, prevTime, countTime;

    public LEDDefaultCommand() {
        super();
        addRequirements(LedSubsystem.getInstance());
        setName("Default led");
    }

    @Override
    public void initialize() {
        prevTime = Timer.getFPGATimestamp();
        countTime = Timer.getFPGATimestamp();

        LedSubsystem.getInstance().startOutput();

        yellowHsv = LedConstants.RGB_VALUES[5];
        purpleHsv = LedConstants.RGB_VALUES[2];
        greenHsv = LedConstants.RGB_VALUES[3];
        blueHsv = LedConstants.RGB_VALUES[4];
        redHsv = LedConstants.RGB_VALUES[0];
    }

    @Override
    public void execute() {
        state = StateMachine.getInstance().getLastPieceState();

        time = Timer.getFPGATimestamp();

        if (LedSubsystem.getInstance().getBlinkOuttaking()) {
            ledSwitch(greenHsv, state == StateMachine.PieceState.CONE ? yellowHsv : purpleHsv);
        } else if (state == StateMachine.PieceState.CONE) {
            runLed(yellowHsv);
        } else if (state == StateMachine.PieceState.CUBE) {
            runLed(purpleHsv);
        }
    }

    private void runLed(int[] hsv) {
        if (DriverStation.getMatchTime() > 30. || DriverStation.getMatchTime() == -1.) {
            LedSubsystem.getInstance().setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                    hsv[0], hsv[1], hsv[2]);
            prevTime = Timer.getFPGATimestamp();
        } else if (DriverStation.getMatchTime() < 15.) {
            ledSwitch(hsv, redHsv);
        } else {
            ledSwitch(hsv, blueHsv);
        }
    }

    private void ledSwitch(int[] hsv, int[] altHsv) {
        if (time - prevTime < LedConstants.TIME_BETWEEN_SWITCH) {
            LedSubsystem.getInstance().setRgbRange(0, LedConstants.STRIP_ONE_LENGTH,
                    hsv[0], hsv[1], hsv[2]);
            countTime = Timer.getFPGATimestamp();
        } else {
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
