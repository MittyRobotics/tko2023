package com.github.mittyrobotics.led.commands;

import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.led.LedConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LEDDefaultCommand extends CommandBase {

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
    }

    @Override
    public void execute() {
        state = StateMachine.getInstance().getLastPieceState();

        time = Timer.getFPGATimestamp();

        if (state == StateMachine.PieceState.NONE) {
            if (LedSubsystem.getInstance().getAltColor() != -1) {
                ledSwitch(LedConstants.RGB_VALUES[LedSubsystem.getInstance().getAltColor()],
                        LedConstants.RGB_VALUES[LedSubsystem.getInstance().getAltColor()]);
            }
        }

        int[] mainHsv = state == StateMachine.PieceState.CONE ? LedConstants.RGB_VALUES[LedSubsystem.Color.YELLOW.index]
                : LedConstants.RGB_VALUES[LedSubsystem.Color.PURPLE.index];

        if (LedSubsystem.getInstance().getAltColor() != -1) {
            ledSwitch(mainHsv, LedConstants.RGB_VALUES[LedSubsystem.getInstance().getAltColor()]);
        } else if (DriverStation.getMatchTime() > 30. || DriverStation.getMatchTime() == -1.) {
            ledSwitch(mainHsv, mainHsv);
        } else if (DriverStation.getMatchTime() < 15.) {
            ledSwitch(mainHsv, LedConstants.RGB_VALUES[LedSubsystem.Color.RED.index]);
        } else {
            ledSwitch(mainHsv, LedConstants.RGB_VALUES[LedSubsystem.Color.ORANGE.index]);
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
