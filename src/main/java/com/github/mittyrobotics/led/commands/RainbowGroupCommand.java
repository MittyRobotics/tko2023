package com.github.mittyrobotics.led.commands;

import com.github.mittyrobotics.led.LedConstants;
import com.github.mittyrobotics.led.LedSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RainbowGroupCommand extends CommandBase {

    double block;
    int cur;
    double prevTime;

    public RainbowGroupCommand() {
        addRequirements(LedSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        //if strip length / 6 is not int, doesnt work
        block = LedConstants.STRIP_LENGTH / 6;
        LedSubsystem.getInstance().startOutput();
        cur = 0;
        prevTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if(Timer.getFPGATimestamp() - prevTime > 0.5) {
            if (Math.ceil(block) == block) {

                if (cur == 6) {
                    cur = 0;
                }

                int prev = cur;

                for (int i = 0; i < 6; i++) {
                    if (prev == 6) {
                        prev = 0;
                    }
                    for (int j = 0; j < block; j++) {
                        LedSubsystem.getInstance().setRgbRange(i * (int) block,
                                (i * (int) (block)) + (int) block,
                                LedConstants.RGB_VALUES[prev][0], LedConstants.RGB_VALUES[prev][1],
                                LedConstants.RGB_VALUES[prev][2]);
                    }
                    prev++;
                }
                prevTime = Timer.getFPGATimestamp();
            } else {
                int currentBlock = 0;

                int block1 = (int) Math.floor(block);
                int block2 = (int) Math.ceil(block);

                if(cur == 6) {
                    cur = 0;
                }

                int prev = cur;

                for (int i = 0; i < 6; i++) {
                    if(prev == 6) {
                        prev = 0;
                    }
                    if(i % 2 == 0) {
                        if(i * block1 + block1 > LedConstants.STRIP_LENGTH) {
                            LedSubsystem.getInstance().setRgbRange(currentBlock,
                                    i * block1 + block1,
                                    LedConstants.RGB_VALUES[prev][0], LedConstants.RGB_VALUES[prev][1],
                                    LedConstants.RGB_VALUES[prev][2]);

                            currentBlock = i * block1 + block1;
                        } else {
                            LedSubsystem.getInstance().setRgbRange(currentBlock,
                                    i * block1 + block1,
                                    LedConstants.RGB_VALUES[prev][0], LedConstants.RGB_VALUES[prev][1],
                                    LedConstants.RGB_VALUES[prev][2]);
                            currentBlock = i * block1 + block1;
                        }
                    }
                    if(i % 2 == 1) {
                        if(i * block2 + block2 > LedConstants.STRIP_LENGTH) {
                            LedSubsystem.getInstance().setRgbRange(currentBlock,
                                    i * block2 + block2,
                                    LedConstants.RGB_VALUES[prev][0], LedConstants.RGB_VALUES[prev][1],
                                    LedConstants.RGB_VALUES[prev][2]);
                        } else {
                            LedSubsystem.getInstance().setRgbRange(currentBlock,
                                    i * block2 + block2,
                                    LedConstants.RGB_VALUES[prev][0], LedConstants.RGB_VALUES[prev][1],
                                    LedConstants.RGB_VALUES[prev][2]);
                            currentBlock = i * block2 + block2;
                        }
                    }
                    prev++;
                }
                cur++;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        LedSubsystem.getInstance().stopOutput();
    }

    @Override
    public boolean isFinished() {
        //TODO: Update
        return false;
    }
}
