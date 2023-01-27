package com.github.mittyrobotics.telescope.commands;

import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class ExtensionCommand extends InstantCommand {
    public ExtensionCommand(double distance) {
        super(() -> TelescopeSubsystem.getInstance().setPositionMeters(distance));
    }
}
