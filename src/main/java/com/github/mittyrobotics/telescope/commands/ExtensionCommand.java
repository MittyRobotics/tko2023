package com.github.mittyrobotics.telescope.commands;

import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ExtensionCommand extends InstantCommand {
    public ExtensionCommand(double distance) {
        super(() -> TelescopeSubsystem.getInstance().setPositionMeters(distance));
    }
}
