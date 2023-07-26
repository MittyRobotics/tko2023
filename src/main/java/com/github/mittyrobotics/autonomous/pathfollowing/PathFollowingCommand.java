package com.github.mittyrobotics.autonomous.pathfollowing;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PathFollowingCommand extends CommandBase {
    private Trajectory path;
    public PathFollowingCommand(Trajectory trajectory) {
        path = trajectory;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        System.out.println(path.getStates());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
