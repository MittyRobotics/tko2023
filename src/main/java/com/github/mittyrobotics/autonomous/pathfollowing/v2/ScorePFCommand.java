package com.github.mittyrobotics.autonomous.pathfollowing.v2;

import com.github.mittyrobotics.util.Gyro;
import edu.wpi.first.wpilibj.Timer;

public class ScorePFCommand extends PathFollowingCommand {
    public int order;
    public ScorePFCommand(int order, SwervePath path, double endHeading, double linearThreshold, double angularThreshold, double angStart, double angEnd, double kP_OR_MAXW, double kI, double kD, boolean useInterp) {
        super(path, endHeading, linearThreshold, angularThreshold, angStart, angEnd, kP_OR_MAXW, kI, kD, useInterp);
        this.order = order;
    }

    @Override
    public void initialize() {

        this.path.changeSpline(order == 1 ? AutoPathManager.scoreSpline1 :
                                order == 2 ? AutoPathManager.scoreSpline2 : AutoPathManager.scoreSpline3);
        this.endHeading = AutoPathManager.left ? Math.PI : 0;

        super.initialize();
    }

}
