package com.github.mittyrobotics.autonomous.arm;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.pivot.ArmKinematics;
import com.github.mittyrobotics.pivot.PivotSubsystem;
import com.github.mittyrobotics.telescope.TelescopeSubsystem;
import com.github.mittyrobotics.util.OI;
import com.github.mittyrobotics.util.Util;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoArmScorePart2 extends CommandBase {

    private boolean ran, scoring;
    private StateMachine.PieceState type;

    public AutoArmScorePart2(StateMachine.PieceState type) {
        this.type = type;
    }

    @Override
    public void initialize() {
        if(type == StateMachine.PieceState.CONE)
            StateMachine.getInstance().setStateCone();
        if(type == StateMachine.PieceState.CUBE)
            StateMachine.getInstance().setStateCube();
        ran = false;
        scoring = true;
    }

    @Override
    public void execute() {
        if (PivotSubsystem.getInstance().withinThreshold() && TelescopeSubsystem.getInstance().withinThreshold()) {
            if(!ran) {
                StateMachine.getInstance().setStateScoring();
                if (StateMachine.getInstance().getCurrentPieceState() == StateMachine.PieceState.CUBE) {
                    StateMachine.getInstance().setOuttaking();

                    Util.triggerFunctionAfterTime(() -> {
                        OI.getInstance().zeroAll();
                        scoring = false;
                        Util.triggerFunctionAfterTime(() -> {
                            StateMachine.getInstance().setIntakeOff();
                            StateMachine.getInstance().setStateNone();
                            switchCam();
                        }, 200);
                    }, 100);
                } else {
                    double curRad = ArmKinematics.getTelescopeDesired();
                    double curAngle = ArmKinematics.getPivotDesired().getRadians();
                    ArmKinematics.setArmKinematics(new Angle(curAngle + 15 * Math.PI/180), curRad);

                    Util.triggerFunctionAfterTime(() -> {
                        StateMachine.getInstance().setOuttaking();
                        scoring = false;
                        Util.triggerFunctionAfterTime(() -> {
                            ArmKinematics.setArmKinematics(new Angle(curAngle), curRad - 0.4);
                            StateMachine.getInstance().setProfile(StateMachine.RobotState.SCORING, StateMachine.RobotState.STOWED);
                            Util.triggerFunctionAfterTime(() -> {
                                OI.getInstance().zeroAll();
                                StateMachine.getInstance().setProfile(StateMachine.RobotState.SCORING, StateMachine.RobotState.STOWED);
                                StateMachine.getInstance().setIntakeOff();
                                StateMachine.getInstance().setStateNone();
                                switchCam();
                            }, 600);
                        }, 10);
                    }, 200);
                }
                ran = true;
            }
        }
    }

    public void switchCam() {
        int des;
        if(Odometry.getInstance().FIELD_LEFT_SIDE) {
            des = Odometry.getInstance().belowMiddleY() ? 3 : 0; //right vs left front cam
        } else {
            des = Odometry.getInstance().belowMiddleY() ? 0 : 3; //left vs right front cam
        }
        Odometry.getInstance().setCustomCam(des);
    }

    @Override
    public void end(boolean interrupted) {
        StateMachine.getInstance().setStateNone();
    }

    @Override
    public boolean isFinished() {
        return !scoring;
    }
}
