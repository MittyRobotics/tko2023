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

import javax.swing.plaf.nimbus.State;

public class AutoArmScoreCommand extends CommandBase {

    private boolean extendingArm;
    private boolean scoring;
    private boolean ran;
    private StateMachine.RobotState level;
    private boolean auto;


    public AutoArmScoreCommand(StateMachine.RobotState level, boolean auto) {
        this.level = level;
        this.auto = auto;
    }

    @Override
    public void initialize() {
        extendingArm = true;
        scoring = false;
        ran = false;
    }

    @Override
    public void execute() {
        if (extendingArm) {
            if(!ran) {
                if (level == StateMachine.RobotState.MID) {
                    OI.getInstance().handleMid();
                } else {
                    OI.getInstance().handleHigh();
                }
                ran = true;
            }

            if (PivotSubsystem.getInstance().withinThreshold() && TelescopeSubsystem.getInstance().withinThreshold()) {
                extendingArm = false;
                ran = false;
                scoring = true;
            }
        } else if (scoring) {
            if(!ran) {
                if (StateMachine.getInstance().getCurrentPieceState() == StateMachine.PieceState.CUBE) {
                    StateMachine.getInstance().setOuttaking();

                    StateMachine.getInstance().setProfile(StateMachine.RobotState.HIGH, StateMachine.RobotState.STOWED);
                    Util.triggerFunctionAfterTime(() -> {
                        OI.getInstance().zeroAll();
                        Util.triggerFunctionAfterTime(() -> {
                            StateMachine.getInstance().setIntakeOff();
                            StateMachine.getInstance().setStateNone();
                            switchCam();
                            scoring = false;
                        }, 200);
                    }, 300);
                } else {
                    double curRad = ArmKinematics.getTelescopeDesired();
                    double curAngle = ArmKinematics.getPivotDesired().getRadians();
                    ArmKinematics.setArmKinematics(new Angle(curAngle + 15 * Math.PI/180), curRad);

                    StateMachine.getInstance().setProfile(StateMachine.RobotState.HIGH, StateMachine.RobotState.STOWED);
                    Util.triggerFunctionAfterTime(() -> {
                        StateMachine.getInstance().setOuttaking();
                        Util.triggerFunctionAfterTime(() -> {
                            OI.getInstance().zeroAll();
                            Util.triggerFunctionAfterTime(() -> {
                                StateMachine.getInstance().setIntakeOff();
                                StateMachine.getInstance().setStateNone();
                                switchCam();
                                scoring = false;
                            }, 500);
                        }, 100);
                    }, 400);
                }
                ran = true;
            }
        }
    }
    public void switchCam() {
        if(auto) {
            int des;
            if(Odometry.getInstance().FIELD_LEFT_SIDE) {
                des = Odometry.getInstance().belowMiddleY() ? 3 : 0; //right vs left front cam
            } else {
                des = Odometry.getInstance().belowMiddleY() ? 0 : 3; //left vs right front cam
            }
            Odometry.getInstance().setCustomCam(des);
        } else {
            Odometry.getInstance().setScoringCam(false);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return !extendingArm && !scoring;
    }
}
