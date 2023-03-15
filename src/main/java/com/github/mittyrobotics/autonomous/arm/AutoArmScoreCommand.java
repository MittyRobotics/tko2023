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
    private StateMachine.PieceState type;


    public AutoArmScoreCommand(StateMachine.RobotState level, StateMachine.PieceState type, boolean auto) {
        this.level = level;
        this.auto = auto;
        this.type = type;
    }

    @Override
    public void initialize() {
        if(type == StateMachine.PieceState.CONE)
            StateMachine.getInstance().setStateCone();
        if(type == StateMachine.PieceState.CUBE)
            StateMachine.getInstance().setStateCube();
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
        }
        else if (scoring) {
            if(!ran) {
                StateMachine.getInstance().setStateScoring();
                if (StateMachine.getInstance().getCurrentPieceState() == StateMachine.PieceState.CUBE) {
                    StateMachine.getInstance().setOuttaking();

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

                    Util.triggerFunctionAfterTime(() -> {
                        StateMachine.getInstance().setOuttaking();
                        Util.triggerFunctionAfterTime(() -> {
                            ArmKinematics.setArmKinematics(new Angle(curAngle), curRad - 0.4);
                            StateMachine.getInstance().setProfile(StateMachine.RobotState.SCORING, StateMachine.RobotState.STOWED);
                            Util.triggerFunctionAfterTime(() -> {
                                OI.getInstance().zeroAll();
                                StateMachine.getInstance().setProfile(StateMachine.RobotState.SCORING, StateMachine.RobotState.STOWED);
                                StateMachine.getInstance().setIntakeOff();
                                StateMachine.getInstance().setStateNone();
                                switchCam();
                                scoring = false;
                            }, 600);
                        }, 10);
                    }, 200);
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
//        System.out.println((!extendingArm && !scoring) ? "ENDED\n\n\n\n\n\n" : "");
        return !extendingArm && !scoring;
    }
}
