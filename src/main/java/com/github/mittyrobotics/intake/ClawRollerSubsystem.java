package com.github.mittyrobotics.intake;

import com.github.mittyrobotics.util.interfaces.ISubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawRollerSubsystem extends SubsystemBase implements ISubsystem {

    private static CANSparkMax rollerSpark;
    private static ClawRollerSubsystem instance;

    private ClawRollerSubsystem(){
        super();
        setName("ClawRoller");
    }
    public static ClawRollerSubsystem getInstance(){
        if(instance==null){
            instance = new ClawRollerSubsystem();
        }
        return instance;
    }

    @Override
    public void updateDashboard() {

    }

    @Override
    public void initHardware() {
        rollerSpark = new CANSparkMax(IntakeConstants.ROLLER_SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        rollerSpark.restoreFactoryDefaults();
        rollerSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rollerSpark.setInverted(IntakeConstants.ROLLER_SPARK_INVERTED);
    }
}
