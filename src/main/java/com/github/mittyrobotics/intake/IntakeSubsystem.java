package com.github.mittyrobotics.intake;

import com.github.mittyrobotics.intake.commands.AutoIntakeCommand;
import com.github.mittyrobotics.intake.commands.IntakeCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;
    private boolean defaultState = true;

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }

        return instance;
    }

    CANSparkMax spark;
    DigitalInput proximitySensor;

    public void initHardware() {
        spark = new CANSparkMax(IntakeConstants.INTAKE_SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        spark.restoreFactoryDefaults();
        spark.getPIDController().setFeedbackDevice(spark.getEncoder());

        spark.getPIDController().setFF(IntakeConstants.FF);
        spark.getPIDController().setP(IntakeConstants.P);
        spark.getPIDController().setI(IntakeConstants.I);
        spark.getPIDController().setD(IntakeConstants.D);

        proximitySensor = new DigitalInput(IntakeConstants.PROX_SENSOR_ID);

        setDefaultCommand(new AutoIntakeCommand());
    }

    public void defaultState() {
        if (defaultState) spark.set(-0.1);
    }

    public void setDefaultState(boolean dfs) {
        defaultState = dfs;
    }

    public void setMotor(double val) {
        spark.set(val);
    }

    public void configPID(double p, double i, double d) {
        spark.getPIDController().setP(p);
        spark.getPIDController().setI(i);
        spark.getPIDController().setD(d);
    }

    public boolean proxSensorTrigger() {
        return !proximitySensor.get();
    }
}
