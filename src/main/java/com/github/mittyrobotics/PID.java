package com.github.mittyrobotics;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PID extends CommandBase {

    private PIDController pid;
    private Encoder encoderlb;
    private Encoder encoderrb;
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;

    WPI_TalonSRX motor_rightf, motor_leftf;
    CANSparkMax motor_rightb, motor_leftb;

    public PID() {
        addRequirements(DriveTrainSystem.getInstance());
    }

    @Override
    public void initialize()
    {
        kP = kD = kI = 0;
        pid = new PIDController(kP, kI, kD);
        encoderlb = new Encoder(0, 1);
        encoderrb = new Encoder(0, 1);
        pid.setTolerance(5, 10);
        pid.setSetpoint(1);

        motor_rightf = new WPI_TalonSRX(5);
        motor_rightb = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor_leftf = new WPI_TalonSRX(22);
        motor_leftb = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);

        motor_leftf.setInverted(true);
        motor_leftb.setInverted(true);

        motor_rightf.configFactoryDefault();
        motor_rightb.restoreFactoryDefaults();
        motor_leftf.configFactoryDefault();
        motor_leftb.restoreFactoryDefaults();

    }

    public void execute() {
        motor_leftf.set(pid.calculate(encoderlb.getDistance(), pid.getSetpoint()));
        motor_leftb.set(pid.calculate(encoderlb.getDistance(), pid.getSetpoint()));
        motor_rightf.set(pid.calculate(encoderrb.getDistance(), pid.getSetpoint()));
        motor_rightb.set(pid.calculate(encoderrb.getDistance(), pid.getSetpoint()));
        if(pid.atSetpoint())
        {
            motor_leftf.set(0);
            motor_leftb.set(0);
            motor_rightf.set(0);
            motor_rightb.set(0);
        }
    }
    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished() {
        return false;
    }


}



