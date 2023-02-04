package com.github.mittyrobotics;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotTestRev extends TimedRobot {
    private static final int deviceID = 1;
    private CANSparkMax m_motor;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    protected RobotTestRev() {
        super();
    }

    @Override
    public void robotInit() {
        allowedErr = 0;
        // initialize motor
        m_motor = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushless);

        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        m_motor.restoreFactoryDefaults();
        m_motor.getEncoder().setPosition(0);

        // initialze PID controller and encoder objects
        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();

        // PID coefficients
        kP = 1;
        kI = 0.0;
        kD = 0.0;
        kFF = 1/6080. ;
        kMaxOutput = 0.3;
        kMinOutput = -0.3;
        maxRPM = 5700;

        // Smart Motion Coefficients
        maxVel = 3000; // rpm
        maxAcc = 1500;
        minVel = 100;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setFF(kFF);
//        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        /**
         * Smart Motion coefficients are set on a SparkMaxPIDController object
         *
         * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
         * the pid controller in Smart Motion mode
         * - setSmartMotionMinOutputVelocity() will put a lower bound in
         * RPM of the pid controller in Smart Motion mode
         * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
         * of the pid controller in Smart Motion mode
         * - setSmartMotionAllowedClosedLoopError() will set the max allowed
         * error for the pid controller in Smart Motion mode
         */
        int smartMotionSlot = 0;
        m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    }

    @Override
    public void autonomousInit() {
        m_motor.set(0.5);
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
    }

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();
    }

    @Override
    public void autonomousPeriodic() {
        System.out.println(m_motor.getEncoder().getVelocity());
    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putNumber("VEL", m_motor.getEncoder().getVelocity());
        SmartDashboard.putNumber("OUTPUT", m_motor.getAppliedOutput());
        m_pidController.setReference(1500, CANSparkMax.ControlType.kSmartMotion);
        System.out.println("POS: " + m_encoder.getPosition());
    }

}

