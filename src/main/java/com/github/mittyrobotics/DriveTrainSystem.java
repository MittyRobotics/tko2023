package com.github.mittyrobotics;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import com.ctre.phoenix.motorcontrol.can.*;


//import static jdk.vm.ci.hotspot.HotSpotJVMCICompilerFactory.CompilationLevel.Simple;

public class DriveTrainSystem extends SubsystemBase {
    private static DriveTrainSystem ourInstance = new DriveTrainSystem();
    public static DriveTrainSystem getInstance() {return ourInstance;}
    private DriveTrainSystem() {
        super();
    }
    WPI_TalonSRX motor_rightf, motor_leftf;
    CANSparkMax motor_rightb, motor_leftb;
    private PIDController pid;
    private RelativeEncoder encoderlb;
    private RelativeEncoder encoderrb;
    private double kP = 0.02;
    private double kI = 0;
    private double kD = 0;
    public static final double TICKS_PER_INCH = 15359.0/24.0;
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1,1.5);
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(10, 0.75);
    public void initHardware()
    {
        //change ids! otherwise only motor_leftb will run, everything else is overridden - naomi
        motor_leftf = new WPI_TalonSRX(5);
        motor_rightb = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor_rightb.getPIDController().setP(0.1);
        motor_rightb.getPIDController().setI(0.1);
        motor_rightb.getPIDController().setD(0.1);
        motor_rightf = new WPI_TalonSRX(22);
        motor_leftb = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor_rightf.configFactoryDefault();
        motor_rightb.restoreFactoryDefaults();
        motor_leftf.configFactoryDefault();
        motor_leftb.restoreFactoryDefaults();
        motor_leftf.setInverted(true);
        motor_leftb.setInverted(true);

        pid = new PIDController(kP, kI, kD);
        encoderlb = motor_leftb.getEncoder();
        encoderrb = motor_rightb.getEncoder();
        encoderlb.setPosition(0);
        encoderrb.setPosition(0);
        //pid.setTolerance(5, 10);

        pid.setSetpoint(50);

    }

    public void trapezoid()
    {
        m_goal = new TrapezoidProfile.State(5,2);
        var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(0.02);
        motor_leftb.getPIDController().setReference(m_goal.position, CANSparkMax.ControlType.kPosition, 0, m_feedforward.calculate(m_goal.velocity));
        motor_rightb.getPIDController().setReference(m_goal.position, CANSparkMax.ControlType.kPosition, 0, m_feedforward.calculate(m_goal.velocity));
        double a = motor_leftb.get();
        double b = motor_rightb.get();
        System.out.println("speed: " + a + " , " + b);
        motor_rightf.set(b);
        motor_leftf.set(a);
    }



    public boolean executePID() {
        /*motor_leftf.set((pid.calculate(encoderlb.getPosition(), pid.getSetpoint()))*0.25);
        motor_leftb.set((pid.calculate(encoderlb.getPosition(), pid.getSetpoint()))*0.25);
        motor_rightf.set((pid.calculate(encoderrb.getPosition(), pid.getSetpoint()))*0.25);
        motor_rightb.set((pid.calculate(encoderrb.getPosition(), pid.getSetpoint()))*0.25);
        //System.out.println(encoderlb.getPosition());
        //System.out.println("left: " + encoderlb.getPosition()+ "   right: " + encoderrb.getPosition());
        Command c = new DriveTrainCommand();
        if(encoderlb.getPosition() > pid.getSetpoint() - 4 && encoderlb.getPosition() < pid.getSetpoint()+2){


            //c.end(true);
        }
        //System.out.println(c.isFinished());*/

        return (encoderlb.getPosition() > pid.getSetpoint() - 4 && encoderlb.getPosition() < pid.getSetpoint()+2);
    }




    public boolean aa()
    {
        motor_rightf.set(0.25);
        motor_rightb.set(0.25);
        motor_leftf.set(0.25);
        motor_leftb.set(0.25);
        //System.out.println("left: " + encoderlb.getPosition()+ "   right: " + encoderrb.getPosition());
        System.out.println(encoderlb.getPosition() > 30);
        if(encoderlb.getPosition() > 30 == true){
            DriveTrainSystem.getInstance().stop();
            Command co = new DriveTrainCommand2();
            co.end(true);

        }
        return (encoderlb.getPosition() > 30);
    }

    public void stop(){
        motor_rightf.set(0);
        motor_rightb.set(0);
        motor_leftf.set(0);
        motor_leftb.set(0);
    }

    public void encoder_value(){
        encoderlb.setPosition(0);
        encoderrb.setPosition(0);
    }


    @Override
    public void periodic()
    {
        //aa();
        //executePID();
        trapezoid();
    }
}

