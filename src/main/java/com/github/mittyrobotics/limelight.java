package com.github.mittyrobotics;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class limelight extends SubsystemBase {

    private static limelight ourInstance = new limelight();
    public static limelight getInstance() {return ourInstance;}
    private limelight() {
        super();
    }
    DoubleArraySubscriber areaSub;
    NetworkTable table;
    NetworkTable table1;
    public void initHardware()
    {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        table1 = NetworkTableInstance.getDefault().getTable("LimelightArea");



    }



    @Override
    public void periodic()
    {

        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        //System.out.println

        areaSub = table.getDoubleArrayTopic("tx").subscribe(new double[] {});

        for(double area1: areaSub.get())
        {
            System.out.println(area1 + ", ");
        }



    }
}
