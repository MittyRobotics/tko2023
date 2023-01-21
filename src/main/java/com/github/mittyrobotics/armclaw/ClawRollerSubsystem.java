package com.github.mittyrobotics.armclaw;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawRollerSubsystem extends SubsystemBase {

    public static CANSparkMax rollerSpark;
    public static Encoder rollerEncoder;

}
