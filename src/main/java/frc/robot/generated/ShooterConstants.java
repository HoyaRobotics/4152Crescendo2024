// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.generated;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

/** Add your docs here. */
public class ShooterConstants {
     public static final double gearRatio = 1.25;
    public static final Slot0Configs shooterSlot0Configs = new Slot0Configs()
        .withKS(0.27) // output to overcome static friction (output)
        .withKV(0.1538) // output per unit of target velocity (output/rps)
        .withKP(0.75) // output per unit of error in position (output/rotation)
        .withKI(0.0) // output per unit of integrated error in position (output/(rotation*s))
        .withKD(0.0); // output per unit of error in velocity (output/rps)
    public static final CurrentLimitsConfigs shooterCurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(40.0)
        .withStatorCurrentLimitEnable(true);
    public static final VoltageConfigs shooterVoltageConfigs = new VoltageConfigs()
        .withPeakForwardVoltage(10.0)
        .withPeakReverseVoltage(10.0);
    public static final FeedbackConfigs shooterFeedbackConfigs = new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(gearRatio);

    /*public static final double kP = 0.0001;
    public static final double kI = 0.0;
    public static final double kD = 0.0002;
    public static final double kFF = 0.000228;*/

    public static final double shootingRPM = 4000 ;//about 1880 rpm exit rotation speed is needed. Thanks to the fly wheels the drop is only about 1000 rpm +-10%
    public static final double spinFactor = 0.05;
    public static final double speedThreshold = 0.98;

    public static final double shootingAngle = 35; //to be used later for projectile profiling.
}

