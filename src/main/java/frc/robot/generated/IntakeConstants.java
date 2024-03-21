// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.generated;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

/** Add your docs here. */
public class IntakeConstants {
    public static final double rotationGearRatio = 45.7143;
    public static final Slot0Configs rotationSlot0Configs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKS(0.3) // output to overcome static friction (output)
        .withKV(5.17) // output per unit of target velocity (output/rps)
        .withKA(0.353) // output per unit of target acceleration (output/(rps/s))
        .withKP(25) // output per unit of error in position (output/rotation)
        .withKI(0.0) // output per unit of integrated error in position (output/(rotation*s))
        .withKD(0.0); // output per unit of error in velocity (output/rps)
    public static final CurrentLimitsConfigs rotationCurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(20.0)
        .withStatorCurrentLimitEnable(true);
    public static final VoltageConfigs rotationVoltageConfigs = new VoltageConfigs()
        .withPeakForwardVoltage(10.0)
        .withPeakReverseVoltage(10.0);
    public static final FeedbackConfigs rotationFeedbackConfigs = new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(rotationGearRatio);
    public static final MotionMagicConfigs rotationMotionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicAcceleration(3.5) // controls acceleration and deceleration rates during the beginning and end of motion
        .withMotionMagicCruiseVelocity(2.0) // peak velocity of the profile; set to 0 to target the system’s max velocity
        .withMotionMagicExpo_kA(0.0) // voltage required to apply a given acceleration, in V/(rps/s)
        .withMotionMagicExpo_kV(0) // voltage required to maintain a given velocity, in V/rps
        .withMotionMagicJerk(0.0); // controls jerk, which is the derivative of acceleration
    public static final SoftwareLimitSwitchConfigs rotationSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(0.05)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(-0.58);
    public static final double stowedPosition = 0.01;
    public static final double shootPosition = 0.01;
    public static final double floorPosition = -0.55;
    public static final double outakePosition = -0.078;
    public static final double sourcePosition = 0.0;
    public static final double ampPosition = -0.21;//-0.23

    public static final double positionError = 0.01;
    //Can assign later if need be
    public static final double floorSpeed = 0.75; //0.6
    public static final double stallSpeed = 0;  //0.04
    public static final double shootSpeed = -1.0;
    public static final double ampSpeed = -0.35; //-0.3
    public static final double trapSpeed = -0.5;
    public static final double shootTrapSpeed = -0.70;
    public static final double outakeSpeed = -1.0;

    public static final double stallTriggerTime = 0.05;
    public static final double stallRPM = 0.05;
    public static final double fastStallRPM = 3700;
    public static final int rotatoinMotorID = 22;
    public static final int rollerMotorID = 23;
    public static final int rollerMotorCurrentLimit = 60;
}
