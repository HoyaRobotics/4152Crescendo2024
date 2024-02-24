package frc.robot.generated;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ClimberConstants {
    public static final Slot0Configs rotationSlot0Configs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKS(0) // output to overcome static friction (output)
        .withKV(0) // output per unit of target velocity (output/rps)
        .withKA(0) // output per unit of target acceleration (output/(rps/s))
        .withKP(0) // output per unit of error in position (output/rotation)
        .withKI(0.0) // output per unit of integrated error in position (output/(rotation*s))
        .withKD(0.0); // output per unit of error in velocity (output/rps)
    public static final CurrentLimitsConfigs rotationCurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(40.0)
        .withStatorCurrentLimitEnable(true);
    public static final VoltageConfigs rotationVoltageConfigs = new VoltageConfigs()
        .withPeakForwardVoltage(10.0)
        .withPeakReverseVoltage(10.0);
    public static final FeedbackConfigs rotationFeedbackConfigs = new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(1);
        public static final MotionMagicConfigs rotationMotionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicAcceleration(3.5) // controls acceleration and deceleration rates during the beginning and end of motion
        .withMotionMagicCruiseVelocity(2.0) // peak velocity of the profile; set to 0 to target the system’s max velocity
        .withMotionMagicExpo_kA(0.0) // voltage required to apply a given acceleration, in V/(rps/s)
        .withMotionMagicExpo_kV(0) // voltage required to maintain a given velocity, in V/rps
        .withMotionMagicJerk(0.0); // controls jerk, which is the derivative of acceleration
    public static final SoftwareLimitSwitchConfigs rotationSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(false)
        .withForwardSoftLimitThreshold(0)
        .withReverseSoftLimitEnable(false);
}
