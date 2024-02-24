// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.IntakeConstants;

public class Climber extends SubsystemBase {
    private TalonFX leftElevator = new TalonFX(26);
    private TalonFX rightElevator = new TalonFX(27);

  /** Creates a new Climber. */
  public Climber() {}


private void configureMotorsControllers()
{
    leftElevator.getConfigurator().apply(new TalonFXConfiguration());
    rightElevator.getConfigurator().apply(new TalonFXConfiguration());
    var talonfxConfigs = new TalonFXConfiguration();
    talonfxConfigs.Slot0 = IntakeConstants.rotationSlot0Configs;
    talonfxConfigs.CurrentLimits = IntakeConstants.rotationCurrentLimits;
    talonfxConfigs.Voltage = IntakeConstants.rotationVoltageConfigs;
    talonfxConfigs.Feedback = IntakeConstants.rotationFeedbackConfigs;
    //talonfxConfigs.MotionMagic = IntakeConstants.rotationMotionMagicConfigs;
    //talonfxConfigs.SoftwareLimitSwitch = IntakeConstants.rotationSoftwareLimitSwitchConfigs;
    talonfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftElevator.getConfigurator().apply(talonfxConfigs);
    rightElevator.getConfigurator().apply(talonfxConfigs);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
