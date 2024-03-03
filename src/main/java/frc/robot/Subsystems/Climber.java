// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.generated.ClimberConstants;

public class Climber extends SubsystemBase {
  private TalonFX rightClimberMotor = new TalonFX(ClimberConstants.rightClimberMotorID);

  private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0);
  /** Creates a new Climber. */
  public Climber() {
    configureMotorsControllers();
    //setClimberPosition(ClimberConstants.midCamClearPosition);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Position", rightClimberMotor.getPosition().getValueAsDouble());
    // This method will be called once per scheduler run
  }

  public void moveClimber(double power) {
    rightClimberMotor.set(power);
  }

  public void stopClimber() {
    rightClimberMotor.stopMotor();
  }

  public void setClimberPosition(double position) {
    rightClimberMotor.setControl(magicRequest.withPosition(position).withSlot(0));
  }

  public double getClimberPosition() {
    return rightClimberMotor.getPosition().getValueAsDouble();
  }

  private void configureMotorsControllers() {
    rightClimberMotor.getConfigurator().apply(new TalonFXConfiguration());
    var talonfxConfigs = new TalonFXConfiguration();
    talonfxConfigs.Slot0 = ClimberConstants.climberSlot0Configs;
    talonfxConfigs.CurrentLimits = ClimberConstants.climberCurrentLimits;
    talonfxConfigs.Voltage = ClimberConstants.climberVoltageConfigs;
    talonfxConfigs.Feedback = ClimberConstants.climberFeedbackConfigs;
    talonfxConfigs.MotionMagic = ClimberConstants.climberMotionMagicConfigs;
    talonfxConfigs.SoftwareLimitSwitch = ClimberConstants.climberSoftwareLimitSwitchConfigs;
    talonfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightClimberMotor.getConfigurator().apply(talonfxConfigs);
    rightClimberMotor.setPosition(0.0);
  }

  public void resetEncoder() {
    rightClimberMotor.setPosition(0.0);
  }
}
