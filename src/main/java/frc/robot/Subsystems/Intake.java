// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.IntakeConstants;

public class Intake extends SubsystemBase {
  private TalonFX rotationMotor = new TalonFX(22);
  private CANSparkFlex rollerMotor = new CANSparkFlex(23, MotorType.kBrushless);

  final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0);

  /** Creates a new Intake. */
  public Intake() {
    configureRotationMotor();
    configureRollerMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("IntakeRollerCurrent", rollerMotor.getOutputCurrent());
    SmartDashboard.putNumber("IntakeRotationPosition", rotationMotor.getPosition().getValueAsDouble());
  }

  private void configureRotationMotor() {
    rotationMotor.getConfigurator().apply(new TalonFXConfiguration());
    var talonfxConfigs = new TalonFXConfiguration();
    talonfxConfigs.Slot0 = IntakeConstants.rotationSlot0Configs;
    talonfxConfigs.CurrentLimits = IntakeConstants.rotationCurrentLimits;
    talonfxConfigs.Voltage = IntakeConstants.rotationVoltageConfigs;
    talonfxConfigs.Feedback = IntakeConstants.rotationFeedbackConfigs;
    talonfxConfigs.MotionMagic = IntakeConstants.rotationMotionMagicConfigs;
    talonfxConfigs.SoftwareLimitSwitch = IntakeConstants.rotationSoftwareLimitSwitchConfigs;
    talonfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rotationMotor.getConfigurator().apply(talonfxConfigs);
    rotationMotor.setPosition(0.0);
    setIntakePosition(IntakeConstants.stowedPosition);
  }

  private void configureRollerMotor() {
    rollerMotor.restoreFactoryDefaults();
    rollerMotor.enableVoltageCompensation(10);
    rollerMotor.setIdleMode(IdleMode.kBrake);
    rollerMotor.setSmartCurrentLimit(5);
    rollerMotor.set(IntakeConstants.stallSpeed);
  }

  public void setIntakePosition(double position) {
    rotationMotor.setControl(magicRequest.withPosition(position).withSlot(0));
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public boolean isRollerStalled() {
    double rollerCurrent = rollerMotor.getOutputCurrent();
    if(rollerCurrent > 2) {
      return true;
    }else{
      return false;
    }
  }
}
