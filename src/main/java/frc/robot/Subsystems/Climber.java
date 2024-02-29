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
<<<<<<< HEAD
import frc.robot.generated.ElevatorConstants;

public class Climber extends SubsystemBase {
  private TalonFX climberMotor = new TalonFX(ElevatorConstants.rightElevatorMotorID);
  //Temporary CANID

=======
import frc.robot.generated.ClimberConstants;
import frc.robot.generated.ElevatorConstants;

public class Climber extends SubsystemBase {
  private TalonFX rightCliberMotor = new TalonFX(ClimberConstants.rightClimberMotorID);
>>>>>>> origin/main

  private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0);
  /** Creates a new Climber. */
  public Climber() {
    configureMotorsControllers();
  }

  @Override
  public void periodic() {
<<<<<<< HEAD
    SmartDashboard.putNumber("Climber Position", climberMotor.getPosition().getValueAsDouble());
    // This method will be called once per scheduler run
  }

  public void moveClimber(double power) {
    climberMotor.set(power);
  }

  public void setClimberPosition(double position) {
    climberMotor.setControl(magicRequest.withPosition(position).withSlot(0));
  }

  private void configureMotorsControllers() {
    climberMotor.getConfigurator().apply(new TalonFXConfiguration());
=======
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Position", rightCliberMotor.getPosition().getValueAsDouble());
  }

  public void moveClimber(double power) {
    rightCliberMotor.set(power);
  }

  public void setElevatorPosition(double position) {
    rightCliberMotor.setControl(magicRequest.withPosition(position).withSlot(0));
  }

  private void configureMotorsControllers() {
    rightCliberMotor.getConfigurator().apply(new TalonFXConfiguration());
>>>>>>> origin/main
    var talonfxConfigs = new TalonFXConfiguration();
    talonfxConfigs.Slot0 = ElevatorConstants.elevatorSlot0Configs;
    talonfxConfigs.CurrentLimits = ElevatorConstants.elevatorCurrentLimits;
    talonfxConfigs.Voltage = ElevatorConstants.elevatorVoltageConfigs;
    talonfxConfigs.Feedback = ElevatorConstants.elevatorFeedbackConfigs;
    talonfxConfigs.MotionMagic = ElevatorConstants.elevatorMotionMagicConfigs;
    talonfxConfigs.SoftwareLimitSwitch = ElevatorConstants.elevatorSoftwareLimitSwitchConfigs;
    talonfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
<<<<<<< HEAD
    climberMotor.getConfigurator().apply(talonfxConfigs);
    climberMotor.setPosition(0.0);
    climberMotor.setInverted(true);
=======
    rightCliberMotor.getConfigurator().apply(talonfxConfigs);
    rightCliberMotor.setPosition(0.0);
>>>>>>> origin/main
  }
}
