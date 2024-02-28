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
import frc.robot.generated.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private TalonFX leftElevator = new TalonFX(ElevatorConstants.leftElevatorMotorID);
    //private TalonFX rightElevator = new TalonFX(ElevatorConstants.rightElevatorMotorID);

    private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0);

  /** Creates a new Climber. */
  public Elevator() {
    configureMotorsControllers();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", leftElevator.getPosition().getValueAsDouble());
  }

  public void moveElevtor(double power) {
    leftElevator.set(power);
    //rightElevator.set(power);
  }

  public void setElevatorPosition(double position) {
    leftElevator.setControl(magicRequest.withPosition(position).withSlot(0));
    //rightElevator.setControl(magicRequest.withPosition(position).withSlot(0));
  }

  private void configureMotorsControllers() {
    leftElevator.getConfigurator().apply(new TalonFXConfiguration());
    //rightElevator.getConfigurator().apply(new TalonFXConfiguration());
    var talonfxConfigs = new TalonFXConfiguration();
    talonfxConfigs.Slot0 = ElevatorConstants.elevatorSlot0Configs;
    talonfxConfigs.CurrentLimits = ElevatorConstants.elevatorCurrentLimits;
    talonfxConfigs.Voltage = ElevatorConstants.elevatorVoltageConfigs;
    talonfxConfigs.Feedback = ElevatorConstants.elevatorFeedbackConfigs;
    talonfxConfigs.MotionMagic = ElevatorConstants.elevatorMotionMagicConfigs;
    talonfxConfigs.SoftwareLimitSwitch = ElevatorConstants.elevatorSoftwareLimitSwitchConfigs;
    talonfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftElevator.getConfigurator().apply(talonfxConfigs);
    //rightElevator.getConfigurator().apply(talonfxConfigs);
    leftElevator.setPosition(0.0);
    //rightElevator.setPosition(0.0);
    leftElevator.setInverted(true);
  }

}