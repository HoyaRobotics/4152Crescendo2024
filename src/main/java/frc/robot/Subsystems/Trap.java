// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TrapConstants;

public class Trap extends SubsystemBase {
  private final CANSparkFlex trapMotor = new CANSparkFlex(TrapConstants.trapMotorID, MotorType.kBrushless);
  /** Creates a new Trap. */
  public Trap() {
    configureTrapMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void configureTrapMotor() {
    trapMotor.restoreFactoryDefaults();
    trapMotor.setIdleMode(IdleMode.kBrake);
    trapMotor.setSmartCurrentLimit(TrapConstants.trapMotorCurrentLimit);
    trapMotor.stopMotor();
  }

  public void setTrapSpeed(double speed) {
    trapMotor.set(speed);
  }
}
