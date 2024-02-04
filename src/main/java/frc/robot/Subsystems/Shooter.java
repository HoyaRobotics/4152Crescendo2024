// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private CANSparkFlex shootLeft = new CANSparkFlex(24, MotorType.kBrushless);
    private CANSparkFlex shootRight = new CANSparkFlex(25, MotorType.kBrushless);

  /** Creates a new Shooter. */
  public Shooter() {
    configShooterMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void configShooterMotors() {
    shootLeft.restoreFactoryDefaults();
    shootRight.restoreFactoryDefaults();
    shootLeft.enableVoltageCompensation(10);
    shootRight.enableVoltageCompensation(10);
    shootLeft.setIdleMode(IdleMode.kCoast);
    shootRight.setIdleMode(IdleMode.kCoast);
    shootRight.setInverted(true);
    shootLeft.setSmartCurrentLimit(20);
    shootRight.setSmartCurrentLimit(20);
  }

  public void setShooterSpeeds(double leftSpeed, double rightSpeed) {}
}
 