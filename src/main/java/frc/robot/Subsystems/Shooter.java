// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.ShooterConstants;

public class Shooter extends SubsystemBase {
    private CANSparkMax shootLeft = new CANSparkMax(24, MotorType.kBrushless);
    private CANSparkMax shootRight = new CANSparkMax(25, MotorType.kBrushless);
    private SparkPIDController leftPID = shootLeft.getPIDController();
    private SparkPIDController rightPID = shootRight.getPIDController();
    
  private boolean upToSpeed = false;

  /** Creates a new Shooter. */
  public Shooter() {
    configShooterMotors();
    configPIDControls();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("shooting");
     SmartDashboard.putNumber("shooter speed", shootLeft.getEncoder().getVelocity());
     //isShooterAtSpeed();
  }

  private void configShooterMotors() {
    shootLeft.restoreFactoryDefaults();
    shootRight.restoreFactoryDefaults();
    shootLeft.enableVoltageCompensation(10);
    shootRight.enableVoltageCompensation(10);
    shootLeft.setIdleMode(IdleMode.kCoast);
    shootLeft.setInverted(true);
    shootRight.setIdleMode(IdleMode.kCoast);
    shootRight.setInverted(false);
    shootLeft.setSmartCurrentLimit(30);
    shootRight.setSmartCurrentLimit(30);

  }

  private void configPIDControls() {
    leftPID.setFF(ShooterConstants.kFF);
    leftPID.setP(ShooterConstants.kP);
    leftPID.setI(ShooterConstants.kI);
    leftPID.setD(ShooterConstants.kD);
    leftPID.setOutputRange(-1,1);

    rightPID.setFF(ShooterConstants.kFF);
    rightPID.setP(ShooterConstants.kP);
    rightPID.setI(ShooterConstants.kI);
    rightPID.setD(ShooterConstants.kD);
    rightPID.setOutputRange(-1,1);
  }

  public void setShooterSpeeds(double leftSpeed, double rightSpeed) {
    leftPID.setReference(ShooterConstants.shootingRPM*(1 - ShooterConstants.spinFactor), CANSparkMax.ControlType.kVelocity);
    rightPID.setReference(ShooterConstants.shootingRPM*(1 + ShooterConstants.spinFactor), CANSparkMax.ControlType.kVelocity);
  }

  public void stopShooter() {
    shootLeft.stopMotor();
    shootRight.stopMotor();
    upToSpeed = false;
  }

  public boolean isShooterAtSpeed() {
    if((shootLeft.getEncoder().getVelocity()+shootRight.getEncoder().getVelocity())/2 >= 0.95*ShooterConstants.shootingRPM) upToSpeed = true;
    return upToSpeed;
  }
}
 