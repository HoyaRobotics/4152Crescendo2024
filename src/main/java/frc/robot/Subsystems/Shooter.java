// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.ShooterConstants;

public class Shooter extends SubsystemBase {
    private TalonFX shootLeft = new TalonFX(ShooterConstants.leftShooterMotorID);
    private TalonFX shootRight = new TalonFX(ShooterConstants.rightShooterMotorID);

    final VelocityVoltage voltageRequest = new VelocityVoltage(0);
    final PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);

    //@Log.NT(level = LogLevel.DEFAULT) double leftShooterSpeed = shootLeft.getVelocity().getValueAsDouble();
    //@Log.NT(level = LogLevel.DEFAULT) double rightShooterSpeed = shootRight.getVelocity().getValueAsDouble();
    
  private boolean upToSpeed = false;

  /** Creates a new Shooter. */
  public Shooter() {
    configShooterMotors();
    //setShooterSpeeds();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter speed", getAverageRPM());

    SmartDashboard.putNumber("current Draw Right Shooter", pdp.getCurrent(3) );
    SmartDashboard.putNumber("Power Draw Right Shooter", pdp.getVoltage()*pdp.getCurrent(3) );

    SmartDashboard.putNumber("Current Draw Left Shooter", pdp.getCurrent(16));
    SmartDashboard.putNumber("Power Draw Left Shooter", pdp.getCurrent(16)*pdp.getVoltage());

  }
  
  private void configShooterMotors() {
    
    shootLeft.getConfigurator().apply(new TalonFXConfiguration());
    shootRight.getConfigurator().apply(new TalonFXConfiguration());
    var talonfxConfigs = new TalonFXConfiguration();
    talonfxConfigs.Slot0 = ShooterConstants.shooterSlot0Configs;
    talonfxConfigs.CurrentLimits = ShooterConstants.shooterCurrentLimits;
    talonfxConfigs.Voltage = ShooterConstants.shooterVoltageConfigs;
    talonfxConfigs.Feedback = ShooterConstants.shooterFeedbackConfigs;
    talonfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shootLeft.getConfigurator().apply(talonfxConfigs);
    shootRight.getConfigurator().apply(talonfxConfigs);
    shootRight.setInverted(true);
  }

  public void setShooterSpeeds() {
    shootLeft.setControl(voltageRequest.withVelocity(ShooterConstants.shootingRPM*(1-ShooterConstants.spinFactor)/60));
    shootRight.setControl(voltageRequest.withVelocity(ShooterConstants.shootingRPM*(1+ShooterConstants.spinFactor)/60));
  }

  public void setTrapShooterSpeeds(){
    shootLeft.setControl(voltageRequest.withVelocity(ShooterConstants.trapHandoffRPM/60));
    shootRight.setControl(voltageRequest.withVelocity(ShooterConstants.trapHandoffRPM/60));
  }

  public void setTrapShootRPM(){
    shootLeft.setControl(voltageRequest.withVelocity(ShooterConstants.trapShootRPM/60));
    shootRight.setControl(voltageRequest.withVelocity(ShooterConstants.trapShootRPM/60));
  }

  public void stopShooter() {
   shootLeft.stopMotor();
    shootRight.stopMotor();
    upToSpeed = false;
  }

  public void idleMotor()
  {
    stopShooter();
    shootLeft.setControl(voltageRequest.withVelocity(ShooterConstants.idleSpeed/60));
    shootRight.setControl(voltageRequest.withVelocity(ShooterConstants.idleSpeed/60));
    upToSpeed = false;
  }

  public boolean isShooterAtSpeed(double setSpeed) {
    if(getAverageRPM()>=ShooterConstants.speedThreshold*setSpeed) upToSpeed = true;
    return upToSpeed;
  }

  public double getAverageRPM()
  {
    return (shootLeft.getVelocity().getValue()+shootRight.getVelocity().getValue())*30;
  }
}
 