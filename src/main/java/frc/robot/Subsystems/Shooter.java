// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.ShooterConstants;

public class Shooter extends SubsystemBase {
    //private CANSparkMax shootLeft = new CANSparkMax(24, MotorType.kBrushless);
    //private CANSparkMax shootRight = new CANSparkMax(25, MotorType.kBrushless);

    private TalonFX shootLeft = new TalonFX(24);
    private TalonFX shootRight = new TalonFX(25);

    final VelocityVoltage voltageRequest = new VelocityVoltage(0);

    /*private SparkPIDController leftPID = shootLeft.getPIDController();
    private SparkPIDController rightPID = shootRight.getPIDController();*/
    
  private boolean upToSpeed = false;

  /** Creates a new Shooter. */
  public Shooter() {
    configShooterMotors();
    setShooterSpeeds();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("shooting");
     SmartDashboard.putNumber("shooter speed", getAverageRPM());
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

  public void stopShooter() {
    shootLeft.stopMotor();
    shootRight.stopMotor();
    upToSpeed = false;
  }

  public boolean isShooterAtSpeed() {
    if(getAverageRPM()>=ShooterConstants.speedThreshold*ShooterConstants.shootingRPM) upToSpeed = true;
    return upToSpeed;
  }

  public double getAverageRPM()
  {
    return (shootLeft.getVelocity().getValue()+shootRight.getVelocity().getValue())*30;
  }
}
 