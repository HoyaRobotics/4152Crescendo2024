// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.ClimberConstants;
import monologue.LogLevel;
import monologue.Logged;
import monologue.Annotations.Log;

public class Climber extends SubsystemBase implements Logged {
  private TalonFX rightClimberMotor = new TalonFX(ClimberConstants.rightClimberMotorID);

  private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0);

  @Log.NT(level = LogLevel.DEFAULT) Pose3d climberPose;
  /** Creates a new Climber. */
  public Climber() {
    configureMotorsControllers();
    setClimberPosition(ClimberConstants.midCamClearPosition);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Position", rightClimberMotor.getPosition().getValueAsDouble());
    // This method will be called once per scheduler run
    double climberPosition = MathUtil.inverseInterpolate(0.0, 3.69, rightClimberMotor.getPosition().getValueAsDouble());
    double climberRotatoins = MathUtil.interpolate(0.0, 73.6, climberPosition);
    climberPose = new Pose3d(-0.223638, 0, 0.642606, new Rotation3d(0, -Units.degreesToRadians(climberRotatoins), 0));
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

  public void resetEncoder(boolean stop) {
    if(stop) {
      rightClimberMotor.stopMotor();
    }
    rightClimberMotor.setPosition(0.0);
  }

  public boolean isClimberAtPosition(double position) {
    double error = Math.abs(position - rightClimberMotor.getPosition().getValue());
    if(error < ClimberConstants.positionError) {
      return true;
    }else{
      return false;
    }
  }
}
