// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.IntakeConstants;
import monologue.LogLevel;
import monologue.Logged;
import monologue.Annotations.Log;


public class Intake extends SubsystemBase implements Logged{
  private final TalonFX rotationMotor = new TalonFX(IntakeConstants.rotatoinMotorID);
  private final CANSparkFlex rollerMotor = new CANSparkFlex(IntakeConstants.rollerMotorID, MotorType.kBrushless);

  private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0);
  //private final MotionMagicExpoVoltage magicRequest = new MotionMagicExpoVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  @Log.NT(level = LogLevel.DEFAULT) Pose3d intakePose;

  //@Log.NT(level = LogLevel.DEFAULT) double IntakeRotationPosition = rotationMotor.getPosition().getValueAsDouble();

  

  /** Creates a new Intake. */
  public Intake() {
    configureRotationMotor();
    configureRollerMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("IntakeRotationPosition", rotationMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("IntakeRollerVelocity", rollerMotor.getEncoder(/*SparkRelativeEncoder.Type.kQuadrature, 7168*/).getVelocity());

    intakePose = new Pose3d(0.32004, 0, 0.280616, new Rotation3d(0, -Units.rotationsToRadians(rotationMotor.getPosition().getValueAsDouble()), 0));
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
    talonfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rotationMotor.getConfigurator().apply(talonfxConfigs);
    rotationMotor.setPosition(IntakeConstants.offset);
    setIntakePosition(IntakeConstants.stowedPosition);
  }

  private void configureRollerMotor() {
    rollerMotor.restoreFactoryDefaults();
    rollerMotor.setIdleMode(IdleMode.kCoast);
    rollerMotor.setSmartCurrentLimit(IntakeConstants.rollerMotorCurrentLimit); //55
    rollerMotor.setInverted(true);
    rollerMotor.set(IntakeConstants.stallSpeed);
  }

  public void setIntakePosition(double position) {
    rotationMotor.setControl(magicRequest.withPosition(position).withSlot(0));
  }

  public void setIntakeRotationSpeed(double speed) {
    rotationMotor.setControl(voltageRequest.withOutput(speed));
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public void setRotationVoltage(double voltage) {
    rotationMotor.setControl(voltageRequest.withOutput(voltage));
  }

  public void stopRotation() {
    rotationMotor.stopMotor();
  }

  public void stopRoller() {
    rollerMotor.stopMotor();
  }

  public boolean isRollerStalled(double stallRPM) {
    double rollerSpeed = rollerMotor.getEncoder(/*SparkRelativeEncoder.Type.kQuadrature, 7168*/).getVelocity();
    if(rollerSpeed < stallRPM) {
      return true;
    }else{
      return false;
    }
  }

  public boolean isIntakeAtPosition(double position) {
    double error = Math.abs(position - rotationMotor.getPosition().getValue());
    if(error < IntakeConstants.positionError) {
      return true;
    }else{
      return false;
    }
  }
}
