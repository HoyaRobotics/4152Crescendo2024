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
import frc.robot.generated.ElevatorConstants;
import monologue.LogLevel;
import monologue.Logged;
import monologue.Annotations.Log;

public class Elevator extends SubsystemBase implements Logged {
    private TalonFX leftElevatorMotor = new TalonFX(ElevatorConstants.leftElevatorMotorID);
    //private TalonFX rightElevator = new TalonFX(ElevatorConstants.rightElevatorMotorID);

    private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0);

    @Log.NT(level = LogLevel.DEFAULT) Pose3d elevatorStage2Pose;
    @Log.NT(level = LogLevel.DEFAULT) Pose3d elevatorStage3Pose;

  /** Creates a new Climber. */
  public Elevator() {
    configureMotorsControllers();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", leftElevatorMotor.getPosition().getValueAsDouble());
    double elevatorRotations = MathUtil.inverseInterpolate(0.0, 2.32, leftElevatorMotor.getPosition().getValueAsDouble());
    double elevator2Positions = MathUtil.interpolate(0.0, 0.3175, elevatorRotations);
    double elevator3Positions = MathUtil.interpolate(0.0, 0.635, elevatorRotations);
    double xTranslation2 = Math.sin(Units.degreesToRadians(22))*elevator2Positions;
    double zTranslation2 = Math.cos(Units.degreesToRadians(22))*elevator2Positions;
    double xTranslation3 = Math.sin(Units.degreesToRadians(22))*elevator3Positions;
    double zTranslation3 = Math.cos(Units.degreesToRadians(22))*elevator3Positions;

    elevatorStage2Pose = new Pose3d(-xTranslation2-0.139299, 0.0, zTranslation2+0.134462, new Rotation3d());
    elevatorStage3Pose = new Pose3d(-xTranslation3-0.148814, 0.0, zTranslation3+0.158012, new Rotation3d());
    
  }

  public void moveElevator(double power) {
    leftElevatorMotor.set(-power);
    //rightElevator.set(power);
  }

  public void stopElevator() {
    leftElevatorMotor.stopMotor();
  }

  public boolean isElevatorAtPosition(double position) {
    double error = Math.abs(position - leftElevatorMotor.getPosition().getValue());
    if(error < ElevatorConstants.positionError) {
      return true;
    }else{
      return false;
    }
  }

  public void setElevatorPosition(double position) {
    leftElevatorMotor.setControl(magicRequest.withPosition(position).withSlot(0));
    //rightElevator.setControl(magicRequest.withPosition(position).withSlot(0));
  }

  public double getElevatorPosition() {
    return leftElevatorMotor.getPosition().getValueAsDouble();
  }

  private void configureMotorsControllers() {
    leftElevatorMotor.getConfigurator().apply(new TalonFXConfiguration());
    //rightElevator.getConfigurator().apply(new TalonFXConfiguration());
    var talonfxConfigs = new TalonFXConfiguration();
    talonfxConfigs.Slot0 = ElevatorConstants.elevatorSlot0Configs;
    talonfxConfigs.CurrentLimits = ElevatorConstants.elevatorCurrentLimits;
    talonfxConfigs.Voltage = ElevatorConstants.elevatorVoltageConfigs;
    talonfxConfigs.Feedback = ElevatorConstants.elevatorFeedbackConfigs;
    talonfxConfigs.MotionMagic = ElevatorConstants.elevatorMotionMagicConfigs;
    talonfxConfigs.SoftwareLimitSwitch = ElevatorConstants.elevatorSoftwareLimitSwitchConfigs;
    talonfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftElevatorMotor.getConfigurator().apply(talonfxConfigs);
    //rightElevator.getConfigurator().apply(talonfxConfigs);
    leftElevatorMotor.setPosition(0.0);
    //rightElevator.setPosition(0.0);
    leftElevatorMotor.setInverted(true);
  }

}
