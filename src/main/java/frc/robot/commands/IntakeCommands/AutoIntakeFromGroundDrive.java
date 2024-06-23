
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.Intake;
import frc.robot.generated.IntakeConstants;

public class AutoIntakeFromGroundDrive extends Command {
  private final Intake intake;
  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier triggerTranslation;
  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;
  private final DoubleSupplier rotation;

  private PIDController yawPIDController = new PIDController(0.1, 0.0, 0.0);

  private SwerveRequest.FieldCentric fieldRequest;
  private SwerveRequest.RobotCentric robotRequest = new SwerveRequest.RobotCentric();

  /** Creates a new Intake. */
  public AutoIntakeFromGroundDrive(Intake intake, CommandSwerveDrivetrain drivetrain, DoubleSupplier triggerTranslation, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation, SwerveRequest.FieldCentric fieldRequest) {
    this.intake = intake;
    this.drivetrain = drivetrain;
    this.triggerTranslation = triggerTranslation;
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotation = rotation;
    this.fieldRequest = fieldRequest;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakePosition(IntakeConstants.floorPosition);
    intake.setRollerSpeed(IntakeConstants.floorSpeed);
    LimelightHelpers.setPipelineIndex("limelight-intake", 0);
    yawPIDController.setSetpoint(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(LimelightHelpers.getTV("limelight-intake")){
      double yawSpeed = yawPIDController.calculate(LimelightHelpers.getTX("limelight-intake"));
      if(triggerTranslation.getAsDouble() >= 0.3) {
        drivetrain.setControl(robotRequest.withRotationalRate(yawSpeed).withVelocityX(MathUtil.inverseInterpolate(0.3, 1.0, triggerTranslation.getAsDouble())).withVelocityY(translationY.getAsDouble()));
      }else{
        drivetrain.setControl(fieldRequest.withRotationalRate(yawSpeed).withVelocityX(translationX.getAsDouble()).withVelocityY(translationY.getAsDouble()));
      }
    }else{
      drivetrain.setControl(fieldRequest.withRotationalRate(rotation.getAsDouble()).withVelocityX(translationX.getAsDouble()).withVelocityY(translationY.getAsDouble()));
      System.out.println("note not found");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePosition(IntakeConstants.stowedPosition);
    intake.setRollerSpeed(IntakeConstants.stallSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(intake.isIntakeAtPosition(IntakeConstants.floorPosition) && intake.isRollerStalled(IntakeConstants.stallRPM)) {
      return true;
    }else{
      return false;
    }
  }
}
