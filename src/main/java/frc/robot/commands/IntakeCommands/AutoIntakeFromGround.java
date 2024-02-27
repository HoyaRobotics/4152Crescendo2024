// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.Intake;
import frc.robot.generated.IntakeConstants;

public class AutoIntakeFromGround extends Command {
  private final Intake intake;
  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;
  private final DoubleSupplier rotation;

  private PIDController yawPIDController = new PIDController(0.1, 0.0, 0.0);

  private SwerveRequest.FieldCentric request;

  /** Creates a new Intake. */
  public AutoIntakeFromGround(Intake intake, CommandSwerveDrivetrain drivetrain,DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation, SwerveRequest.FieldCentric request) {
    this.intake = intake;
    this.drivetrain = drivetrain;
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotation = rotation;
    this.request = request;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakePosition(IntakeConstants.floorPosition);
    intake.setRollerSpeed(IntakeConstants.floorSpeed);

    yawPIDController.setSetpoint(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(LimelightHelpers.getTV("")){
      double yawSpeed = yawPIDController.calculate(LimelightHelpers.getTX(""));
      drivetrain.setControl(request.withRotationalRate(yawSpeed).withVelocityX(translationX.getAsDouble()).withVelocityY(translationY.getAsDouble()));
    }else{
      drivetrain.setControl(request.withRotationalRate(rotation.getAsDouble()).withVelocityX(translationX.getAsDouble()).withVelocityY(translationY.getAsDouble()));
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
