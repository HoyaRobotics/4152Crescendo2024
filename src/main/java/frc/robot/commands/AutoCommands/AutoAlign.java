// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.Intake;

public class AutoAlign extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Intake intake;
  /** Creates a new AutoAlign. */
  public AutoAlign(CommandSwerveDrivetrain drivetrain, Intake intake) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight-intake", 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(LimelightHelpers.getTV("limelight-intake")){
      double limelighMeasurement = LimelightHelpers.getTX("limelight-intake");
      limelighMeasurement = MathUtil.inverseInterpolate(-29.8, 29.8, limelighMeasurement);
      limelighMeasurement = MathUtil.interpolate(-20, 20, limelighMeasurement);
      double rotation = drivetrain.getState().Pose.getRotation().getDegrees()-limelighMeasurement;
      Rotation2d rotationTarget = Rotation2d.fromDegrees(rotation);
      PPHolonomicDriveController.setRotationTargetOverride(()-> Optional.of(rotationTarget));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
