// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.Intake;
import frc.robot.generated.IntakeConstants;

public class IntakeStart extends Command {
  private final Intake intake;
  private final CommandSwerveDrivetrain drivetrain;
  private boolean tryAlign; //TODO test
  /** Creates a new IntakeStart. */
  public IntakeStart(Intake intake, CommandSwerveDrivetrain drivetrain, boolean tryAlign) {
    this.intake = intake;
    this.drivetrain = drivetrain;
    this.tryAlign = tryAlign;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight-intake", 1);
    intake.setIntakePosition(IntakeConstants.floorPosition);
    intake.setRollerSpeed(IntakeConstants.floorSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(tryAlign)
    {
     if(LimelightHelpers.getTV("limelight-intake")){
       double rotation = drivetrain.getState().Pose.getRotation().getDegrees()-LimelightHelpers.getTX("limelight-intake");
       Rotation2d rotationTarget = Rotation2d.fromDegrees(rotation);
       PPHolonomicDriveController.setRotationTargetOverride(()-> Optional.of(rotationTarget));
      }
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
