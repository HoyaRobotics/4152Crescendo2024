// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.generated.IntakeConstants;

public class IntakeFromGround extends Command {
  private final Intake intake;
  /** Creates a new Intake. */
  public IntakeFromGround(Intake intake) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakePosition(IntakeConstants.floorPosition);
    intake.setRollerSpeed(IntakeConstants.floorSpeed);
    System.out.println("Running Intake");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePosition(IntakeConstants.stowedPosition);
    intake.setRollerSpeed(IntakeConstants.stallSpeed);
    System.out.println("Stopping Intake");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.isRollerStalled();
  }
}
