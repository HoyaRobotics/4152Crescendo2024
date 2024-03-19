// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;

public class ClimberPosition extends Command {
  private final Climber climber;
  private double desiredPosition;
  /** Creates a new RaiseClimber. */
  public ClimberPosition(Climber climber, double desiredPosition) {
    this.climber = climber;
    this.desiredPosition = desiredPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setClimberPosition(desiredPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.isClimberAtPosition(desiredPosition);
  }
}
