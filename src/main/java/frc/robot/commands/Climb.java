// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;
<<<<<<< HEAD
import frc.robot.Subsystems.Elevator;

public class Climb extends Command {
    private final Climber climber;
    private final DoubleSupplier movePower;

  /** Creates a new Climb. */
  public Climb(Climber climber, DoubleSupplier movePower) {
    this.climber = climber;
    this.movePower = movePower;

    // Use addRequirements() here to declare subsystem dependencies.
=======

public class Climb extends Command {
  private final Climber climber;
  private final DoubleSupplier movePower;
  /** Creates a new Elevate. */
  public Climb(Climber climber, DoubleSupplier movePower) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.movePower = movePower;
>>>>>>> origin/main
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.moveClimber(movePower.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.moveClimber(0);
<<<<<<< HEAD

=======
>>>>>>> origin/main
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
