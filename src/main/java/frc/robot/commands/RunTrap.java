// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Trap;

public class RunTrap extends Command {
  private final Trap trap;
  private final DoubleSupplier trapSpeed;
  /** Creates a new RunTrap. */
  public RunTrap(Trap trap, DoubleSupplier trapSpeed) {
    this.trap = trap;
    this.trapSpeed = trapSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(trap);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    trap.setTrapSpeed(trapSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trap.setTrapSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
