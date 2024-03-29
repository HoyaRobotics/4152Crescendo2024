// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Trap;
import frc.robot.generated.ElevatorConstants;
import frc.robot.generated.TrapConstants;

public class TrapFeedShot extends Command {
  private double scoreTime;
  private final Trap trap;
  private final Elevator elevator;
  /** Creates a new TrapFeedShot. */
  public TrapFeedShot(Trap trap, Elevator elevator) {
    this.trap = trap;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(trap);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trap.setTrapSpeed(TrapConstants.trapAmpScoreSpeed);
    scoreTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trap.setTrapSpeed(0.0);
    elevator.setElevatorPosition(ElevatorConstants.elevatorStowedPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() - scoreTime > TrapConstants.trapScoreTime) {
      return true;
    }else{
      return false;
    }
  }
}
