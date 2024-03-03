// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Trap;
import frc.robot.generated.IntakeConstants;
import frc.robot.generated.TrapConstants;

public class TrapHandoff extends Command {
  private final Intake intake;
  private final Shooter shooter;
  private final Elevator elevator;
  private final Trap trap;
  private double handoffTime;

  /** Creates a new TrapHandoff. */
  public TrapHandoff(Intake intake, Shooter shooter, Elevator elevator, Trap trap) {
    this.intake = intake;
    this.shooter = shooter;
    this.elevator = elevator;
    this.trap = trap;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter, elevator, trap);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setTrapShooterSpeeds();
    intake.setIntakePosition(IntakeConstants.shootPosition);
    intake.setRollerSpeed(IntakeConstants.trapSpeed);
    elevator.setElevatorPosition(1);
    trap.setTrapSpeed(0.1);
    handoffTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //shooter.stopShooter();
    shooter.idleMotor();
    intake.setIntakePosition(IntakeConstants.stowedPosition);
    intake.setRollerSpeed(IntakeConstants.stallRPM);
    trap.setTrapSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() - handoffTime > TrapConstants.trapHandoffTime) {
      return true;
    }else{
      return false;
    }
  }
}
