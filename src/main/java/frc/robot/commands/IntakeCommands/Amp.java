// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.generated.IntakeConstants;

public class Amp extends Command {
  private final Intake intake;
  private boolean finished = false;
  private boolean timeStampLock = true;
  private double shootTime = 0;
  /** Creates a new Amp. */
  public Amp(Intake intake) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakePosition(IntakeConstants.ampPosition);
    timeStampLock = true;
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("COMMAND RAN");
    if(intake.isIntakeAtPosition(IntakeConstants.ampPosition)) {
      if(timeStampLock){
        shootTime = Timer.getFPGATimestamp();
        timeStampLock = false;
      }

      if(!timeStampLock && Timer.getFPGATimestamp() - shootTime > 0.4){
        finished = true;
      }
      intake.setRollerSpeed(IntakeConstants.ampSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePosition(IntakeConstants.stowedPosition);
    intake.setRollerSpeed(IntakeConstants.stallSpeed);
    finished = false;
    System.out.println("COMMAND DONE");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
