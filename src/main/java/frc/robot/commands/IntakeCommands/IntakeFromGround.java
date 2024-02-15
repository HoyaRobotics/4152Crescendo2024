// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.generated.IntakeConstants;

public class IntakeFromGround extends Command {
  private final Intake intake;
  double stallTime = 0.0;
  boolean stalled = false;
  boolean finished = false;
  double oldTime = 0;
  double currentTime = 0;
  /** Creates a new Intake. */
  public IntakeFromGround(Intake intake) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    stalled = false;
    stallTime = 0.0;
    intake.setIntakePosition(IntakeConstants.floorPosition);
    intake.setRollerSpeed(IntakeConstants.floorSpeed);
    //System.out.println("Running Intake");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = Timer.getFPGATimestamp();
    if(intake.isRollerStalled())
    {
      if(stalled)
      {
        stallTime += currentTime-oldTime;
        if(stallTime >= IntakeConstants.stallTriggerTime)
        {
           finished = true;
           //System.out.println("intake finished, or it should be. Stalled for: " + stallTime);
        }
      }
      stalled = true;
    }
    else{
      //if(stalled) System.out.println("stalled for: " + stallTime);
      stallTime = 0;
      stalled = false;
      
    }
    oldTime = currentTime;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePosition(IntakeConstants.stowedPosition);
    intake.setRollerSpeed(IntakeConstants.stallSpeed);
    //System.out.println("Stopping Intake");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
