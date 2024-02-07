// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.generated.IntakeConstants;
import frc.robot.Subsystems.Shooter;

public class Shoot extends Command {
  //private final Shooter shooter;
  private final Intake intake;
  private final Shooter shooter;
  /** Creates a new Shoot. 
 * @param shooter */
  public Shoot(Intake intake, Shooter shooter) {
    //this.shooter = shooter;
    this.intake = intake;
    this.shooter = shooter;
    //shooter.setShooterSpeeds(0, 0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterSpeeds(0, 0);
    intake.setIntakePosition(IntakeConstants.stowedPosition);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.isShooterAtSpeed()) 
    {
      System.out.println("at speed");
      intake.setRollerSpeed(IntakeConstants.shootSpeed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePosition(IntakeConstants.stowedPosition);
    intake.setRollerSpeed(IntakeConstants.stallSpeed);
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
