// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.generated.IntakeConstants;
import frc.robot.generated.ShooterConstants;

public class AutoManuelShoot extends Command {
  Intake intake;
  Shooter shooter;
  double speed;

  private boolean finished = false;
  private boolean timeStampLock = true;
  private double shootTime = 0;
  /** Creates a new ManuelShoot. */
  public AutoManuelShoot(Shooter shooter, Intake intake, double speed) {
    this.shooter = shooter;
    this.intake = intake;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterSpeeds(speed, 0.0);
    //shooter.setShooterSpeeds(ShooterConstants.trapShootRPM, 0.0);
    intake.setIntakePosition(IntakeConstants.shootPosition);
    timeStampLock = true;
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!shooter.isShooterAtSpeed(ShooterConstants.trapShootRPM))System.out.println("shooterNotAtSpeed");
    if(!intake.isIntakeAtPosition(IntakeConstants.shootPosition))System.out.println("IntakeNotInPosition");
    if(shooter.isShooterAtSpeed(ShooterConstants.trapShootRPM) && intake.isIntakeAtPosition(IntakeConstants.shootPosition)) 
    {

      intake.setRollerSpeed(IntakeConstants.shootSpeed);
      if(timeStampLock){
        shootTime = Timer.getFPGATimestamp();
        timeStampLock = false;
      }

      if(!timeStampLock && Timer.getFPGATimestamp() - shootTime > 0.4){
        finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePosition(IntakeConstants.stowedPosition);
    intake.setRollerSpeed(IntakeConstants.stallSpeed);
    //shooter.stopShooter();
    finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
