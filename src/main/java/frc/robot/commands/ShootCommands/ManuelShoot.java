// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.generated.IntakeConstants;
import frc.robot.generated.ShooterConstants;

public class ManuelShoot extends Command {
  Intake intake;
  Shooter shooter;
  double speed;
  /** Creates a new ManuelShoot. */
  public ManuelShoot(Shooter shooter, Intake intake, double speed) {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.log("isIntakeAtPosition", intake.isIntakeAtPosition(IntakeConstants.shootPosition));
    shooter.log("isShooterAtSpeed", shooter.isShooterAtSpeed(ShooterConstants.trapShootRPM));
    //if(!shooter.isShooterAtSpeed(ShooterConstants.trapShootRPM))System.out.println("shooterNotAtSpeed");
    //if(!intake.isIntakeAtPosition(IntakeConstants.shootPosition))System.out.println("IntakeNotInPosition");
    if(shooter.isShooterAtSpeed(ShooterConstants.trapShootRPM) && intake.isIntakeAtPosition(IntakeConstants.shootPosition)) 
    {

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
