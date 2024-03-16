// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.generated.ElevatorConstants;
import frc.robot.generated.IntakeConstants;
import frc.robot.generated.ShooterConstants;

public class ShootDeflect extends Command {
  private final Shooter shooter;
  private final Intake intake;
  private final Elevator elevator;
  /** Creates a new ShootDeflect. */
  public ShootDeflect(Shooter shooter, Intake intake, Elevator elevator) {
    this.shooter = shooter;
    this.intake = intake;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterSpeeds();
    //shooter.setShooterSpeeds(ShooterConstants.trapHandoffRPM, 0.0);
    intake.setIntakePosition(IntakeConstants.shootPosition);
    elevator.setElevatorPosition(ElevatorConstants.ShootDeflect);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.isShooterAtSpeed(ShooterConstants.shootingRPM) && elevator.isElevatorAtPosition(ElevatorConstants.ShootDeflect) && intake.isIntakeAtPosition(IntakeConstants.shootPosition)) {
      intake.setRollerSpeed(IntakeConstants.shootSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePosition(IntakeConstants.stowedPosition);
    intake.setRollerSpeed(IntakeConstants.stallSpeed);
    shooter.stopShooter();
    elevator.setElevatorPosition(ElevatorConstants.elevatorStowedPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
