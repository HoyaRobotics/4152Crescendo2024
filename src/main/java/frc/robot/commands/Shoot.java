// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Photonvision;
import frc.robot.generated.IntakeConstants;
import frc.robot.Subsystems.Shooter;

public class Shoot extends Command {
  //private final Shooter shooter;
  private final Intake intake;
  private final Shooter shooter;
  private final Photonvision photonvision;
  private final CommandSwerveDrivetrain drivetrain;

  private PIDController yawPIDController = new PIDController(0, 0, 0); 
  private PIDController distancePIDController = new PIDController(0, 0, 0); 

  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
  /** Creates a new Shoot. 
 * @param shooter */
  public Shoot(Intake intake, Shooter shooter, Photonvision photonvision, CommandSwerveDrivetrain drivetrain) {
    //this.shooter = shooter;
    this.intake = intake;
    this.shooter = shooter;
    this.photonvision = photonvision;
    this.drivetrain = drivetrain;
    //shooter.setShooterSpeeds(0, 0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yawPIDController.setTolerance(1);
    distancePIDController.setTolerance(0.4);
    shooter.setShooterSpeeds();
    intake.setIntakePosition(IntakeConstants.stowedPosition);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(photonvision.doesTagExist(7)) {
      double turnSpeed = yawPIDController.calculate(photonvision.getTagYaw(7));
      double drivespeed = distancePIDController.calculate(photonvision.getTagDistance(7));
      drivetrain.setControl(drive.withRotationalRate(turnSpeed).withVelocityX(drivespeed));
    }else{
      //Manual Drive!
    }
    if(shooter.isShooterAtSpeed() && intake.isIntakeAtPosition(IntakeConstants.stowedPosition) && yawPIDController.atSetpoint()) 
    {
      intake.setRollerSpeed(IntakeConstants.shootSpeed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePosition(IntakeConstants.stowedPosition);
    intake.setRollerSpeed(IntakeConstants.stallSpeed);
    //shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
