// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Photonvision;
import frc.robot.generated.IntakeConstants;
import frc.robot.generated.ShooterConstants;
import frc.robot.Subsystems.Shooter;


public class AutoShoot extends Command {
  private final Intake intake;
  private final Shooter shooter;
  private final Photonvision photonvision;
  private final CommandSwerveDrivetrain drivetrain;

  private PIDController yawPIDController = new PIDController(0.045, 0.0, 0.0);
  private PIDController distancePIDController = new PIDController(3.0, 0.3, 0.03);
  private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric();
  private int targetTag;

  private boolean finished = false;
  private boolean timeStampLock = true;
  private double shootTime = 0;

  /** Creates a new AutoShoot. */
  public AutoShoot(Intake intake, Shooter shooter, Photonvision photonvision, CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.shooter = shooter;
    this.photonvision = photonvision;
    this.drivetrain = drivetrain;

    addRequirements(intake, shooter, drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yawPIDController.setTolerance(2);
    distancePIDController.setTolerance(0.05);
    yawPIDController.setSetpoint(0.0);
    distancePIDController.setSetpoint(Units.inchesToMeters(ShooterConstants.shootPosition));
    distancePIDController.setIZone(1);
    shooter.setShooterSpeeds();
    intake.setIntakePosition(IntakeConstants.shootPosition);
    targetTag = DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?7:4;
    timeStampLock = true;
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(photonvision.doesTagExist(targetTag)) {
      double turnSpeed = yawPIDController.calculate(photonvision.getTagYaw(targetTag));
      double drivespeed = distancePIDController.calculate(photonvision.getTagDistance(targetTag));
      drivetrain.setControl(driveRobot.withRotationalRate(turnSpeed).withVelocityX(drivespeed).withVelocityY(0.0));
      //System.out.println("Dist" + distancePIDController.getPositionError());
      //System.out.println("Yaw" + yawPIDController.getPositionError());
    }else{
      System.out.println("TAG NOT FOUND");
    }

    if(shooter.isShooterAtSpeed() && intake.isIntakeAtPosition(IntakeConstants.shootPosition) && yawPIDController.atSetpoint() && distancePIDController.atSetpoint() && photonvision.doesTagExist(targetTag)) 
    {
      System.out.println("SHOOTING");
      intake.setRollerSpeed(IntakeConstants.shootSpeed);

      if(timeStampLock){
        shootTime = Timer.getFPGATimestamp();
        timeStampLock = false;
      }

      if(!timeStampLock && Timer.getFPGATimestamp() - shootTime > 1){
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
