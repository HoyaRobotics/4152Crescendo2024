// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Photonvision;
import frc.robot.generated.IntakeConstants;
import frc.robot.generated.ShooterConstants;
import frc.robot.Subsystems.Shooter;

public class ShootOld extends Command {
  //private final Shooter shooter;
  private final Intake intake;
  private final Shooter shooter;
  private final Photonvision photonvision;
  private final CommandSwerveDrivetrain drivetrain;

  private final DoubleSupplier translationX, translationY, rotation;
  private PIDController yawPIDController = new PIDController(0.1, 0.0, 0.0);
  //private PIDController distancePIDController = new PIDController(3.0, 0.3, 0.03); 
  private PIDController distancePIDController = new PIDController(3.0, 0, 0);
  

  private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric();
  private final SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric();

  private int targetTag;
  private int tagLostCount;
  /** Creates a new Shoot. 
 * @param shooter */
  public ShootOld(Intake intake, Shooter shooter, Photonvision photonvision, CommandSwerveDrivetrain drivetrain, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    this.intake = intake;
    this.shooter = shooter;
    this.photonvision = photonvision;
    this.drivetrain = drivetrain;
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotation = rotation;
  
    //yawPIDController.setIZone(0);
   //distancePIDController.setIZone(0); //these can be useful for limiting 

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yawPIDController.setTolerance(2);
    distancePIDController.setTolerance(0.05);
    yawPIDController.setSetpoint(0.0);
    distancePIDController.setSetpoint(Units.inchesToMeters(ShooterConstants.shootPosition)-0.06); //120 with old limelight
    distancePIDController.setIZone(1);
    shooter.setShooterSpeeds();
    //shooter.setShooterSpeeds(ShooterConstants.shootingRPM, ShooterConstants.spinFactor);
    intake.setIntakePosition(IntakeConstants.shootPosition);
    targetTag = DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?7:4;
    tagLostCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(photonvision.doesTagExist(targetTag)) {
      double turnSpeed = yawPIDController.calculate(photonvision.getTagYaw(targetTag));
      double drivespeed = distancePIDController.calculate(photonvision.getTagDistance(targetTag));
      drivetrain.setControl(driveRobot.withRotationalRate(turnSpeed).withVelocityX(drivespeed).withVelocityY(translationY.getAsDouble()));
      SmartDashboard.putNumber("distance from target", photonvision.getTagDistance(targetTag));
    }else{
      if(tagLostCount<=5)
      {
        tagLostCount++;
      }
      else{
      drivetrain.setControl(driveField.withRotationalRate(rotation.getAsDouble()).withVelocityX(translationX.getAsDouble()).withVelocityY(translationY.getAsDouble()));
      }
    }
    if(shooter.isShooterAtSpeed(ShooterConstants.shootingRPM) && intake.isIntakeAtPosition(IntakeConstants.shootPosition) && yawPIDController.atSetpoint() && distancePIDController.atSetpoint() && photonvision.doesTagExist(targetTag)) 
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
    
    //shooter.idleMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
