// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootCommands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.Intake;
import frc.robot.generated.IntakeConstants;
import frc.robot.generated.ShooterConstants;
import frc.robot.Subsystems.Shooter;

public class Shoot extends Command {
  //private final Shooter shooter;
  private final Intake intake;
  private final Shooter shooter;
  private final CommandSwerveDrivetrain drivetrain;

  private final DoubleSupplier translationX, translationY, rotation;
  private PIDController yawPIDController = new PIDController(0.1, 0.0, 0.0);
  //private PIDController distancePIDController = new PIDController(3.0, 0.3, 0.03); 
  private PIDController distancePIDController = new PIDController(0.5, 0, 0);
  

  private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric();
  private final SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric();

  private int targetTag;
  private int tagLostCount;
  private boolean tryFinding = false; //TODO test
  private double drivespeed = 0;
  private double turnSpeed = 0;
  /** Creates a new Shoot. 
 * @param shooter */
  public Shoot(Intake intake, Shooter shooter, CommandSwerveDrivetrain drivetrain, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    this.intake = intake;
    this.shooter = shooter;
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
    targetTag = DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?7:4;
    LimelightHelpers.setPipelineIndex("limelight-shooter", targetTag);
    yawPIDController.setTolerance(2);
    distancePIDController.setTolerance(0.15);
    yawPIDController.setSetpoint(ShooterConstants.xSetPoint);
    distancePIDController.setSetpoint(ShooterConstants.ySetPoint); //120 with old limelight
    distancePIDController.setIZone(1);
    shooter.setShooterSpeeds();
    //shooter.setShooterSpeeds(ShooterConstants.shootingRPM, ShooterConstants.spinFactor);
    intake.setIntakePosition(IntakeConstants.shootPosition);
    tagLostCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean hasTarget = LimelightHelpers.getTV("limelight-shooter");
    if(hasTarget) {
      turnSpeed = yawPIDController.calculate(LimelightHelpers.getTX("limelight-shooter"));
      drivespeed = distancePIDController.calculate(-LimelightHelpers.getTY("limelight-shooter"));
      SmartDashboard.putNumber("Shooting Drive Speed", drivespeed);
      drivetrain.setControl(driveRobot.withRotationalRate(turnSpeed).withVelocityX(drivespeed).withVelocityY(translationY.getAsDouble()));
    }else if(tryFinding){
      turnSpeed = yawPIDController.calculate(PhotonUtils.getYawToPose(drivetrain.getState().Pose, AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(targetTag).get().toPose2d()).getDegrees());
      drivetrain.setControl(driveRobot.withRotationalRate(turnSpeed).withVelocityX(0).withVelocityY(0));
    }else{
      if(tagLostCount<=5)
      {
        tagLostCount++;
      }
      else{
      drivetrain.setControl(driveField.withRotationalRate(rotation.getAsDouble()).withVelocityX(translationX.getAsDouble()).withVelocityY(translationY.getAsDouble()));
      }
    }
    /*System.out.println(shooter.isShooterAtSpeed(ShooterConstants.shootingRPM));
    System.out.println(intake.isIntakeAtPosition(IntakeConstants.shootPosition));
    System.out.println(yawPIDController.atSetpoint());
    System.out.println(distancePIDController.atSetpoint());
    System.out.println(hasTarget);
    System.out.println("loop end");*/

    if(shooter.isShooterAtSpeed(ShooterConstants.shootingRPM) && intake.isIntakeAtPosition(IntakeConstants.shootPosition) && yawPIDController.atSetpoint() && distancePIDController.atSetpoint() && hasTarget) 
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
    LimelightHelpers.setPipelineIndex("limelight-shooter", 0);
    //shooter.idleMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
