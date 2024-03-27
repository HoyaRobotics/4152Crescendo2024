// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.Intake;
import frc.robot.generated.IntakeConstants;
import frc.robot.generated.ShooterConstants;
import frc.robot.Subsystems.Shooter;


public class AutoShoot extends Command {
  private final Intake intake;
  private final Shooter shooter;
  private final CommandSwerveDrivetrain drivetrain;

  //private PIDController yawPIDController = new PIDController(0.045, 0.0, 0.0);
  //private PIDController distancePIDController = new PIDController(3.0, 0.3, 0.03);
  private PIDController yawPIDController = new PIDController(0.1, 0.0, 0.0);
  private PIDController distancePIDController = new PIDController(0.5, 0, 0);
  private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric();
  private int targetTag;
  private double drivespeed = 0;
  private double turnSpeed = 0;

  private boolean finished = false;
  private boolean timeStampLock = true;
  private double shootTime = 0;
  private boolean tryFinding; //TODO test
  private Pose2d targetPose;

  /** Creates a new AutoShoot. */
  public AutoShoot(Intake intake, Shooter shooter, CommandSwerveDrivetrain drivetrain, boolean tryFinding) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    this.tryFinding = tryFinding;

    addRequirements(intake, shooter, drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetTag = DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?7:4;
    //System.out.println(Timer.getFPGATimestamp());
    targetPose = ShooterConstants.aprilTags.getTagPose(targetTag).get().toPose2d();
    //System.out.println(Timer.getFPGATimestamp());
    LimelightHelpers.setPipelineIndex("limelight-shooter", targetTag);
    yawPIDController.setTolerance(1);
    distancePIDController.setTolerance(0.15);
    yawPIDController.setSetpoint(ShooterConstants.xSetPoint);
    distancePIDController.setSetpoint(ShooterConstants.ySetPoint); //120 with old limelight
    distancePIDController.setIZone(1);
    shooter.setShooterSpeeds();
    //shooter.setShooterSpeeds(ShooterConstants.shootingRPM, ShooterConstants.spinFactor);
    intake.setIntakePosition(IntakeConstants.shootPosition);
    timeStampLock = true;
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean hasTarget = LimelightHelpers.getTV("limelight-shooter");
    if(hasTarget) {
      turnSpeed = yawPIDController.calculate(LimelightHelpers.getTX("limelight-shooter"));
      drivespeed = distancePIDController.calculate(-LimelightHelpers.getTY("limelight-shooter"));
      drivetrain.setControl(driveRobot.withRotationalRate(turnSpeed).withVelocityX(drivespeed).withVelocityY(0.0));
      //System.out.println("Dist" + distancePIDController.getPositionError());
      //System.out.println("Yaw" + yawPIDController.getPositionError());
    }else if(tryFinding){
      turnSpeed = yawPIDController.calculate(PhotonUtils.getYawToPose(drivetrain.getState().Pose, targetPose).getDegrees());
      drivetrain.setControl(driveRobot.withRotationalRate(turnSpeed).withVelocityX(0).withVelocityY(0));
      System.out.println(turnSpeed);
    }else{
      //System.out.println("TAG NOT FOUND");
      drivetrain.setControl(driveRobot.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
    }

    if(shooter.isShooterAtSpeed(ShooterConstants.shootingRPM) && intake.isIntakeAtPosition(IntakeConstants.shootPosition) && yawPIDController.atSetpoint() && distancePIDController.atSetpoint() && hasTarget) 
    {
      System.out.println("SHOOTING");
      intake.setRollerSpeed(IntakeConstants.shootSpeed);

      if(timeStampLock){
        shootTime = Timer.getFPGATimestamp();
        timeStampLock = false;
      }

      if(!timeStampLock && Timer.getFPGATimestamp() - shootTime > 0.1){ //0.4
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
