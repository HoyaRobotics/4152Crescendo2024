// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootCommands;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.generated.IntakeConstants;
import frc.robot.generated.ShooterConstants;

public class ShootPose extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Shooter shooter;
  private final Intake intake;

  //private final TrapezoidProfile.Constraints yawConstraints = new Constraints(0.5, 0.5);
  //private final TrapezoidProfile.Constraints distanceConstraints = new Constraints(2, 3);
  //private final ProfiledPIDController yawController = new ProfiledPIDController(1, 0, 0, yawConstraints);
  //private final ProfiledPIDController distanceController = new ProfiledPIDController(1, 0, 0, distanceConstraints);
  //private final SimpleMotorFeedforward yawFeedforward = new SimpleMotorFeedforward(0, 2.71/12, 0.18/12);
  //private final SimpleMotorFeedforward distanceFeedforward = new SimpleMotorFeedforward(0, 2.71/12, 0.18/12);
  private final PIDController yawController = new PIDController(0.06, 0, 0);
  private final PIDController distanceController = new PIDController(0, 0, 0);
  //private final SlewRateLimiter yawFilter = new SlewRateLimiter(0.5);
  //private final SlewRateLimiter distanceFilter = new SlewRateLimiter(1.0);

  private Rotation2d rotationToBlue;
  private double distanceToBlue;

  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

  private int targetTag;
  private Pose2d targetPose;
  /** Creates a new ShootPose. */
  public ShootPose(CommandSwerveDrivetrain drivetrain, Shooter shooter, Intake intake) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //shooter.setShooterSpeeds();
    intake.setIntakePosition(IntakeConstants.shootPosition);
    targetTag = DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?7:4;
    //yawController.setGoal(0);
    //distanceController.setGoal(Units.inchesToMeters(120));
    yawController.setSetpoint(180.0);
    yawController.enableContinuousInput(-180, 180);
    distanceController.setSetpoint(Units.inchesToMeters(100));
    targetPose = ShooterConstants.aprilTags.getTagPose(targetTag).get().toPose2d();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;
    rotationToBlue = PhotonUtils.getYawToPose(currentPose, targetPose);
    distanceToBlue = PhotonUtils.getDistanceToPose(currentPose, targetPose);
    SmartDashboard.putNumber("Yaw To Target", rotationToBlue.getDegrees());
    SmartDashboard.putNumber("Distance To Target", distanceToBlue);

    double yawSpeed = yawController.calculate(-rotationToBlue.getDegrees());
    double distanceSpeed = yawController.calculate(distanceToBlue);

    SmartDashboard.putNumber("Yaw Speed", yawSpeed);
    SmartDashboard.putNumber("Distance Speed", distanceSpeed);


    /*double yawSpeed = yawController.calculate(rotationToBlue.getDegrees())
      + yawFeedforward.calculate(yawController.getSetpoint().velocity);

    double distanceSpeed = distanceController.calculate(distanceToBlue)
      + distanceFeedforward.calculate(distanceController.getSetpoint().velocity);*/

    drivetrain.setControl(drive.withRotationalRate(yawSpeed).withVelocityX(0).withVelocityY(0));

    if(yawController.atSetpoint() && distanceController.atSetpoint() && intake.isIntakeAtPosition(IntakeConstants.shootPosition) && shooter.isShooterAtSpeed(ShooterConstants.shootingRPM)) {
      intake.setRollerSpeed(IntakeConstants.shootSpeed);
    };
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
