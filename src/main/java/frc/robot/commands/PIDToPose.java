// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;

public class PIDToPose extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Pose2d endPose;
  private final PIDController yawController = new PIDController(0.1, 0, 0);
  private final PIDController xController = new PIDController(3.5, 0, 0.02);
  private final PIDController yController = new PIDController(3.5, 0, 0.02);

  private boolean finished = false;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  /** Creates a new PIDToPose. */
  public PIDToPose(CommandSwerveDrivetrain drivetrain, Pose2d endPose) {
    this.drivetrain = drivetrain;
    this.endPose = endPose;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    yawController.setSetpoint(endPose.getRotation().getDegrees());
    yawController.setTolerance(2.0);
    xController.setSetpoint(endPose.getX());
    xController.setTolerance(Units.inchesToMeters(2.0));
    yController.setSetpoint(endPose.getY());
    yController.setTolerance(Units.inchesToMeters(2.0));
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yawSpeed = yawController.calculate(drivetrain.getState().Pose.getRotation().getDegrees());
    yawSpeed = MathUtil.clamp(yawSpeed, -3.8, 3.8);
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    if(Math.abs(yawController.getPositionError()) < 5) {
      xSpeed = xController.calculate(drivetrain.getState().Pose.getX());
      xSpeed = MathUtil.clamp(xSpeed, -4.2, 4.2);
      ySpeed = yController.calculate(drivetrain.getState().Pose.getY());
      ySpeed = MathUtil.clamp(ySpeed, -4.2, 4.2);
    }
    drivetrain.setControl(drive.withRotationalRate(yawSpeed).withVelocityX(xSpeed).withVelocityY(ySpeed));

    if(xController.atSetpoint() && yController.atSetpoint()) {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
    finished = false;
    xController.reset();
    yController.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
