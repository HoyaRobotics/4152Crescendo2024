// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.commands.AlignWithChain;
import frc.robot.commands.AlignWithPodium;
import frc.robot.commands.ChangeClimbPosition;
import frc.robot.commands.ChangeLights;
import frc.robot.commands.DownElevator;
import frc.robot.commands.ForwardTrap;
import frc.robot.commands.Outake;
import frc.robot.commands.ReverseTrap;
import frc.robot.commands.Shoot;
import frc.robot.commands.SwitchDriveMode;
import frc.robot.commands.UpElevator;
import frc.robot.commands.rollIntake;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0); // My driverController
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final Shooter shooter = new Shooter();
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    driverController.start().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.back().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    
    drivetrain.seedFieldRelative();

//BUTTON ASSIGNING BELOW//
driverController.a().whileTrue(new Outake(intake));
driverController.b().whileTrue(new rollIntake(intake));
driverController.y().onTrue(new SwitchDriveMode());

operatorController.a().onTrue(new AlignWithPodium());
operatorController.b().onTrue(new AlignWithChain());
operatorController.y().onTrue(new ChangeClimbPosition(climber));
operatorController.rightBumper().whileTrue(new Shoot(shooter));
operatorController.rightTrigger().whileTrue(new ForwardTrap());
operatorController.leftTrigger().whileTrue(new ReverseTrap());
operatorController.povUp().onTrue(new UpElevator(climber));
operatorController.povDown().onTrue(new DownElevator(climber));
operatorController.rightStick().onTrue(new ChangeLights());

  //not working yet
    //driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    //NM here - the above line was working, but all it does is give a button that resets field orientation based on current orienation.
    //the cause of the original issue may be due to the wierd coordinate system. 
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
