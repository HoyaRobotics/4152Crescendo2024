  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Limelight;
import frc.robot.commands.AutoStage;
import frc.robot.commands.Climb;
import frc.robot.commands.Elevate;
import frc.robot.commands.HoldClimber;
import frc.robot.commands.HoldElevator;
import frc.robot.commands.RunTrap;
import frc.robot.commands.TrapAmpShot;
import frc.robot.commands.TrapScoring;
import frc.robot.commands.AutoCommands.AutoAlign;
import frc.robot.commands.AutoCommands.AutoShoot;
import frc.robot.commands.AutoCommands.IntakeStart;
import frc.robot.commands.AutoCommands.IntakeStop;
import frc.robot.commands.AutoCommands.ShootPoseAuto;
import frc.robot.commands.IntakeCommands.*;
import frc.robot.commands.ShootCommands.ManuelShoot;
import frc.robot.commands.ShootCommands.ShootDeflect;
import frc.robot.commands.ShootCommands.ShootPose;
import frc.robot.generated.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.OtherConstants.stageLocation;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Trap;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0); // My driverController
  private final CommandXboxController operatorController = new CommandXboxController(1);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  private final Elevator elevator = new Elevator();
  private final Trap trap = new Trap();
  private final Limelight limelight = new Limelight(drivetrain, "limelight-shooter");
  


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 2% deadband
      .withSteerRequestType(SteerRequestType.MotionMagic)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final Telemetry logger = new Telemetry(MaxSpeed);


  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    //BUTTON ASSIGNING BELOW//
    driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d())));
    driverController.rightTrigger().whileTrue(new IntakeFromGroundOld(intake));
    driverController.rightBumper().whileTrue(new AutoIntakeFromGround(intake,drivetrain,()-> -driverController.getLeftY() * MaxSpeed, ()-> -driverController.getLeftX() * MaxSpeed, ()-> -driverController.getRightX() * MaxAngularRate, drive));
    //driverController.leftBumper().whileTrue(new Shoot(intake, shooter, drivetrain, ()-> -driverController.getLeftY() * MaxSpeed, ()-> -driverController.getLeftX() * MaxSpeed, ()-> -driverController.getRightX() * MaxAngularRate));
    driverController.leftBumper().whileTrue(new ShootPose(drivetrain, shooter, intake));
    driverController.leftTrigger().whileTrue(new ManuelShoot(shooter, intake, ShooterConstants.shootingRPM));
    //driverController.b().whileTrue(new Amp(intake)); // Disabled for the trapshot test
    driverController.b().onTrue(new TrapAmpShot(elevator, intake, shooter, trap));
    driverController.povLeft().whileTrue(new AutoStage(drivetrain, climber, stageLocation.leftStage));
    driverController.povLeft().onFalse(new InstantCommand(()-> drivetrain.setControl(new SwerveRequest.Idle()), drivetrain));
    driverController.povUp().whileTrue(new AutoStage(drivetrain, climber, stageLocation.centerStage));
    driverController.povUp().onFalse(new InstantCommand(()-> drivetrain.setControl(new SwerveRequest.Idle()), drivetrain));
    driverController.povRight().whileTrue(new AutoStage(drivetrain, climber, stageLocation.rightStage));
    driverController.povRight().onFalse(new InstantCommand(()-> drivetrain.setControl(new SwerveRequest.Idle()), drivetrain));
    driverController.rightStick().whileTrue(new AutoAmp(intake, drivetrain));
    driverController.a().whileTrue(new ShootDeflect(shooter, intake, elevator, ShooterConstants.deflectSpeed));
    operatorController.a().whileTrue(new Climb(climber, ()-> -operatorController.getLeftY())).onFalse(new HoldClimber(climber));
    operatorController.y().whileTrue(new Elevate(elevator, ()-> operatorController.getLeftY())).onFalse(new HoldElevator(elevator));
    operatorController.x().whileTrue(new RunTrap(trap, ()-> operatorController.getLeftY()));
    operatorController.leftBumper().onTrue(new IntakeDownClimb(intake));
    operatorController.back().onTrue(new InstantCommand(()-> climber.resetEncoder(true), climber));
    operatorController.rightBumper().onTrue(new TrapScoring(elevator, intake, shooter, trap));
    operatorController.rightTrigger().whileTrue(new TryHarder(intake));

    //driverController.povDown().whileTrue(new ShootDeflect(shooter, intake, elevator));
    //driverController.povUp().whileTrue(new ShootFromIntake(intake));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    
    //TODO remove line after this. It's being used for testing purposes
    //drivetrain.seedFieldRelative();
  }

  public RobotContainer() {

    NamedCommands.registerCommand("startIntake", new IntakeStart(intake, drivetrain, false));
    NamedCommands.registerCommand("stopIntake", new IntakeStop(intake));
    //NamedCommands.registerCommand("autoShoot", new AutoShoot(intake,shooter, drivetrain, false));
    NamedCommands.registerCommand("autoShoot", new ShootPoseAuto(drivetrain, shooter, intake));
    NamedCommands.registerCommand("autoIntake", new IntakeStart(intake, drivetrain, false));
    NamedCommands.registerCommand("autoShootAlign", new AutoShoot(intake, shooter,drivetrain, true));
    NamedCommands.registerCommand("autoAlign", new AutoAlign(drivetrain, intake));
    NamedCommands.registerCommand("deflect", new ShootDeflect(shooter, intake, elevator, ShooterConstants.deflectSpeed));
    autoChooser = AutoBuilder.buildAutoChooser();
    configureBindings();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void configureAutoSettings() {
    shooter.setShooterSpeeds();
    limelight.useLimelight(false);
  }

  public void configureTeleopSettings() {
    //shooter.idleMotor();
    shooter.stopShooter();
    limelight.useLimelight(true);
  }
}
