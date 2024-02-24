// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.generated.IntakeConstants;

public class AutoIntakeFromGround extends Command {
  private final Intake intake;
  private final CommandSwerveDrivetrain drivetrain;
  double stallTime = 0.0;
  boolean stalled = false;
  boolean finished = false;
  double oldTime = 0;
  double currentTime = 0;
  double hOffset = 0;
  double vOffset = 0;
  double turnSpeed = 0;
  double driveSpeed = 0;
  DoubleSupplier translationX;
  DoubleSupplier translationY;
  DoubleSupplier rotation;

  private PIDController yawPIDController = new PIDController(0.1, 0.0, 0.0);
  private PIDController distancePIDController = new PIDController(3.0, 0.3, 0.03); 

  private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric();
  private final SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric();

  /** Creates a new Intake. */
  public AutoIntakeFromGround(Intake intake, CommandSwerveDrivetrain drivetrain,DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    this.intake = intake;
    this.drivetrain = drivetrain;
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotation = rotation;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    stalled = false;
    stallTime = 0.0;
    intake.setIntakePosition(IntakeConstants.floorPosition);
    intake.setRollerSpeed(IntakeConstants.floorSpeed);
    //System.out.println("Running Intake");

    yawPIDController.setTolerance(2);
    distancePIDController.setTolerance(0.05);
    yawPIDController.setSetpoint(0.0);
    distancePIDController.setSetpoint(Units.inchesToMeters(120));
    distancePIDController.setIZone(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //move and angle to note

    //check if limelight has a target
    if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false))
    {
      //get the x and y offset in degrees
      hOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      vOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
      turnSpeed = yawPIDController.calculate(hOffset);
    }
    else turnSpeed = rotation.getAsDouble();
   // driveSpeed = distancePIDController.calculate(0);
    drivetrain.setControl(driveField.withRotationalRate(turnSpeed).withVelocityX(translationX.getAsDouble()).withVelocityY(translationY.getAsDouble()));
    currentTime = Timer.getFPGATimestamp();
    if(intake.isRollerStalled())
    {
      if(stalled)
      {
        stallTime += currentTime-oldTime;
        if(stallTime >= IntakeConstants.stallTriggerTime)
        {
           finished = true;
           //System.out.println("intake finished, or it should be. Stalled for: " + stallTime);
        }
      }
      stalled = true;
    }
    else{
      //if(stalled) System.out.println("stalled for: " + stallTime);
      stallTime = 0;
      stalled = false;
      
    }
    oldTime = currentTime;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePosition(IntakeConstants.stowedPosition);
    intake.setRollerSpeed(IntakeConstants.stallSpeed);
    //System.out.println("Stopping Intake");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
