// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Climber;
import frc.robot.generated.ClimberConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStage extends SequentialCommandGroup {
  /** Creates a new AutoStage. */
  public AutoStage(CommandSwerveDrivetrain drivetrain, Climber climber, String positionOnBlue) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      drivetrain.pathfindingCommand(drivetrain.stagePose(positionOnBlue)),
      new ClimberPosition(climber, ClimberConstants.climbingPosition),
      new ParallelDeadlineGroup(
        new WaitCommand(0.5), 
        new InstantCommand(()-> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0.2)), drivetrain)),
      new ClimberPosition(climber, ClimberConstants.readyToClimbPosition)
    );
  }
}
