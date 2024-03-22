// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Climber;
import frc.robot.generated.ClimberConstants;
import frc.robot.generated.OtherConstants.stageLocation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStage extends SequentialCommandGroup {
  /** Creates a new AutoStage. */
  public AutoStage(CommandSwerveDrivetrain drivetrain, Climber climber, stageLocation position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      drivetrain.pathfindingCommand(position, new PathConstraints(4.00, 3.0, Units.degreesToRadians(90.0), Units.degreesToRadians(360.0))),
      new ClimberPosition(climber, ClimberConstants.climbingPosition),
      new PrintCommand("Start Drive"),
      new ParallelDeadlineGroup(
        new WaitCommand(1.0), 
        new DriveTillCancel(drivetrain)),
      new PrintCommand("End Drive"),
      new ClimberPosition(climber, ClimberConstants.readyToClimbPosition)
    );
  }
}
