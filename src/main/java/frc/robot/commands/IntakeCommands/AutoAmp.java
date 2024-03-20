// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.commands.PIDToPose;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAmp extends SequentialCommandGroup {
  Pose2d blueAmpPose = new Pose2d(1.83, 7.78, Rotation2d.fromDegrees(90.0));
  /** Creates a new AutoAmp. */
  public AutoAmp(Intake intake, CommandSwerveDrivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //drivetrain.pathfindingCommand(blueAmpPose, new PathConstraints(2.00, 3.0, Units.degreesToRadians(90), Units.degreesToRadians(360.0))),
      new PIDToPose(drivetrain, blueAmpPose),
      new Amp(intake) 
    );
  }
}
