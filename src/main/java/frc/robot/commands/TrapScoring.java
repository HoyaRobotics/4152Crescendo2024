// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Trap;
import frc.robot.commands.IntakeCommands.IntakeDownClimb;
import frc.robot.commands.IntakeCommands.IntakeUpClimb;
import frc.robot.generated.ElevatorConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrapScoring extends SequentialCommandGroup {
  /** Creates a new TrapScoring. */
  public TrapScoring(Elevator elevator, Intake intake, Shooter shooter, Trap trap) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ElevatorPosition(elevator, ElevatorConstants.elevatorHandoffPosition),
      new TrapHandoff(intake, shooter, elevator, trap),
      new IntakeDownClimb(intake),
      new ElevatorPosition(elevator, ElevatorConstants.trapPosition),
      new DepositTrapNote(elevator, trap),
      new IntakeUpClimb(intake),
      new ElevatorPosition(elevator, ElevatorConstants.elevatorStowedPosition)
    );
  }
}
