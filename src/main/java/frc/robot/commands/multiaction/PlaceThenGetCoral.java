// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multiaction;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.effector.IntakeCoral;
import frc.robot.commands.effector.OuttakeCoral;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.commands.elevator.MoveElevatorSlightlyDown;
import frc.robot.commands.swervedrive.AlignReef;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceThenGetCoral extends ParallelCommandGroup {
  /** Creates a new PlaceThenGetCoral. */
  public PlaceThenGetCoral(SwerveSubsystem swerve, Elevator elevator, CoralEffector effector, int reefId, int stationId, int level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Place Coral
      Commands.race(
        new IntakeCoral(effector),
        new AlignReef(swerve, true, reefId)
      ),

      new MoveElevator(elevator, level),
      new OuttakeCoral(effector),
      new MoveElevator(elevator, 0),
      //Go to Station
      Commands.parallel(
        new MoveElevatorSlightlyDown(elevator),
        swerve.driveToAprilTag(stationId, 0)
      ),
      //Wait for human player to give coral
      new IntakeCoral(effector).withTimeout(1)
    );
  }
}
