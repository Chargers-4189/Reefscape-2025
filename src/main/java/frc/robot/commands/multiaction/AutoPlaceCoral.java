// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multiaction;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveElevatorSlightlyDown;
import frc.robot.commands.OuttakeCoral;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceCoral extends SequentialCommandGroup {

  /** Creates a new AutoPlaceCoral. */
  public AutoPlaceCoral(Elevator elevator, CoralEffector effector, int level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveElevator(elevator, level),
      new OuttakeCoral(effector),
      new MoveElevator(elevator, 0),
      new MoveElevatorSlightlyDown(elevator)
    );
  }
}
