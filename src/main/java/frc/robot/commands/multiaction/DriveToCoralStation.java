// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multiaction;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.MoveElevatorSlightlyDown;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToCoralStation extends ParallelRaceGroup {
  /** Creates a new DriveToCoralStation. */
  public DriveToCoralStation(SwerveSubsystem swerveSubsystem, Elevator elevator, CoralEffector effector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeCoral(effector),
      new MoveElevatorSlightlyDown(elevator),
      swerveSubsystem.driveToAprilTag(13,0)
    );
  }
}
