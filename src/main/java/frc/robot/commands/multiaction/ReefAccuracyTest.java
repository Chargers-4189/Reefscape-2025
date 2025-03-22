// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multiaction;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReefAccuracyTest extends SequentialCommandGroup {
  /** Creates a new ThreeCoralAuto. */
  public ReefAccuracyTest(SwerveSubsystem swerveSubsystem, Elevator elevator, CoralEffector effector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      swerveSubsystem.driveToAprilTag(19, new Translation2d(1.8,0)),
      swerveSubsystem.driveToReef(19, true),
      new AutoPlaceCoral(elevator, effector, 4),

      new GetThenPlaceCoral(swerveSubsystem, elevator, effector, 13, 19, false, 4),
      new GetThenPlaceCoral(swerveSubsystem, elevator, effector, 13, 19, true, 3),
      new GetThenPlaceCoral(swerveSubsystem, elevator, effector, 13, 19, false, 3),
      new GetThenPlaceCoral(swerveSubsystem, elevator, effector, 13, 19, true, 2),
      new GetThenPlaceCoral(swerveSubsystem, elevator, effector, 13, 19, false, 2)
    );
  }
}
