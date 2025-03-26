// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multiaction;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.ActuateIntakeUp;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAccuracyTest extends ParallelCommandGroup {
  /** Creates a new ThreeCoralAuto. */
  public AlignAccuracyTest(SwerveSubsystem swerve, Elevator elevator, CoralEffector effector, Intake intake, int reefId, int stationId) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ActuateIntakeUp(intake),
      Commands.sequence(
        new PlaceThenGetCoral(swerve, elevator, effector, reefId, stationId, true, 4),
        new PlaceThenGetCoral(swerve, elevator, effector, reefId, stationId, false, 4),
        new PlaceThenGetCoral(swerve, elevator, effector, reefId, stationId, true, 3),
        new PlaceThenGetCoral(swerve, elevator, effector, reefId, stationId, false, 3),
        new PlaceThenGetCoral(swerve, elevator, effector, reefId, stationId, true, 2),
        new PlaceThenGetCoral(swerve, elevator, effector, reefId, stationId, false, 2)
      )
    );
  }
  public AlignAccuracyTest(SwerveSubsystem swerve, Elevator elevator, CoralEffector effector, Intake intake) {
    this(swerve, elevator, effector, intake, 19, 13);
  }
}
