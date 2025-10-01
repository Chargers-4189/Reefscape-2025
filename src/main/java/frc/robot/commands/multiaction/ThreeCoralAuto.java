// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multiaction;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.ActuateIntakeUp;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeCoralAuto extends ParallelCommandGroup {
  private int reefId1;
  private int reefId2;
  private int stationId;
  /** Creates a new ThreeCoralAuto. */
  public ThreeCoralAuto(SwerveSubsystem swerve, Elevator elevator, CoralEffector effector, Intake intake, boolean rightStart, boolean red) {


    if (red) {
      if (rightStart) {
        //red-right
        reefId1 = 9;
        reefId2 = 8;
        stationId = 2;
      } else {
        //red-left
        reefId1 = 11;
        reefId2 = 6;
        stationId = 1;
      }
    } else {
      if (rightStart) {
        //blue-right
        reefId1 = 22;
        reefId2 = 17;
        stationId = 12;
      } else {
        //blue-left
        reefId1 = 20;
        reefId2 = 19;
        stationId = 13;
      }
    }
    // Add your commands in the addCommands() call.
    addCommands(
      new ActuateIntakeUp(intake),
      Commands.sequence(
        new PlaceThenGetCoral(swerve, elevator, effector, reefId1, stationId, !rightStart),
        new PlaceThenGetCoral(swerve, elevator, effector, reefId2, stationId, rightStart),
        new PlaceThenGetCoral(swerve, elevator, effector, reefId2, stationId, !rightStart)
      )
    );
  }
}
