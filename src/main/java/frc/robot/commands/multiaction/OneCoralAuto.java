// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multiaction;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.ActuateIntakeUp;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneCoralAuto extends ParallelCommandGroup {
  private int reefId;
  private int stationId;
  /** Creates a new ThreeCoralAuto. */
  public OneCoralAuto(SwerveSubsystem swerve, Elevator elevator, CoralEffector effector, Intake intake, boolean rightStart, boolean red) {


    if (red) {
      if (rightStart) {
        //red-right
        reefId = 9;
        stationId = 2;
      } else {
        //red-left
        reefId = 11;
        stationId = 1;
      }
    } else {
      if (rightStart) {
        //blue-right
        reefId = 20;
        stationId = 22;
      } else {
        //blue-left
        reefId = 20;
        stationId = 13;
      }
    }
    // Add your commands in the addCommands() call.
    addCommands(
      new ActuateIntakeUp(intake),
      new PlaceThenGetCoral(swerve, elevator, effector, reefId, stationId, !rightStart)
    );
  }
}
