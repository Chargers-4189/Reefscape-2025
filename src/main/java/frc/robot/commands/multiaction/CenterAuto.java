// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multiaction;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.effector.IntakeCoral;
import frc.robot.commands.intake.ActuateIntakeUp;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.swervedrive.AlignReef;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterAuto extends ParallelCommandGroup {
  private int reefId;
  /** Creates a new ThreeCoralAuto. */
  public CenterAuto(SwerveSubsystem swerve, Elevator elevator, CoralEffector effector, Intake intake, boolean red, boolean alignRight) {


    if (red) {
      reefId = -1;
    } else {
      reefId = 18;
    }
    // Add your commands in the addCommands() call.
    addCommands(
      new ActuateIntakeUp(intake),
      Commands.sequence(
        Commands.race(
          new IntakeCoral(effector),
          new AlignReef(swerve, alignRight, reefId)
        ),
        new PlaceCoral(elevator, effector, 4)
      )
    );
  }
}
