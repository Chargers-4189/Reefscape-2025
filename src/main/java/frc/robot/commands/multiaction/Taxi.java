// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multiaction;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.effector.IntakeCoral;
import frc.robot.commands.intake.ActuateIntakeUp;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.util.Elastic.ElasticTaxi;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Taxi extends ParallelCommandGroup {
  /** Creates a new Taxi. */
  public Taxi(SwerveSubsystem swerve, CoralEffector effector, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.runEnd(
        () -> swerve.drive(new Translation2d(ElasticTaxi.kX.get(), ElasticTaxi.kY.get()), 0, false),
        () -> swerve.drive(new Translation2d(0, 0), 0, false),
        swerve
      ).withTimeout(ElasticTaxi.kSECONDS.get()),
      new ActuateIntakeUp(intake),
      new IntakeCoral(effector)
    );
  }
}
