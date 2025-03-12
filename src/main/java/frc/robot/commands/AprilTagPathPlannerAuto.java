// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralactuator.actuatebase.MoveElevator;
import frc.robot.commands.coralactuator.actuatebase.MoveElevatorSlightlyDown;
import frc.robot.subsystems.coralactuator.Elevator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AprilTagPathPlannerAuto extends SequentialCommandGroup {
  /** Creates a new AprilTagPathPlannerAuto. */
  public AprilTagPathPlannerAuto(SwerveSubsystem swerveSubsystem, Elevator elevator, int apriltagnumber, boolean right, int level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
      addCommands(
        swerveSubsystem.driveToAprilTag(apriltagnumber, new Translation2d(1,0)),
        swerveSubsystem.driveToReef(apriltagnumber, right),
        //new MoveElevator(elevator, level),
        //new MoveElevator(elevator, 0),
        //new MoveElevatorSlightlyDown(elevator),
        swerveSubsystem.driveToAprilTag(13,0)
        //swerveSubsystem.driveToPose(aprilTagFieldLayout.getTagPose(12).get().toPose2d())
      );
  }
}
