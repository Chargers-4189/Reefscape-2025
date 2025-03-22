// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.util.Elastic.ElasticAlign;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignReefPosition extends Command {

  private Vision vision;
  private SwerveSubsystem swerve;
  private boolean alignRight;
  private Transform3d tagPose;
  private Pose2d tagGoal;
  private Pose2d lastPos;
  private Pose2d toTravel;

  /** Creates a new AutoAlignPose. */
  public AlignReefPosition(
    Vision vision,
    SwerveSubsystem swerve,
    boolean alignRight
  ) {
    this.vision = vision;
    this.swerve = swerve;
    this.alignRight = alignRight;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Align Reef Pose");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (alignRight) {
      if (vision.getLeftTagPose() != null) {
        tagPose = vision.getLeftTagPose();
      }
    } else {
      if (vision.getRightTagPose() != null) {
        tagPose = vision.getRightTagPose();
      }
    }

    tagGoal =
      new Pose2d()
        .transformBy(
          new Transform2d(
            -tagPose.getX() - AlignmentConstants.kDIST_FROM_REEF,
            -tagPose.getY(),
            new Rotation2d()
          )
        );
    lastPos = swerve.getPose();

    if (lastPos != null) {
      toTravel = tagGoal.relativeTo(swerve.getPose().relativeTo(lastPos));
    } else {
      toTravel = tagGoal;
    }
    System.out.print(tagGoal + " ");

    System.out.println(toTravel);
    if (toTravel != null) {
      swerve.drive(
        new Translation2d(
          MathUtil.clamp(
            -toTravel.getX() * ElasticAlign.kPROPORTIONAL_X.get(),
            -ElasticAlign.kMAX_SPEED_X.get(),
            ElasticAlign.kMAX_SPEED_X.get()
          ),
          MathUtil.clamp(
            -toTravel.getY() * ElasticAlign.kPROPORTIONAL_Y.get(),
            -ElasticAlign.kMAX_SPEED_Y.get(),
            ElasticAlign.kMAX_SPEED_Y.get()
          )
        ),
        0,
        false
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, false);
  }

  // Returns true when the command should end.P
  @Override
  public boolean isFinished() {
    if (alignRight) {
      if (vision.getLeftTagPose() != null) {
        return (
          vision.getLeftTagPose().getX() <= 0.15 &&
          vision.getLeftTagPose().getY() <= 0.05
        );
      }
    } else {
      if (vision.getRightTagPose() != null) {
        return (
          vision.getRightTagPose().getX() <= 0.15 &&
          vision.getRightTagPose().getY() <= 0.05
        );
      }
    }
    try {
      return toTravel.getX() <= 0.15 && toTravel.getY() <= 0.05;
    } catch (Exception e) {
      return true;
    }
  }
}
