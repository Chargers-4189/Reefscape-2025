// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.util.Networker.NetworkAlign;
import edu.wpi.first.math.MathUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignReefPosition extends Command {

  private SwerveSubsystem swerve;
  private Vision vision;
  private boolean alignRight;
  private Transform3d tagPose;
  private Pose2d tagGoal;
  private Pose2d lastPos;
  private Pose2d toTravel;

  /** Creates a new AutoAlignPose. */
  public AlignReefPosition(SwerveSubsystem swerve, Vision vision, boolean alignRight) {
    this.swerve = swerve;
    this.vision = vision;
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
      if (vision.getFLTagPose() != null) {
        tagPose = vision.getFLTagPose();
        
      }
    } else {
      if (vision.getFRTagPose() != null) {
        tagPose = vision.getFRTagPose();
      }
    }
    
    tagGoal =
      new Pose2d()
        .transformBy(
          new Transform2d(
            tagPose.getX() - AlignmentConstants.kDIST_FROM_REEF,
            tagPose.getY(),
            new Rotation2d(
              tagPose.getRotation().getX(),
              tagPose.getRotation().getY()
            )
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
        MathUtil.clamp(-toTravel.getX() * NetworkAlign.kPROPORTIONAL_X.get(), -NetworkAlign.kMAX_SPEED_X.get(), NetworkAlign.kMAX_SPEED_X.get()),
        MathUtil.clamp(-toTravel.getY() * NetworkAlign.kPROPORTIONAL_Y.get(), -NetworkAlign.kMAX_SPEED_Y.get(), NetworkAlign.kMAX_SPEED_Y.get()),
        0,
        false
      );
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (alignRight) {
      if (vision.getFLTagPose() != null) {
        return (
          vision.getFLTagPose().getX() <= 0.15 &&
          vision.getFLTagPose().getY() <= 0.05
        );
      }
    } else {
      if (vision.getFRTagPose() != null) {
        return (
          vision.getFRTagPose().getX() <= 0.15 &&
          vision.getFRTagPose().getY() <= 0.05
        );
      }
    }
    try{
      return toTravel.getX() <= 0.15 && toTravel.getY() <= 0.05;
    } catch(Exception e){
      return true;
    }
  }
}
