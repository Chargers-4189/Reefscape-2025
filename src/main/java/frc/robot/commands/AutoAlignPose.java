// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignPose extends Command {
  private SwerveSubsystem swerve;
  private Vision vision;
  private boolean alignRight;
  private Transform3d tagPose;
  private Pose2d tagGoal;
  private Pose2d lastPos;
  private Pose2d toTravel;

  /** Creates a new AutoAlignPose. */
  public AutoAlignPose(SwerveSubsystem swerve, Vision vision, boolean alignRight) {
    this.swerve = swerve;
    this.vision = vision;
    this.alignRight = alignRight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (alignRight) {
      tagPose = vision.getFLTagPose();
    } else {
      tagPose = vision.getFRTagPose();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (alignRight) {
      //System.out.print("alignRight  ");
      if (vision.getFLTagPose() != null) {
        //System.out.println(vision.getFLTagPose());
        tagPose = vision.getFLTagPose();
        tagGoal = new Pose2d(tagPose.getX(),tagPose.getY(), new Rotation2d(tagPose.getRotation().getX(), tagPose.getRotation().getY()));
        lastPos = swerve.getPose();
        //System.out.println(tagGoal);
      }
    } else {
      //System.out.print("alignLeft  ");
      if (vision.getFRTagPose() != null) {
        //System.out.println(vision.getFRTagPose());
        tagPose = vision.getFRTagPose();
        tagGoal = new Pose2d(tagPose.getX(),tagPose.getY(), new Rotation2d(tagPose.getRotation().getX(), tagPose.getRotation().getY()));
        lastPos = swerve.getPose();
        //System.out.print(tagGoal);
      }
    }
    if (lastPos != null) {
      toTravel = tagGoal.relativeTo(swerve.getPose().relativeTo(lastPos));
    } else {
      toTravel = tagGoal;
    }
    
    //System.out.print(toTravel.getX());
    if (toTravel != null) {
      //swerve.drive(toTravel.getX() * .005, toTravel.getY() * .005, 0.0, false);
      System.out.print(toTravel.getX());
      System.out.print(" ");
      System.out.print(toTravel.getY());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
