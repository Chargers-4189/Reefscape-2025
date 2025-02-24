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
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignReef extends Command {
  private Swerve swerve;
  private Vision vision;
  private boolean alignRight;
  private Transform3d tagPose;
  private Pose2d tagGoal;
  private Pose2d lastPos;
  private Pose2d toTravel;

  /** Creates a new AutoAlignPose. */
  public AlignReef(Swerve swerve, Vision vision, boolean alignRight) {
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
      if (vision.getFLTagPose() != null) {
        tagPose = vision.getFLTagPose();
        tagGoal = new Pose2d().transformBy(new Transform2d(tagPose.getX(), tagPose.getY(),
            new Rotation2d(tagPose.getRotation().getX(), tagPose.getRotation().getY())));
        lastPos = swerve.getPose();
      }
      
    } else {
      if (vision.getFRTagPose() != null) {
        tagPose = vision.getFRTagPose();
        tagGoal = new Pose2d().transformBy(new Transform2d(tagPose.getX(), tagPose.getY(),
            new Rotation2d(tagPose.getRotation().getX(), tagPose.getRotation().getY())));
        lastPos = swerve.getPose();
      }
    }
    if (lastPos != null) {
      toTravel = tagGoal.relativeTo(swerve.getPose().relativeTo(lastPos));
    } else {
      toTravel = tagGoal;
    }
    System.out.println(toTravel);
    if (toTravel != null) {
      swerve.driveWithAngleSetPoint(-toTravel.getX() * SwerveConstants.kAlignSpeedX, -toTravel.getY() * SwerveConstants.kAlignSpeedY, 0);
    }
    
    /*
    if (vision.getFLTagPose() != null) {
      tagPose3d = vision.getFLTagPose();
      tagPose = new Pose2d(tagPose3d.getX(), tagPose3d.getY(), tagPose3d.getRotation().toRotation2d());
      tagField = swerve.getPose().relativeTo(tagPose.rotateBy(swerve.getPose().getRotation()));
      lastPos = swerve.getPose();
    }*/

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
