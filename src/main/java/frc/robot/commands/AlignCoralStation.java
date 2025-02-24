// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignCoralStation extends Command {
  private final Swerve swerve;
  private final Vision vision;
  private Transform3d tagPose;
  private Pose2d tagGoal;
  private Pose2d lastPos;
  private Pose2d toTravel;

  /** Creates a new autoIntake. */
  public AlignCoralStation(Vision vision, Swerve swerve) {
    this.vision = vision;
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tagPose = vision.getBTagPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.getBTagPose() != null) {
        tagPose = vision.getBTagPose();
        tagGoal = new Pose2d().transformBy(new Transform2d(tagPose.getX(), tagPose.getY(),
            new Rotation2d(tagPose.getRotation().getX(), tagPose.getRotation().getY())));
        lastPos = swerve.getPose();
      }
      if (lastPos != null) {
        toTravel = tagGoal.relativeTo(swerve.getPose().relativeTo(lastPos));
      } else {
        toTravel = tagGoal;
      }

      if (toTravel != null) {
        swerve.drive(-toTravel.getX() * SwerveConstants.kAlignSpeedX, -toTravel.getY() * SwerveConstants.kAlignSpeedY, 0.0, false);
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
return false;  }
}
