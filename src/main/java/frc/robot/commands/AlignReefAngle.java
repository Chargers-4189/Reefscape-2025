// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignReefAngle extends Command {

  private SwerveSubsystem swerve;
  private Vision vision;
  private boolean alignRight;
  private Transform3d tagPose;
  private Pose2d tagGoal;
  private Pose2d lastPos;
  private Pose2d toTravel;
  
  /** Creates a new AlignReefAngle. */
  public AlignReefAngle(SwerveSubsystem swerve, Vision vision, boolean alignRight) {
    this.swerve = swerve;
    this.vision = vision;
    this.alignRight = alignRight;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
