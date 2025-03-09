// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.util.Stopwatch;
import frc.robot.Constants.AlignmentConstants;
import frc.util.GetAprilTagRotation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignReefAngle extends Command {

  private SwerveSubsystem swerve;
  private Vision vision;
  private boolean alignRight;
  private int tagId;
  private Stopwatch stopwatch = new Stopwatch();
  private Rotation2d rotationSetpoint;
  
  
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
  public void initialize() {
    stopwatch.start(AlignmentConstants.kROTATION_TIMEOUT);
    rotationSetpoint = new Rotation2d(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (alignRight) {
      tagId = vision.getFRTagId();
    } else {
      tagId = vision.getFLTagId();
    }
    try {
      rotationSetpoint = GetAprilTagRotation.getReefTagAngle(tagId);
    } catch (Exception e) {
      System.out.print(e);
      this.cancel();
    }
    //System.out.println(rotationSetpoint);
    swerve.drive(0, 0, rotationSetpoint);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println("Finished?");
    if (stopwatch.hasTriggered()) {
      //System.out.println("ALERT: FINISHED");
      return true;
    } else {
      //System.out.println("Check Setpoint");
      return Math.abs(swerve.getPose().getRotation().getDegrees() - rotationSetpoint.getDegrees()) < AlignmentConstants.kROTATION_TOLERANCE;
    }
  }
}