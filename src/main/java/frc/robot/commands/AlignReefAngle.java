// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.util.Stopwatch;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.VisionConstants;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignReefAngle extends Command {

  private SwerveSubsystem swerve;
  private Rotation2d rotationSetpoint;
  private DoubleSupplier tagId;
  
  private final NetworkTableInstance networkTable = NetworkTableInstance
    .getDefault()
    .getTable("SwerveSubsystem")
    .getInstance();
  private final StructPublisher<Pose2d> publisher = networkTable
    .getStructTopic("AngleCode", Pose2d.struct)
    .publish();
  
  /** Creates a new AlignReefAngle. */
  public AlignReefAngle(SwerveSubsystem swerve) {
    this.swerve = swerve;
    this.tagId = tagId;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationSetpoint = swerve.getClosestReefTagPose().getRotation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    publisher.set(swerve.getClosestReefTagPose());
    //System.out.println(rotationSetpoint);
    swerve.driveRotationSetpoint(0, 0, rotationSetpoint.getCos(), rotationSetpoint.getSin());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println("Finished?");
      //System.out.println("ALERT: FINISHED");
    return false;
    
      //System.out.println("Check Setpoint");
    //return Math.abs(swerve.getPose().getRotation().getDegrees() - rotationSetpoint.getDegrees()) < AlignmentConstants.kROTATION_TOLERANCE;
    
  }
}