// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HumanDriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.util.Elastic.ElasticTeleopDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  SwerveSubsystem swerve;
  DoubleSupplier x;
  DoubleSupplier y;
  DoubleSupplier angle;
  boolean nitro;
  boolean alignStation;

  double scaleFactor;

  /** Creates a new Drive. */
  public Drive(
    SwerveSubsystem swerve,
    DoubleSupplier x,
    DoubleSupplier y,
    DoubleSupplier angle,
    boolean nitro,
    boolean alignStation
  ) {
    this.swerve = swerve;
    this.x = x;
    this.y = y;
    this.angle = angle;
    this.nitro = nitro;
    this.alignStation = alignStation;
    if (nitro) {
      scaleFactor = 1;
    } else {
      scaleFactor = ElasticTeleopDrive.kDRIVE_POWER.get();
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    swerve.drive(
      new Translation2d(x.getAsDouble(), y.getAsDouble()),
      angle.getAsDouble(),
      true
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
