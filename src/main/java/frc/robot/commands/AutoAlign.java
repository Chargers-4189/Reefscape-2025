// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  private final SwerveSubsystem swerve;
  private final Vision vision;
  private boolean alignRight;
  
  /** Creates a new AutoAlign. */
  public AutoAlign(SwerveSubsystem swerve, Vision vision, boolean alignRight) {
    this.swerve = swerve;
    this.vision = vision;
    this.alignRight = alignRight;
    
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
    if(alignRight == true){
      if(vision.getFrontRightTagYaw() > 0){
        swerve.driveCommand(()->-1.0 * 0.1, ()->0.0, ()->0.0, false);
      }
      else{
        swerve.driveCommand(()->1.0 * 0.1, ()->0.0, ()->0.0, false);
      }
    }
    else{
      if(vision.getFrontLeftTagYaw() > 0){
        swerve.driveCommand(()->-1.0 * 0.1, ()->0.0, ()->0.0, false);
      }
      else{
        swerve.driveCommand(()->1.0 * 0.1, ()->0.0, ()->0.0, false);
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.driveCommand(()->0.0, ()->0.0, ()->0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(alignRight == true){
      return (vision.getFrontRightTagYaw() <= 0.0698132 && vision.getFrontRightTagYaw() >= -0.0698132 );
    }
    else{
      return (vision.getFrontLeftTagYaw() <= 0.0698132 && vision.getFrontLeftTagYaw() >= -0.0698132 );
    }
  }
}
