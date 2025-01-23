// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DrivePIDController extends Command {

  private final DriveSubsystem driveSubsystem;
  private final CommandXboxController xboxController;
  private final PIDController swerveFeedback = new PIDController(0.0, 0.0, 0.0);

  /** Creates a new DrivePIDController. */
  public DrivePIDController(
    DriveSubsystem driveSubsystem,
    CommandXboxController xboxController
  ) {
    swerveFeedback.enableContinuousInput(-180, 180);
    this.driveSubsystem = driveSubsystem;
    this.xboxController = xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.drive(
      -MathUtil.applyDeadband(
        xboxController.getLeftY(),
        OperatorConstants.kDriveDeadband
      ),
      -MathUtil.applyDeadband(
        xboxController.getLeftX(),
        OperatorConstants.kDriveDeadband
      ),
      MathUtil.clamp(
        swerveFeedback.calculate(
          driveSubsystem.getHeading(),
          Math.atan2(xboxController.getRightY(), xboxController.getRightX())
        ),
        -0.3,
        0.3
      ),
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
