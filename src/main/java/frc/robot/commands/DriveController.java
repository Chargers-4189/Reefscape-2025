// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveController extends Command {
  DriveSubsystem driveSubsystem;
  CommandXboxController xboxController;
  /** Creates a new DriveController. */
  public DriveController(
    DriveSubsystem driveSubsystem,
    CommandXboxController xboxController
  ) {
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
      -MathUtil.applyDeadband(
        xboxController.getRightX(),
        OperatorConstants.kDriveDeadband
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