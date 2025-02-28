// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.util.Elastic.ElasticHumanDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopDrive extends Command {
  private SwerveSubsystem swerve;
  private CommandXboxController driveController;
  /** Creates a new TeleopDrive. */
  public TeleopDrive(SwerveSubsystem swerve, CommandXboxController driveController) {
    this.swerve = swerve;
    this.driveController = driveController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driveController.getRightTriggerAxis() > .5) {
      swerve.drive(
        Math.pow(-driveController.getLeftY(), ElasticHumanDrive.kDRIVE_EXPONENT.get()),
        Math.pow(-driveController.getLeftX(), ElasticHumanDrive.kDRIVE_EXPONENT.get()),
        Math.pow(-driveController.getRightX(), ElasticHumanDrive.kROTATIONAL_EXPONENT.get()),
        true
      );
    }
    swerve.drive(
        Math.pow(-driveController.getLeftY(), ElasticHumanDrive.kDRIVE_EXPONENT.get())
        * ElasticHumanDrive.kDRIVE_POWER.get(),
        Math.pow(-driveController.getLeftX(), ElasticHumanDrive.kDRIVE_EXPONENT.get())
        * ElasticHumanDrive.kDRIVE_POWER.get(),
        Math.pow(-driveController.getRightX(), ElasticHumanDrive.kROTATIONAL_EXPONENT.get())
        * ElasticHumanDrive.kROTATIONAL_POWER.get(),
        true
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
