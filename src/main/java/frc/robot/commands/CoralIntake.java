// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntake extends Command {

  private CoralEffector coraleffector;

  /**
   *
   *  If intake sensor is active, inputs coral until outtake sensor sees coral
   * @param coraleffector import coral subsystem
   */
  public CoralIntake(CoralEffector coraleffector) {
    this.coraleffector = coraleffector;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coraleffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (coraleffector.getIntakeSensor() == true) {
      coraleffector.set(0.1, 0.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coraleffector.set(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //(coraleffector.getOuttakeSensor() || !coraleffector.getIntakeSensor());
  }
}
