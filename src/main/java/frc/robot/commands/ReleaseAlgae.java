// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEffector;
import frc.util.Stopwatch;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReleaseAlgae extends Command {
  private CoralEffector coraleffector;
  private Stopwatch stopwatch = new Stopwatch();
  /** Creates a new ReleaseAlgae. */
  public ReleaseAlgae(CoralEffector coraleffector) {
    this.coraleffector = coraleffector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coraleffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopwatch.start(2000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coraleffector.outtakeAlgae(0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coraleffector.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopwatch.hasTriggered();
  }
}
