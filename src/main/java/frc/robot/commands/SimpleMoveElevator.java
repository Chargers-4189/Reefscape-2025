// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimpleMoveElevator extends Command {

  private final Elevator elevatorSubsystem;
  private static double desiredHeight;
  private static int level;

  /** Creates a new SimpleMoveElevator. */
  public SimpleMoveElevator(Elevator elevatorSubsystem, int level) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.level = level;
    this.desiredHeight = ElevatorConstants.kHEIGHTS_METERS[level];

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setVoltage(
      Math.pow((elevatorSubsystem.getHeightMeters() - desiredHeight), 1 / 3)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(desiredHeight - elevatorSubsystem.getHeightMeters()) < .05;
  }
}
