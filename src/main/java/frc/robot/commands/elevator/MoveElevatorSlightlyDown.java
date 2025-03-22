// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.util.Stopwatch;
import frc.util.Elastic.ElasticElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorSlightlyDown extends Command {
  private Elevator elevator;
  private Stopwatch stopwatch = new Stopwatch();
  /** Creates a new MoveElevatorSlightlyDown. */
  public MoveElevatorSlightlyDown(Elevator elevator) {
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopwatch.start(ElasticElevator.kSLIGHTLY_DOWN_TIMEOUT.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setVoltageNoGravity(-.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopwatch.hasTriggered();
  }
}
