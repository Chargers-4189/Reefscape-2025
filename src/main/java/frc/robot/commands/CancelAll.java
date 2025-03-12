// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CancelAll extends Command {

  private CoralEffector effector;
  private Elevator elevator;
  private Intake intake;

  /** Creates a new CancelAll. */
  public CancelAll(
    CoralEffector effector,
    Elevator elevator,
    Intake intake
  ) {
    this.effector = effector;
    this.elevator = elevator;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(effector, elevator, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    effector.stop();
    elevator.setVoltage(0);
    intake.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
