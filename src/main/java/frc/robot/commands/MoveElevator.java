// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevator extends Command {
  
  private final Elevator elevator;
  private int level;
  private double goal;
  private double startTime;
  private boolean up;


  /**
   * Creates a new moveElevator command.
   *
   * @param level The level to move the elevator to. 0 moves to the intake.
   */
  public MoveElevator(Elevator elevator, int level) {

    this.elevator = elevator;
    this.level = level;
    this.goal = elevator.kHEIGHTS.get()[level];

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startTime = Timer.getFPGATimestamp();
    this.up = elevator.getEncoder() < goal;
  };

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var proportionalVoltage = Math.abs(goal - elevator.getEncoder()) * elevator.kPROPORTIONAL_VOLTS.get();
    if (level == 0) {
      proportionalVoltage = 10;
    }
    var maxVoltage = Math.min(elevator.kMAX_VOLTS.get(), (Timer.getFPGATimestamp() - startTime) * elevator.kMAX_VOLT_CHANGE_PER_SECOND.get());
    if (up) {
      elevator.setVoltage(Math.min(proportionalVoltage, maxVoltage));
    } else {
      elevator.setVoltage(-Math.min(proportionalVoltage, maxVoltage));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setLevel(level);
    elevator.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (level == 0) {
      return false;
    }
    if (up) {
      return elevator.getEncoder() > goal + elevator.kTOLERANCE.get();
    } else {
      return elevator.getEncoder() < goal - elevator.kTOLERANCE.get();
    }
  }
}
