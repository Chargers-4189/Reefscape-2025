// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralactuator.actuatebase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralactuator.Elevator;
import frc.robot.Constants.CoralActuatorConstants.ElevatorConstants;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevator extends Command {

  private final Elevator elevator;
  private double goal;
  private double startTime;
  private boolean up;
  private int level;
  private Timer stopwatch = new Timer();

  /**
   * Creates a new moveElevator command.
   *
   * @param level The level to move the elevator to. 0 = intake, 1-4 = corresponding level, 5 = algae low, 6 = algae high
   */
  public MoveElevator(Elevator elevator, int level) {
    this.elevator = elevator;
    this.level = level;
    elevator.setLevel(level);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startTime = Timer.getFPGATimestamp();
    if (level == -1) {
      //this.goal = ElevatorConstants.kTEST_HEIGHT;
    } else {
      this.goal = ElevatorConstants.kHEIGHTS[level];
    }
    this.up = elevator.getEncoder() < goal;
    stopwatch.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var proportionalVoltage =
      Math.abs(goal - elevator.getEncoder()) *
      ElevatorConstants.kPROPORTIONAL_VOLTS;
    var maxVoltage = Math.min(
      ElevatorConstants.kMAX_VOLTS,
      (Timer.getFPGATimestamp() - startTime) *
      ElevatorConstants.kMAX_VOLT_CHANGE_PER_SECOND
    );
    if (up) {
      elevator.setVoltage(Math.min(proportionalVoltage, maxVoltage));
    } else {
      elevator.setVoltage(-Math.min(proportionalVoltage, maxVoltage));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setVoltage(0);
    //System.out.print(goal);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (stopwatch.hasElapsed(ElevatorConstants.kTIMEOUT)) {
      return true;
    }
    if (up) {
      return elevator.getEncoder() > goal - ElevatorConstants.kTOLERANCE;
    } else {
      return elevator.getEncoder() < goal + ElevatorConstants.kTOLERANCE;
    }
  }
}
