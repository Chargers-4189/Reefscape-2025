// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorSlightlyDown extends Command {
  private Elevator elevator;
  private Timer timer = new Timer();
  /** Creates a new MoveElevatorSlightlyDown. */
  public MoveElevatorSlightlyDown(Elevator elevator) {
    this.elevator = elevator;
    
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    elevator.moveElevator(ElevatorConstants.kGRAVITY_VOLTS - 0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stayStill();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() >= ElevatorConstants.kSLIGHTLY_DOWN_TIMEOUT){
      return true;
    }else{
    return false;
  }
  }
}
