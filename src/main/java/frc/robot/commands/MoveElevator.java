// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevator extends Command {
  /** Creates a new MoveElevator. */
  private Elevator elevator;
  private CoralEffector coraleffector;
  private int levelwanted;
  private double heightneeded;
  private boolean coralscored;


  public MoveElevator(int levelwanted, Elevator elevator, CoralEffector coraleffector) {
    this.elevator = elevator;
    this.coraleffector = coraleffector;
    this.levelwanted = levelwanted;


    addRequirements(elevator);
    addRequirements(coraleffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //goes from the level wanted to the amount of inches the bot needs to travel
    if(levelwanted == 1){
      heightneeded = ElevatorConstants.kHEIGHTS[2];
    }else if(levelwanted == 2){
      heightneeded = ElevatorConstants.kHEIGHTS[3];
    }else if(levelwanted == 3){
      heightneeded = ElevatorConstants.kHEIGHTS[4];
    }else if(levelwanted == 4){
      heightneeded = ElevatorConstants.kHEIGHTS[5];
    }

    //find level elevator is at (current level)
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(levelwanted == 0){
        //If you want the elevator at the bottom, it goes if the bottom limit switch is false
        if(!elevator.getBottomLimitSwitch()){     
          elevator.moveElevator(ElevatorConstants.kGRAVITY_VOLTS - 0.5);   //WHY IS TOLERANCE NOT BUILT INTO THIS QQUUIINNNN!!!!
      }
      if(elevator.getEncoderValue() < (heightneeded - ElevatorConstants.kTOLERANCE_IN)){
        // it goes up until it hits the inch it wants, the 1 is tolerance so when it is closer to the target it slows doen to not damage itself or overshoot
        if((elevator.getEncoderValue() + 1) < (heightneeded - ElevatorConstants.kTOLERANCE_IN)){
          elevator.moveElevator(ElevatorConstants.kGRAVITY_VOLTS + 0.5);
        }
        else{
          //slowing down because elevator is close to target
          elevator.moveElevator(ElevatorConstants.kGRAVITY_VOLTS + 0.01); 
        }
      }else if(elevator.getEncoderValue() > (heightneeded + ElevatorConstants.kTOLERANCE_IN)){
        //same as the one above but this one moves the elevator down
        if((elevator.getEncoderValue() - 1) > (heightneeded + ElevatorConstants.kTOLERANCE_IN)){
          elevator.moveElevator(ElevatorConstants.kGRAVITY_VOLTS - 0.5);
        }
        else{
          elevator.moveElevator(ElevatorConstants.kGRAVITY_VOLTS - 0.01);
        }
      }
    }

    if(Math.abs(elevator.getEncoderValue() - heightneeded) < ElevatorConstants.kTOLERANCE_IN || Math.abs(elevator.getEncoderValue() + heightneeded) < ElevatorConstants.kTOLERANCE_IN || levelwanted == 0 && !elevator.getBottomLimitSwitch()){
      if(coraleffector.getAnalogSensor()){
        //output coral
        coraleffector.moveEffector(); 
      }else{
        //puts target back to 0 to reset and move elevator to the bottom
        coralscored = true;
        levelwanted = 0;
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stayStill();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(elevator.getEncoderValue() - heightneeded) < ElevatorConstants.kTOLERANCE_IN || Math.abs(elevator.getEncoderValue() + heightneeded) < ElevatorConstants.kTOLERANCE_IN || levelwanted == 0 && !elevator.getBottomLimitSwitch()){
      if(coralscored == true){ // if elevator is within tolerance of target and coral is scores than return true
      return true;
      }else{
    return false;
    }}else{
    return false;
    }
  }
}
