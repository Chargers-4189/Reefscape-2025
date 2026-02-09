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
    if(levelwanted == 0){
      heightneeded = 0;
    }else if(levelwanted == 1){
      heightneeded = ElevatorConstants.kHEIGHTS[1];
    }else if(levelwanted == 2){
      heightneeded = ElevatorConstants.kHEIGHTS[2];
    }else if(levelwanted == 3){
      heightneeded = ElevatorConstants.kHEIGHTS[3];
    }else if(levelwanted == 4){
      heightneeded = ElevatorConstants.kHEIGHTS[4];
    }
    if(levelwanted != 0){
    elevator.ZeroEncoder();
    }
\

\"?"]
    //find level elevator is at (current level)
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("heightneeded: " + heightneeded);
    System.out.println("elevator.getEncoderValue(): " + elevator.getEncoderValue());
    
      if(elevator.getEncoderValue() < (heightneeded)){
        // it goes up until it hits the inch it wants, the 1 is tolerance so when it is closer to the target it slows doen to not damage itself or overshoot
        if((elevator.getEncoderValue() + 0.1) < (heightneeded)){
          System.out.println("one");
          elevator.moveElevator(ElevatorConstants.kGRAVITY_VOLTS + 0.2);
        }
        else{
          //slowing down because elevator is close to target
          System.out.println("two");
          elevator.moveElevator(ElevatorConstants.kGRAVITY_VOLTS + 0.01); 
        }
      }else if(elevator.getEncoderValue() > (heightneeded + (5 * 0.01))){
        //same as the one above but this one moves the elevator down
        if((elevator.getEncoderValue() - 0.1) > (heightneeded + (5 * 0.01))){
          elevator.moveElevator(ElevatorConstants.kGRAVITY_VOLTS - 0.1);
        }
        else{
          elevator.moveElevator(ElevatorConstants.kGRAVITY_VOLTS - 0.005);
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
  public boolean isFinished(){
    
    if(Math.abs(elevator.getEncoderValue() - heightneeded) <= 0.1){  // error is within tolerance constant from 0
      System.out.println(Math.abs(elevator.getEncoderValue() - heightneeded));
      return true;
     }else{
    return false;
    }
  }
}
