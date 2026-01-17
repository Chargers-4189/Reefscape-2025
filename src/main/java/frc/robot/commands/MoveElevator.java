// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevator extends Command {
  /** Creates a new MoveElevator. */
  private Elevator elevator;
  private double currentlevel;
  private int levelwanted;
  private double heightneeded;


  public MoveElevator(int levelwanted, Elevator elevator) {
    this.elevator = elevator;
    this.levelwanted = levelwanted;


    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
      if(currentlevel < heightneeded){
        if((currentlevel + 1) < levelwanted){
          elevator.moveElevator(0.4);
        }
        else{
          elevator.moveElevator(0.05); //emm/caleb/bryan what do you reccommend for these values?
        }
      }else if(currentlevel > heightneeded){
        if((currentlevel - 1) > levelwanted){
          elevator.moveElevator(-0.4);
        }
        else{
          elevator.moveElevator(-0.05);
        }
      }
      if(levelwanted == 0){
        if(elevator.getTopLimitSwitch() /= "true"){
          elevator.moveElevator(-0.4);
        }
      }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
