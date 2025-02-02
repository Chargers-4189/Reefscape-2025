// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.YAGSLDrivetrain;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  private final YAGSLDrivetrain yagslDrivetrain;
  private boolean alignDirectionRight;
  
  /** Creates a new AutoAlign. */
  public AutoAlign(YAGSLDrivetrain yagslDrivetrain) {
    this.yagslDrivetrain = yagslDrivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(yagslDrivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(/* Left button is pressed */){
      alignDirectionRight = false;
    }
    else if(/*right button is pressed */){
    alignDirectionRight = true;
    }
    else{
      System.out.println("Was not able to get align direction");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(alignDirectionRight == true){
      //align it right
    }
    else{
      //align it left
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
